#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>
#include <EEPROM.h>

// ─── Configuration ──────────────────────────────────────────
#define IMU_ADDRESS    0x68
#define EEPROM_MAGIC   0xA5          // Magic byte for IMU calibration
#define EEPROM_ADDR    0             // EEPROM start address for IMU cal
#define JOY_MAGIC      0xB6          // Magic byte for joystick calibration
#define JOY_EEPROM     60            // EEPROM start for joystick cal
#define LOOP_RATE_HZ   500           // Filter update rate
#define SERIAL_DIV     5             // Send serial every Nth cycle (100Hz)
#define LOOP_INTERVAL  (1000000UL / LOOP_RATE_HZ)

// ─── Joystick Pins ──────────────────────────────────────────
#define JOY_X_PIN      A0            // VRx → yaw / rudder
#define JOY_Y_PIN      A1            // VRy → throttle
#define JOY_SW_PIN     2             // Joystick built-in button → mode toggle
#define EXT_BTN_PIN    3             // External button

// ─── Control Mapping ────────────────────────────────────────
static const float MAX_TILT_DEG   = 30.0f;  // ±30° = full deflection
static const float EXPO           = 0.3f;   // Expo curve: 0=linear, 1=full cubic
static const float JOY_DEADZONE   = 0.08f;  // 8% deadzone around center
static const float RAD2DEG_F      = 57.29578f;

// ─── Mahony Filter Gains ────────────────────────────────────
static const float KpStable  = 2.0f;
static const float KpDynamic = 0.2f;
static const float Ki = 0.005f;
static const float ACCEL_REJECT_THRESH = 0.15f;

// ─── Sensor Objects ─────────────────────────────────────────
MPU6500 IMU;
calData calib = { 0 };
AccelData accelData;
GyroData gyroData;

// ─── Mahony Filter State ────────────────────────────────────
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float filteredAccelMag = 1.0f;

// ─── Joystick State ─────────────────────────────────────────
int16_t joyCenterX = 512;           // Calibrated center (ADC value)
int16_t joyCenterY = 512;
float joyYaw      = 0.0f;           // Normalized -1..1
float joyThrottle = 0.0f;           // Normalized  0..1

// ─── Button / Mode State ────────────────────────────────────
bool assistMode     = false;        // false=manual, true=stabilized
bool lastSwState    = HIGH;         // For debounce (active LOW w/ pullup)
unsigned long lastSwTime = 0;
#define DEBOUNCE_MS 200

// ─── Control Outputs ────────────────────────────────────────
float rollCmd  = 0.0f;              // -1..1  from IMU
float pitchCmd = 0.0f;              // -1..1  from IMU

// ─── Timing ─────────────────────────────────────────────────
unsigned long lastMicros = 0;
uint8_t serialCounter = 0;

// ═══════════════════════════════════════════════════════════
//  Expo Curve — RC-style control feel
//  output = (1 - expo) * input + expo * input³
// ═══════════════════════════════════════════════════════════

float applyExpo(float input, float expo) {
  return (1.0f - expo) * input + expo * input * input * input;
}

// ═══════════════════════════════════════════════════════════
//  Deadzone — zero out small values near center
// ═══════════════════════════════════════════════════════════

float applyDeadzone(float input, float deadzone) {
  if (fabsf(input) < deadzone) return 0.0f;
  // Rescale remaining range so output starts from 0 outside deadzone
  float sign = (input > 0.0f) ? 1.0f : -1.0f;
  return sign * (fabsf(input) - deadzone) / (1.0f - deadzone);
}

// ═══════════════════════════════════════════════════════════
//  Mahony AHRS Update (6-DOF, no magnetometer)
// ═══════════════════════════════════════════════════════════

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  float recipNorm;

  // ── Low-pass filtered accel magnitude ──
  float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  filteredAccelMag = 0.95f * filteredAccelMag + 0.05f * accelMag;

  bool accelValid = (filteredAccelMag > 0.01f) && (fabsf(filteredAccelMag - 1.0f) < ACCEL_REJECT_THRESH);

  float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  float currentKp = (gyroMag < 1.5f && accelValid) ? KpStable : KpDynamic;

  if (accelValid) {
    recipNorm = 1.0f / accelMag;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    float halfex = (ay * halfvz - az * halfvy);
    float halfey = (az * halfvx - ax * halfvz);
    float halfez = (ax * halfvy - ay * halfvx);

    if (Ki > 0.0f) {
      integralFBx += Ki * halfex * dt;
      integralFBy += Ki * halfey * dt;
      integralFBz += Ki * halfez * dt;
      integralFBx = constrain(integralFBx, -0.5f, 0.5f);
      integralFBy = constrain(integralFBy, -0.5f, 0.5f);
      integralFBz = constrain(integralFBz, -0.5f, 0.5f);
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }

    gx += currentKp * halfex;
    gy += currentKp * halfey;
    gz += currentKp * halfez;
  } else {
    integralFBx *= 0.999f;
    integralFBy *= 0.999f;
    integralFBz *= 0.999f;
  }

  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;

  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  float qNorm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (qNorm < 1e-6f) { q0 = 1.0f; q1 = q2 = q3 = 0.0f; return; }
  recipNorm = 1.0f / sqrtf(qNorm);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// ═══════════════════════════════════════════════════════════
//  IMU Calibration — EEPROM
// ═══════════════════════════════════════════════════════════

bool loadIMUCalibration() {
  uint8_t magic;
  EEPROM.get(EEPROM_ADDR, magic);
  if (magic != EEPROM_MAGIC) return false;
  EEPROM.get(EEPROM_ADDR + 1, calib);
  return true;
}

void saveIMUCalibration() {
  EEPROM.put(EEPROM_ADDR, (uint8_t)EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR + 1, calib);
}

void runIMUCalibration() {
  Serial.println(F("CAL:IMU:START"));
  Serial.println(F("Keep sensor STILL and FLAT..."));
  delay(2000);

  IMU.calibrateAccelGyro(&calib);
  saveIMUCalibration();

  Serial.println(F("CAL:IMU:DONE"));
  Serial.print(F("Gyro bias: "));
  Serial.print(calib.gyroBias[0]); Serial.print(F(", "));
  Serial.print(calib.gyroBias[1]); Serial.print(F(", "));
  Serial.println(calib.gyroBias[2]);

  IMU.init(calib, IMU_ADDRESS);
  IMU.setAccelRange(2);
  IMU.setGyroRange(500);

  // Reset filter
  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
  filteredAccelMag = 1.0f;
}

// ═══════════════════════════════════════════════════════════
//  Joystick Calibration — EEPROM
// ═══════════════════════════════════════════════════════════

bool loadJoyCalibration() {
  uint8_t magic;
  EEPROM.get(JOY_EEPROM, magic);
  if (magic != JOY_MAGIC) return false;
  EEPROM.get(JOY_EEPROM + 1, joyCenterX);
  EEPROM.get(JOY_EEPROM + 3, joyCenterY);
  return true;
}

void saveJoyCalibration() {
  EEPROM.put(JOY_EEPROM, (uint8_t)JOY_MAGIC);
  EEPROM.put(JOY_EEPROM + 1, joyCenterX);
  EEPROM.put(JOY_EEPROM + 3, joyCenterY);
}

void runJoyCalibration() {
  Serial.println(F("CAL:JOY:START"));
  Serial.println(F("Release joystick..."));
  delay(500);

  // Average 32 samples for stable center
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 32; i++) {
    sumX += analogRead(JOY_X_PIN);
    sumY += analogRead(JOY_Y_PIN);
    delay(5);
  }
  joyCenterX = (int16_t)(sumX / 32);
  joyCenterY = (int16_t)(sumY / 32);

  saveJoyCalibration();

  Serial.println(F("CAL:JOY:DONE"));
  Serial.print(F("Center: ")); Serial.print(joyCenterX);
  Serial.print(F(", ")); Serial.println(joyCenterY);
}

// ═══════════════════════════════════════════════════════════
//  Read Joystick — normalize with deadzone
// ═══════════════════════════════════════════════════════════

void readJoystick() {
  int rawX = analogRead(JOY_X_PIN);
  int rawY = analogRead(JOY_Y_PIN);

  // Normalize to -1..1 around calibrated center
  float normX = (float)(rawX - joyCenterX) / 512.0f;
  float normY = (float)(rawY - joyCenterY) / 512.0f;
  normX = constrain(normX, -1.0f, 1.0f);
  normY = constrain(normY, -1.0f, 1.0f);

  // Apply deadzone + expo for yaw
  joyYaw = applyExpo(applyDeadzone(normX, JOY_DEADZONE), EXPO);

  // Throttle: map full range to 0..1 (forward=1, back=0, center≈0.5)
  float normThrottle = applyDeadzone(normY, JOY_DEADZONE);
  joyThrottle = constrain((-normThrottle + 1.0f) * 0.5f, 0.0f, 1.0f);
}

// ═══════════════════════════════════════════════════════════
//  Read Buttons — debounced mode toggle
// ═══════════════════════════════════════════════════════════

void readButtons() {
  bool swState = digitalRead(JOY_SW_PIN);

  // Debounced toggle on falling edge (button pressed, active LOW)
  if (swState == LOW && lastSwState == HIGH && (millis() - lastSwTime > DEBOUNCE_MS)) {
    assistMode = !assistMode;
    lastSwTime = millis();
    Serial.print(F("MODE:")); Serial.println(assistMode ? F("STABILIZED") : F("MANUAL"));
  }
  lastSwState = swState;
}

// ═══════════════════════════════════════════════════════════
//  Quaternion → Euler → Normalized Control Commands
// ═══════════════════════════════════════════════════════════

void computeControlCommands() {
  // Quaternion → Euler (only roll and pitch needed)
  float rollDeg  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                          1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG_F;
  float sinPitch = 2.0f * (q0 * q2 - q3 * q1);
  sinPitch = constrain(sinPitch, -1.0f, 1.0f);  // Clamp for asinf safety
  float pitchDeg = asinf(sinPitch) * RAD2DEG_F;

  // Normalize to -1..1 with ±MAX_TILT_DEG as full deflection
  float rawRoll  = constrain(rollDeg  / MAX_TILT_DEG, -1.0f, 1.0f);
  float rawPitch = constrain(pitchDeg / MAX_TILT_DEG, -1.0f, 1.0f);

  // Apply expo curve for RC-style precision near center
  rollCmd  = applyExpo(rawRoll,  EXPO);
  pitchCmd = applyExpo(rawPitch, EXPO);
}

// ═══════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(500000);
  while (!Serial);

  Serial.println(F("\n--- Orion Flight Controller ---"));

  // ── Pin setup ──
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  pinMode(EXT_BTN_PIN, INPUT_PULLUP);

  // ── I2C + IMU ──
  Wire.begin();
  Wire.setClock(400000);

  bool imuCalLoaded = loadIMUCalibration();
  if (imuCalLoaded) {
    Serial.println(F("IMU calibration loaded."));
  } else {
    Serial.println(F("No IMU calibration found."));
  }

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print(F("IMU init error: ")); Serial.println(err);
    while (true);
  }

  IMU.setAccelRange(2);
  IMU.setGyroRange(500);

  if (!imuCalLoaded) {
    runIMUCalibration();
  }

  // ── Initialize quaternion from accelerometer ──
  IMU.update();
  IMU.getAccel(&accelData);
  float ax = accelData.accelX, ay = accelData.accelY, az = accelData.accelZ;
  float aMag = sqrtf(ax * ax + ay * ay + az * az);
  if (aMag > 0.5f) {
    ax /= aMag; ay /= aMag; az /= aMag;
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
    Serial.print(F("Init: roll=")); Serial.print(roll * RAD2DEG_F, 1);
    Serial.print(F(" pitch=")); Serial.println(pitch * RAD2DEG_F, 1);
  }

  // ── Joystick calibration ──
  bool joyCalLoaded = loadJoyCalibration();
  if (joyCalLoaded) {
    Serial.print(F("Joy center loaded: "));
    Serial.print(joyCenterX); Serial.print(F(", ")); Serial.println(joyCenterY);
  } else {
    runJoyCalibration();
  }

  Serial.println(F("Ready. Commands: 'c'=IMU cal, 'j'=Joy cal"));
  Serial.print(F("Filter: ")); Serial.print(LOOP_RATE_HZ); Serial.println(F("Hz"));
  Serial.print(F("Output: ")); Serial.print(LOOP_RATE_HZ / SERIAL_DIV); Serial.println(F("Hz"));

  lastMicros = micros();
}

// ═══════════════════════════════════════════════════════════
//  Main Loop — 500Hz filter, 100Hz dual serial output
// ═══════════════════════════════════════════════════════════

void loop() {
  // ── Timing ──
  unsigned long now = micros();
  if (now - lastMicros < LOOP_INTERVAL) return;
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt > 0.01f) dt = 0.01f;

  // ── Serial commands ──
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') { runIMUCalibration(); return; }
    if (c == 'j' || c == 'J') { runJoyCalibration(); return; }
  }

  // ── Read inputs ──
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  float ax = accelData.accelX, ay = accelData.accelY, az = accelData.accelZ;
  float gx = gyroData.gyroX * (PI / 180.0f);
  float gy = gyroData.gyroY * (PI / 180.0f);
  float gz = gyroData.gyroZ * (PI / 180.0f);

  // ── Run Mahony filter (every cycle = 500Hz) ──
  mahonyUpdate(gx, gy, gz, ax, ay, az, dt);

  // ── Read joystick + buttons (every cycle for responsiveness) ──
  readJoystick();
  readButtons();

  // ── Serial output at reduced rate (100Hz) ──
  serialCounter++;
  if (serialCounter >= SERIAL_DIV) {
    serialCounter = 0;

    // Compute flight commands from filtered quaternion
    computeControlCommands();

    bool extBtn = !digitalRead(EXT_BTN_PIN);  // Active LOW

    // ── Quaternion + raw data (for web visualizer) ──
    Serial.print(F("Q:"));
    Serial.print(q0, 4); Serial.print(',');
    Serial.print(q1, 4); Serial.print(',');
    Serial.print(q2, 4); Serial.print(',');
    Serial.print(q3, 4); Serial.print(',');
    Serial.print(accelData.accelX, 3); Serial.print(',');
    Serial.print(accelData.accelY, 3); Serial.print(',');
    Serial.print(accelData.accelZ, 3); Serial.print(',');
    Serial.print(gyroData.gyroX, 2); Serial.print(',');
    Serial.print(gyroData.gyroY, 2); Serial.print(',');
    Serial.println(gyroData.gyroZ, 2);

    // ── Control packet (for Unity) ──
    // Format: C,roll,pitch,yaw,throttle,mode,btn
    Serial.print(F("C,"));
    Serial.print(rollCmd, 3);  Serial.print(',');
    Serial.print(pitchCmd, 3); Serial.print(',');
    Serial.print(joyYaw, 3);   Serial.print(',');
    Serial.print(joyThrottle, 3); Serial.print(',');
    Serial.print(assistMode ? 1 : 0); Serial.print(',');
    Serial.println(extBtn ? 1 : 0);
  }
}