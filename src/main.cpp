#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>
#include <EEPROM.h>

// ─── Configuration ──────────────────────────────────────────
#define IMU_ADDRESS    0x68
#define EEPROM_MAGIC   0xA5          // Magic byte to detect valid calibration
#define EEPROM_ADDR    0             // EEPROM start address for calibration
#define LOOP_RATE_HZ   500           // Filter update rate
#define SERIAL_DIV     5             // Send serial every Nth cycle (500/5 = 100Hz)
#define LOOP_INTERVAL  (1000000UL / LOOP_RATE_HZ)  // Microseconds per cycle

// ─── Mahony Filter Gains ────────────────────────────────────
// Kp: Proportional — how aggressively accel corrects gyro drift
// Ki: Integral — slowly learns and removes persistent gyro bias
static const float KpStable  = 2.0f;    // Kp when stationary (fast convergence)
static const float KpDynamic = 0.2f;    // Kp during motion (smooth, trust gyro)
static const float Ki = 0.005f;
static const float ACCEL_REJECT_THRESH = 0.15f;  // |a| must be within 1g ± this

// ─── Sensor Objects ─────────────────────────────────────────
MPU6500 IMU;
calData calib = { 0 };
AccelData accelData;
GyroData gyroData;

// ─── Mahony Filter State ────────────────────────────────────
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // Quaternion
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // Integral error
float filteredAccelMag = 1.0f;  // Low-pass filtered accel magnitude

// ─── Timing ─────────────────────────────────────────────────
unsigned long lastMicros = 0;
uint8_t serialCounter = 0;

// ═══════════════════════════════════════════════════════════
//  Mahony AHRS Update (6-DOF, no magnetometer)
// ═══════════════════════════════════════════════════════════

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  float recipNorm;

  // ── Low-pass filtered accel magnitude for smooth mode switching ──
  float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  filteredAccelMag = 0.95f * filteredAccelMag + 0.05f * accelMag;

  // Only trust accel when filtered |a| ≈ 1g (no linear acceleration)
  bool accelValid = (filteredAccelMag > 0.01f) && (fabsf(filteredAccelMag - 1.0f) < ACCEL_REJECT_THRESH);

  // Adaptive gain: high when stable, low during aggressive motion
  float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  float currentKp = (gyroMag < 1.5f && accelValid) ? KpStable : KpDynamic;

  if (accelValid) {
    // Normalize accelerometer
    recipNorm = 1.0f / accelMag;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // ── Estimated gravity direction from quaternion ──
    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    // ── Error: cross product of measured vs estimated gravity ──
    float halfex = (ay * halfvz - az * halfvy);
    float halfey = (az * halfvx - ax * halfvz);
    float halfez = (ax * halfvy - ay * halfvx);

    // ── Integral feedback (learns gyro bias over time) ──
    if (Ki > 0.0f) {
      integralFBx += Ki * halfex * dt;
      integralFBy += Ki * halfey * dt;
      integralFBz += Ki * halfez * dt;
      // Anti-windup: clamp integral to prevent unrealistic bias
      integralFBx = constrain(integralFBx, -0.5f, 0.5f);
      integralFBy = constrain(integralFBy, -0.5f, 0.5f);
      integralFBz = constrain(integralFBz, -0.5f, 0.5f);
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }

    // ── Proportional feedback (adaptive) ──
    gx += currentKp * halfex;
    gy += currentKp * halfey;
    gz += currentKp * halfez;
  } else {
    // Accel invalid: slowly leak stale integral bias to prevent overshoot
    integralFBx *= 0.999f;
    integralFBy *= 0.999f;
    integralFBz *= 0.999f;
  }

  // ── Integrate quaternion rate of change ──
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  // ── Normalize quaternion (with safety guard) ──
  float qNorm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (qNorm < 1e-6f) { q0 = 1.0f; q1 = q2 = q3 = 0.0f; return; }
  recipNorm = 1.0f / sqrtf(qNorm);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// ═══════════════════════════════════════════════════════════
//  Calibration — EEPROM Read/Write
// ═══════════════════════════════════════════════════════════

bool loadCalibration() {
  uint8_t magic;
  EEPROM.get(EEPROM_ADDR, magic);
  if (magic != EEPROM_MAGIC) return false;

  EEPROM.get(EEPROM_ADDR + 1, calib);
  return true;
}

void saveCalibration() {
  EEPROM.put(EEPROM_ADDR, (uint8_t)EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR + 1, calib);
}

void runCalibration() {
  Serial.println(F("CAL:START"));
  Serial.println(F("Keep sensor STILL and FLAT for calibration..."));
  delay(2000);  // Give user time to set it down

  IMU.calibrateAccelGyro(&calib);
  saveCalibration();

  Serial.println(F("CAL:DONE"));
  Serial.print(F("Gyro bias: "));
  Serial.print(calib.gyroBias[0]); Serial.print(F(", "));
  Serial.print(calib.gyroBias[1]); Serial.print(F(", "));
  Serial.println(calib.gyroBias[2]);
  Serial.print(F("Accel bias: "));
  Serial.print(calib.accelBias[0]); Serial.print(F(", "));
  Serial.print(calib.accelBias[1]); Serial.print(F(", "));
  Serial.println(calib.accelBias[2]);

  // Re-initialize with new calibration
  IMU.init(calib, IMU_ADDRESS);
  IMU.setAccelRange(2);
  IMU.setGyroRange(500);

  // Reset filter state
  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
  filteredAccelMag = 1.0f;
}

// ═══════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(500000);
  while (!Serial);

  Serial.println(F("\n--- Orion IMU | Mahony AHRS ---"));
  Wire.begin();
  Wire.setClock(400000);

  // Load calibration from EEPROM (or use zeros)
  bool calibLoaded = loadCalibration();
  if (calibLoaded) {
    Serial.println(F("Loaded calibration from EEPROM."));
  } else {
    Serial.println(F("No calibration found."));
  }

  // Initialize IMU
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print(F("IMU init error: "));
    Serial.println(err);
    while (true);
  }

  // Set ranges
  IMU.setAccelRange(2);    // ±2g
  IMU.setGyroRange(500);   // ±500 dps (better for drone maneuvers)

  // Auto-calibrate if no saved data
  if (!calibLoaded) {
    runCalibration();
  }

  // ── Initialize quaternion from current accelerometer reading ──
  // This prevents the model from snapping if the board starts tilted
  IMU.update();
  IMU.getAccel(&accelData);
  float ax = accelData.accelX;
  float ay = accelData.accelY;
  float az = accelData.accelZ;
  float aMag = sqrtf(ax * ax + ay * ay + az * az);
  if (aMag > 0.5f) {  // Only if we have a reasonable reading
    ax /= aMag; ay /= aMag; az /= aMag;
    // Compute roll and pitch from gravity direction
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    // Convert roll/pitch to quaternion (yaw = 0)
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;
    Serial.print(F("Init attitude: roll=")); Serial.print(roll * 57.3f, 1);
    Serial.print(F("° pitch=")); Serial.print(pitch * 57.3f, 1);
    Serial.println(F("°"));
  }

  Serial.println(F("Ready. Send 'c' to recalibrate."));
  Serial.print(F("Filter: ")); Serial.print(LOOP_RATE_HZ); Serial.println(F("Hz"));
  Serial.print(F("Output: ")); Serial.print(LOOP_RATE_HZ / SERIAL_DIV); Serial.println(F("Hz"));

  lastMicros = micros();
}

// ═══════════════════════════════════════════════════════════
//  Main Loop — 500Hz filter, 100Hz serial output
// ═══════════════════════════════════════════════════════════

void loop() {
  // ── Tight timing loop ──
  unsigned long now = micros();
  if (now - lastMicros < LOOP_INTERVAL) return;
  float dt = (now - lastMicros) * 1e-6f;  // Convert to seconds
  lastMicros = now;

  // Clamp dt to avoid spikes
  if (dt > 0.01f) dt = 0.01f;

  // ── Check for recalibration command ──
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      runCalibration();
      return;
    }
  }

  // ── Read sensor data ──
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  float ax = accelData.accelX;
  float ay = accelData.accelY;
  float az = accelData.accelZ;

  // Convert gyro from dps to rad/s for Mahony
  float gx = gyroData.gyroX * (PI / 180.0f);
  float gy = gyroData.gyroY * (PI / 180.0f);
  float gz = gyroData.gyroZ * (PI / 180.0f);

  // ── Run Mahony filter ──
  mahonyUpdate(gx, gy, gz, ax, ay, az, dt);

  // ── Serial output at reduced rate ──
  serialCounter++;
  if (serialCounter >= SERIAL_DIV) {
    serialCounter = 0;

    // Format: Q:qw,qx,qy,qz,ax,ay,az,gx,gy,gz
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
  }
}