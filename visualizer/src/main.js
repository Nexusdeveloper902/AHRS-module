// ============================================
//  Orion IMU — 3D Orientation Visualizer
//  Web Serial + Three.js (MCU-side Mahony AHRS)
// ============================================

import * as THREE from 'three';

// ──────────────────────────────────────────────
//  State
// ──────────────────────────────────────────────

const state = {
  // Serial
  port: null,
  reader: null,
  reading: false,

  // Raw sensor data (for display)
  accel: { x: 0, y: 0, z: 0 },
  gyro:  { x: 0, y: 0, z: 0 },

  // Quaternion from MCU (pre-filtered)
  q: new THREE.Quaternion(0, 0, 0, 1),
  qSmooth: new THREE.Quaternion(0, 0, 0, 1),

  // Euler angles for display (degrees)
  pitch: 0,
  roll:  0,
  yaw:   0,

  // Timing
  dataCount: 0,
  lastRateCheck: 0,
  dataRate: 0,
};

const SLERP_SPEED = 0.25;   // Rendering smoothing (higher = more responsive)
const RAD2DEG = 180 / Math.PI;

// ──────────────────────────────────────────────
//  DOM Elements
// ──────────────────────────────────────────────

const dom = {
  canvas:     document.getElementById('canvas-container'),
  connectBtn: document.getElementById('connect-btn'),
  statusDot:  document.getElementById('status-dot'),
  statusText: document.getElementById('status-text'),
  portInfo:   document.getElementById('port-info'),
  portName:   document.getElementById('port-name'),
  dataRate:   document.getElementById('data-rate'),

  // Orientation gauges
  barPitch:  document.getElementById('bar-pitch'),
  barRoll:   document.getElementById('bar-roll'),
  barYaw:    document.getElementById('bar-yaw'),
  valPitch:  document.getElementById('val-pitch'),
  valRoll:   document.getElementById('val-roll'),
  valYaw:    document.getElementById('val-yaw'),

  // Raw data
  accelX: document.getElementById('accel-x'),
  accelY: document.getElementById('accel-y'),
  accelZ: document.getElementById('accel-z'),
  gyroX:  document.getElementById('gyro-x'),
  gyroY:  document.getElementById('gyro-y'),
  gyroZ:  document.getElementById('gyro-z'),
};

// ──────────────────────────────────────────────
//  Three.js Setup
// ──────────────────────────────────────────────

const scene    = new THREE.Scene();
const camera   = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });

renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.2;
dom.canvas.appendChild(renderer.domElement);

camera.position.set(5, 3.5, 6);
camera.lookAt(0, 0, 0);

// Background
scene.background = new THREE.Color(0x06060f);
scene.fog = new THREE.FogExp2(0x06060f, 0.04);

// ── Lighting ──
const ambientLight = new THREE.AmbientLight(0x404080, 0.6);
scene.add(ambientLight);

const mainLight = new THREE.DirectionalLight(0xccddff, 1.8);
mainLight.position.set(5, 8, 5);
mainLight.castShadow = false;
scene.add(mainLight);

const fillLight = new THREE.DirectionalLight(0x6c8cff, 0.4);
fillLight.position.set(-4, 2, -3);
scene.add(fillLight);

const rimLight = new THREE.PointLight(0x4af0c0, 0.5, 30);
rimLight.position.set(-3, 5, -5);
scene.add(rimLight);

// ── Grid ──
const gridHelper = new THREE.GridHelper(20, 40, 0x1a1a3e, 0x0f0f28);
gridHelper.position.y = -2;
gridHelper.material.transparent = true;
gridHelper.material.opacity = 0.5;
scene.add(gridHelper);

// ── Axis Helper (subtle) ──
const axesLength = 3;

function createAxisLine(color, start, end) {
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.35 });
  const geo = new THREE.BufferGeometry().setFromPoints([start, end]);
  return new THREE.Line(geo, mat);
}

scene.add(createAxisLine(0xff6c6c, new THREE.Vector3(0, 0, 0), new THREE.Vector3(axesLength, 0, 0)));  // X
scene.add(createAxisLine(0x4af0c0, new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, axesLength, 0)));  // Y
scene.add(createAxisLine(0x6c8cff, new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, axesLength)));  // Z

// ── Star field ──
function createStarField() {
  const count = 800;
  const positions = new Float32Array(count * 3);
  for (let i = 0; i < count * 3; i++) {
    positions[i] = (Math.random() - 0.5) * 100;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.PointsMaterial({
    color: 0x6c8cff,
    size: 0.08,
    transparent: true,
    opacity: 0.6,
    sizeAttenuation: true,
  });
  return new THREE.Points(geo, mat);
}
const stars = createStarField();
scene.add(stars);

// ──────────────────────────────────────────────
//  Airplane Model (Procedural)
// ──────────────────────────────────────────────

function createAirplane() {
  const group = new THREE.Group();

  // Materials
  const bodyMat = new THREE.MeshStandardMaterial({
    color: 0x3a4a8c,
    metalness: 0.6,
    roughness: 0.35,
    flatShading: true,
  });
  const accentMat = new THREE.MeshStandardMaterial({
    color: 0x6c8cff,
    metalness: 0.7,
    roughness: 0.25,
    flatShading: true,
    emissive: 0x1a2a6c,
    emissiveIntensity: 0.15,
  });
  const darkMat = new THREE.MeshStandardMaterial({
    color: 0x1e2444,
    metalness: 0.5,
    roughness: 0.4,
    flatShading: true,
  });
  const glowMat = new THREE.MeshStandardMaterial({
    color: 0x4af0c0,
    emissive: 0x4af0c0,
    emissiveIntensity: 0.8,
    metalness: 0.0,
    roughness: 0.3,
  });

  // Fuselage — elongated octagonal prism
  const fuselageGeo = new THREE.CylinderGeometry(0.28, 0.22, 3.2, 8);
  fuselageGeo.rotateZ(Math.PI / 2);
  const fuselage = new THREE.Mesh(fuselageGeo, bodyMat);
  group.add(fuselage);

  // Nose cone
  const noseGeo = new THREE.ConeGeometry(0.28, 0.7, 8);
  noseGeo.rotateZ(-Math.PI / 2);
  const nose = new THREE.Mesh(noseGeo, accentMat);
  nose.position.x = 1.95;
  group.add(nose);

  // Cockpit canopy
  const canopyGeo = new THREE.SphereGeometry(0.18, 8, 6, 0, Math.PI * 2, 0, Math.PI / 2);
  const canopyMat = new THREE.MeshStandardMaterial({
    color: 0x6ceeff,
    metalness: 0.9,
    roughness: 0.1,
    transparent: true,
    opacity: 0.6,
  });
  const canopy = new THREE.Mesh(canopyGeo, canopyMat);
  canopy.position.set(0.9, 0.22, 0);
  group.add(canopy);

  // Main wings
  const wingShape = new THREE.Shape();
  wingShape.moveTo(0, 0);
  wingShape.lineTo(2.2, -0.3);
  wingShape.lineTo(2.0, -0.5);
  wingShape.lineTo(-0.3, -0.15);
  wingShape.closePath();
  const wingExtrudeSettings = { depth: 0.06, bevelEnabled: false };
  const wingGeo = new THREE.ExtrudeGeometry(wingShape, wingExtrudeSettings);

  const leftWing = new THREE.Mesh(wingGeo, accentMat);
  leftWing.position.set(0.1, -0.05, 0.03);
  group.add(leftWing);

  const rightWing = new THREE.Mesh(wingGeo, accentMat);
  rightWing.position.set(0.1, -0.05, -0.09);
  rightWing.scale.z = -1;
  group.add(rightWing);

  // Horizontal stabilizers (tail wings)
  const tailWingShape = new THREE.Shape();
  tailWingShape.moveTo(0, 0);
  tailWingShape.lineTo(0.8, -0.15);
  tailWingShape.lineTo(0.7, -0.25);
  tailWingShape.lineTo(-0.15, -0.08);
  tailWingShape.closePath();
  const tailWingGeo = new THREE.ExtrudeGeometry(tailWingShape, { depth: 0.04, bevelEnabled: false });

  const leftTail = new THREE.Mesh(tailWingGeo, darkMat);
  leftTail.position.set(-1.4, 0.05, 0.02);
  group.add(leftTail);

  const rightTail = new THREE.Mesh(tailWingGeo, darkMat);
  rightTail.position.set(-1.4, 0.05, -0.06);
  rightTail.scale.z = -1;
  group.add(rightTail);

  // Vertical stabilizer (tail fin)
  const finShape = new THREE.Shape();
  finShape.moveTo(0, 0);
  finShape.lineTo(-0.3, 0.8);
  finShape.lineTo(-0.6, 0.7);
  finShape.lineTo(-0.2, 0);
  finShape.closePath();
  const finGeo = new THREE.ExtrudeGeometry(finShape, { depth: 0.04, bevelEnabled: false });
  const fin = new THREE.Mesh(finGeo, accentMat);
  fin.position.set(-1.2, 0.12, -0.02);
  group.add(fin);

  // Engine glow (under-belly strip)
  const engineGeo = new THREE.BoxGeometry(0.8, 0.04, 0.12);
  const engine = new THREE.Mesh(engineGeo, glowMat);
  engine.position.set(-0.4, -0.24, 0);
  group.add(engine);

  // Wing tip lights
  const tipGeo = new THREE.SphereGeometry(0.04, 6, 6);
  const leftTip = new THREE.Mesh(tipGeo, new THREE.MeshStandardMaterial({
    color: 0xff4444, emissive: 0xff4444, emissiveIntensity: 1.0
  }));
  leftTip.position.set(0.35, -0.38, 2.15);
  group.add(leftTip);

  const rightTip = new THREE.Mesh(tipGeo, new THREE.MeshStandardMaterial({
    color: 0x44ff44, emissive: 0x44ff44, emissiveIntensity: 1.0
  }));
  rightTip.position.set(0.35, -0.38, -2.15);
  group.add(rightTip);

  return group;
}

const airplane = createAirplane();
airplane.rotation.y = Math.PI; // Face the other direction visually
const airplaneMount = new THREE.Group(); // Receives orientation quaternion
airplaneMount.add(airplane);
scene.add(airplaneMount);

// Frame conversion: sensor body (X=right, Y=forward, Z=up) → Three.js airplane (X=forward, Y=up, Z=left)
const FRAME_Q = new THREE.Quaternion(-0.5, -0.5, -0.5, 0.5);
const FRAME_Q_INV = FRAME_Q.clone().conjugate();
const _displayQ = new THREE.Quaternion();

// ──────────────────────────────────────────────
//  Serial Communication (Web Serial API)
// ──────────────────────────────────────────────

async function forceCleanup() {
  // Kill any existing read loop
  state.reading = false;

  // Release the reader if it exists
  try {
    if (state.reader) {
      await state.reader.cancel().catch(() => {});
      state.reader.releaseLock();
      state.reader = null;
    }
  } catch (_) { /* already cleaned up */ }

  // Close the port if it exists and is open
  try {
    if (state.port && state.port.readable) {
      await state.port.close().catch(() => {});
    }
  } catch (_) { /* already closed */ }

  state.port = null;
}

async function connectSerial() {
  if (!('serial' in navigator)) {
    alert('Web Serial API not available. Use Chrome or Edge.');
    return;
  }

  // Always clean up any previous connection first
  await forceCleanup();

  try {
    state.port = await navigator.serial.requestPort();
    await state.port.open({ baudRate: 500000 });

    setConnected(true);
    _firstParsed = false;
    state.reading = true;
    state.lastTime = performance.now();
    state.lastRateCheck = performance.now();
    state.dataCount = 0;

    readSerialLoop();
  } catch (err) {
    console.error('Serial connection failed:', err);
    state.port = null;
    setConnected(false);
  }
}

async function disconnectSerial() {
  await forceCleanup();
  setConnected(false);
}

async function readSerialLoop() {
  console.log('🔌 readSerialLoop started');
  const textDecoder = new TextDecoderStream();
  const readableStreamClosed = state.port.readable.pipeTo(textDecoder.writable);
  state.reader = textDecoder.readable.getReader();

  let buffer = '';

  try {
    while (state.reading) {
      const { value, done } = await state.reader.read();
      if (done) break;

      buffer += value;
      const lines = buffer.split('\n');
      buffer = lines.pop();

      for (const line of lines) {
        parseLine(line.trim());
      }
    }
  } catch (err) {
    if (state.reading) {
      console.error('Read error:', err);
    }
  } finally {
    state.reading = false;
    try {
      if (state.reader) {
        state.reader.releaseLock();
        state.reader = null;
      }
    } catch (_) { /* ignore */ }
    try {
      await readableStreamClosed.catch(() => {});
    } catch (_) { /* ignore */ }
    try {
      if (state.port) {
        await state.port.close().catch(() => {});
        state.port = null;
      }
    } catch (_) { /* ignore */ }
    setConnected(false);
    state.dataRate = 0;
    console.log('Serial connection closed.');
  }
}

let _firstParsed = false;

function parseLine(line) {
  if (line.startsWith('Q:')) {
    // Format: Q:qw,qx,qy,qz,ax,ay,az,gx,gy,gz
    const parts = line.substring(2).split(',');
    if (parts.length !== 10) return;
    const values = parts.map(Number);
    if (values.some(isNaN)) return;

    const [qw, qx, qy, qz, ax, ay, az, gx, gy, gz] = values;

    // Set quaternion from MCU (Three.js format: x, y, z, w)
    state.q.set(qx, qy, qz, qw);

    // Store raw data for display
    state.accel.x = ax;
    state.accel.y = ay;
    state.accel.z = az;
    state.gyro.x  = gx;
    state.gyro.y  = gy;
    state.gyro.z  = gz;

    if (!_firstParsed) {
      console.log('✅ First quaternion received:', { qw, qx, qy, qz });
      _firstParsed = true;
    }

  } else if (line.startsWith('CAL:')) {
    console.log('🎯 Calibration:', line);

  } else {
    return; // Ignore other lines
  }

  // Data rate tracking
  const now = performance.now();
  state.dataCount++;
  if (now - state.lastRateCheck >= 1000) {
    state.dataRate = state.dataCount;
    state.dataCount = 0;
    state.lastRateCheck = now;
  }
}

// ──────────────────────────────────────────────
//  UI Updates
// ──────────────────────────────────────────────

function setConnected(connected) {
  dom.statusDot.className  = `status-dot ${connected ? 'connected' : 'disconnected'}`;
  dom.statusText.textContent = connected ? 'Connected' : 'Disconnected';

  const btnSpan = dom.connectBtn.querySelector('span');
  if (connected) {
    dom.connectBtn.classList.add('disconnect');
    btnSpan.textContent = 'Disconnect';
    dom.portInfo.classList.remove('hidden');
  } else {
    dom.connectBtn.classList.remove('disconnect');
    btnSpan.textContent = 'Connect Serial';
    dom.portInfo.classList.add('hidden');
  }
}

function updateGaugeBar(barEl, value, maxAngle) {
  // Bar centered at 50%, extends left/right based on angle
  const ratio = Math.max(-1, Math.min(1, value / maxAngle));
  const center = 50;
  const halfWidth = Math.abs(ratio) * 45; // Max 45% from center

  if (ratio >= 0) {
    barEl.style.left = `${center}%`;
    barEl.style.width = `${halfWidth}%`;
  } else {
    barEl.style.left = `${center - halfWidth}%`;
    barEl.style.width = `${halfWidth}%`;
  }
}

function updateUI() {
  // Orientation gauges (use display angles)
  updateGaugeBar(dom.barPitch, state.pitch, 90);
  updateGaugeBar(dom.barRoll,  state.roll, 90);
  updateGaugeBar(dom.barYaw,   state.yaw, 180);

  dom.valPitch.textContent = `${state.pitch.toFixed(1)}°`;
  dom.valRoll.textContent  = `${state.roll.toFixed(1)}°`;
  dom.valYaw.textContent   = `${state.yaw.toFixed(1)}°`;

  // Raw data
  dom.accelX.textContent = state.accel.x.toFixed(4);
  dom.accelY.textContent = state.accel.y.toFixed(4);
  dom.accelZ.textContent = state.accel.z.toFixed(4);
  dom.gyroX.textContent  = state.gyro.x.toFixed(4);
  dom.gyroY.textContent  = state.gyro.y.toFixed(4);
  dom.gyroZ.textContent  = state.gyro.z.toFixed(4);

  // Data rate
  dom.dataRate.textContent = `${state.dataRate} Hz`;
  dom.portName.textContent = state.port ? 'Serial' : '—';
}

// ──────────────────────────────────────────────
//  Render Loop
// ──────────────────────────────────────────────

function animate() {
  requestAnimationFrame(animate);

  // Smooth quaternion interpolation (slerp)
  state.qSmooth.slerp(state.q, SLERP_SPEED);

  // Apply frame conversion: similarity transform rotates sensor axes to airplane axes
  // q_display = FRAME_Q * q_sensor * FRAME_Q_INV
  _displayQ.copy(FRAME_Q).multiply(state.qSmooth).multiply(FRAME_Q_INV);
  airplaneMount.quaternion.copy(_displayQ);

  // Extract Euler angles for UI display (from the converted quaternion)
  const euler = new THREE.Euler().setFromQuaternion(_displayQ, 'YXZ');
  state.pitch = euler.x * RAD2DEG;
  state.roll  = euler.z * RAD2DEG;
  state.yaw   = euler.y * RAD2DEG;

  // Gentle star rotation
  stars.rotation.y += 0.00008;

  // Update UI
  updateUI();

  renderer.render(scene, camera);
}

// ──────────────────────────────────────────────
//  Events
// ──────────────────────────────────────────────

dom.connectBtn.addEventListener('click', () => {
  if (state.reading) {
    disconnectSerial();
  } else {
    connectSerial();
  }
});

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ──────────────────────────────────────────────
//  Start
// ──────────────────────────────────────────────

animate();
