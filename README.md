# Orion AHRS Module

A high-performance Attitude and Heading Reference System (AHRS) implemented on embedded hardware, coupled with a WebGL-based real-time 3D visualizer.

## Overview

The Orion AHRS module provides stable, drift-compensated 3D orientation tracking. It offloads the complex quaternion mathematics directly to the micro-controller, running a highly optimized Mahony filter at 500Hz. The resulting data is streamed via a high-speed serial connection to a zero-installation, browser-based 3D visualizer.

## Architecture

### Firmware (MCU)
The firmware is built using PlatformIO and the Arduino framework. It interfaces with the IMU sensor to acquire high-rate accelerometer and gyroscope data.

**Key Features:**
- **High-Speed Mahony Filter:** Runs at 500Hz to ensure minimal latency and high accuracy.
- **Adaptive Proportional Gain (Kp):** Dynamically adjusts filter responsiveness. High gain during stable periods for rapid convergence, and low gain during aggressive motion to trust the gyroscope and ignore linear acceleration noise.
- **Motion Rejection:** Monitors the magnitude of the accelerometer vector. If the sensor experiences linear acceleration (deviating significantly from 1g), the filter temporarily suspends accelerometer corrections to prevent false tilt.
- **Integral Anti-Windup:** Clamps the integral feedback terms to prevent learned gyroscope bias from growing unrealistically during extended dynamic maneuvers.
- **Persistent Calibration:** Automated or manual calibration routines save gyroscope and accelerometer biases directly to EEPROM, persisting across power cycles.
- **Smart Initialization:** On boot, the initial quaternion is calculated directly from the gravity vector, preventing model "snapping" if the device is powered on at an angle.

### Visualizer (Web)
A lightweight web application that consumes the serial data stream.

**Key Features:**
- **Web Serial API:** Connects directly to the micro-controller via the browser without requiring intermediate server software or Python scripts.
- **Quaternion Rendering:** Utilizes Three.js quaternion spherical linear interpolation (Slerp) to smoothly animate a 3D model without suffering from Gimbal Lock.
- **Frame Conversion:** Handles similarity transformations to correctly map the sensor's native coordinate frame to the 3D rendering space.

## Installation and Setup

### Prerequisites
- [PlatformIO Core](https://platformio.org/) or the PlatformIO IDE extension for VS Code.
- A compatible micro-controller (currently optimized for Arduino Nano / ATmega328P, but easily adaptable to ESP32 or STM32).
- An MPU6500 IMU (or any IMU supported by the FastIMU library).

### Building the Firmware
1. Clone the repository.
2. Open the project directory in PlatformIO.
3. Build and upload the firmware.

```bash
platformio run --target upload
```

### Running the Visualizer
The visualizer runs entirely client-side. No build step or local web server is strictly necessary for basic operation.

1. Navigate to the `visualizer` directory.
2. Install dependencies (e.g., using `npm install` if required by the environment, or run a local static server).
3. If using Vite or similar, start the development server: `npm run dev`.
4. Open the application in a modern web browser that supports the Web Serial API (e.g., Google Chrome, Microsoft Edge).
5. Click "Connect Serial" and select the appropriate COM port. The default baud rate is `500000`.

## Serial Protocol Specification

The firmware outputs ASCII text over the serial port. Data packets are prefixed with `Q:` denoting a quaternion and raw sensor data payload.

| Field | Description | Type |
|-------|-------------|------|
| `qw` | Quaternion W (Scalar) | Float (4 decimal places) |
| `qx` | Quaternion X (Vector) | Float (4 decimal places) |
| `qy` | Quaternion Y (Vector) | Float (4 decimal places) |
| `qz` | Quaternion Z (Vector) | Float (4 decimal places) |
| `ax` | Accelerometer X (g) | Float (3 decimal places) |
| `ay` | Accelerometer Y (g) | Float (3 decimal places) |
| `az` | Accelerometer Z (g) | Float (3 decimal places) |
| `gx` | Gyroscope X (rad/s) | Float (2 decimal places) |
| `gy` | Gyroscope Y (rad/s) | Float (2 decimal places) |
| `gz` | Gyroscope Z (rad/s) | Float (2 decimal places) |

**Example Packet:**
```text
Q:0.9998,0.0123,-0.0054,0.0010,0.015,-0.008,1.002,0.01,-0.02,0.00
```

## Calibration Procedure

> [!IMPORTANT]
> The AHRS module requires an initial calibration to function optimally. Ensure the module is placed on a completely flat, stable surface before initiating.

1. Power on the device while it is resting perfectly still.
2. If no calibration data is found in EEPROM, the device will automatically initiate a 2-second countdown followed by reading sensor biases.
3. To manually trigger a recalibration at any time, send the character `c` or `C` over the serial connection.
