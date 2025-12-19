# Xsens Sirius IMU — Working Notes (Cepheus)

> **Status:** Living notes canvas. Consolidated understanding only. No thesis framing yet.

---

## 1. Device Identity

- **Product family:** Xsens Sirius
- **Part number:** **S1A43A-DK**
- **Product name:** Xsens Sirius AHRS/VRU/IMU Rugged – Development Kit – RS232
- **Interfaces:** RS-232, CAN(-FD) (USB via FTDI converter)
- **Protection rating:** IP68
- **Sensor ranges:**
  - Accelerometer: ±8 g
  - Gyroscope: ±300 °/s

**Interpretation:**
- Single physical inertial sensor
- Multiple runtime operating modes selectable via software (IMU / VRU / AHRS)
- Development Kit variant allows unrestricted configuration and firmware access

---

## Setup_101.md — Hardware Summary

### Hardware Overview
The system uses an **Xsens Sirius S1A43A-DK** rugged inertial measurement unit (IMU), part of the Xsens Sirius product family. The Sirius is an industrial-grade, body-mounted inertial sensor designed for harsh environments and high-dynamic applications. It integrates a tri-axial gyroscope, accelerometer, and magnetometer in a sealed IP68-rated enclosure, providing calibrated inertial measurements suitable for robotics and aerospace research platforms.

The S1A43A-DK development kit variant enables operation in multiple firmware modes (IMU, VRU, and AHRS). In this work, the device is treated as a configurable IMU, with operating mode selection fixed prior to experiments and not altered during runtime.

### Sensor Characteristics
- **Gyroscope:** ±300 °/s full-scale range, low noise density and high in-run bias stability, suitable for high-rate yaw-rate estimation in planar motion.
- **Accelerometer:** ±8 g full-scale range, temperature compensated, primarily used for diagnostic and validation purposes in planar operation.
- **Magnetometer:** Integrated but not relied upon for heading estimation due to magnetic disturbances typical of laboratory and robotic environments.

All sensor outputs are factory calibrated for bias, scale factor, misalignment, and temperature effects. Internal strapdown integration and coning/sculling compensation are performed onboard, ensuring time-aligned and physically consistent inertial measurements.

### Interfaces and Power
The Sirius IMU communicates via **RS-232 serial interface**, exposed through an FTDI-based USB–serial converter when connected to a host PC. This configuration is used for both development and deployment. Additional interfaces (RS-422 and CAN/CAN-FD) are supported by the hardware but are not used in the present setup.

The device operates from an external supply voltage in the range **4.5–24 V**. Power and communication signals are routed through a single multi-pin connector, as defined in the official Xsens connector and cable documentation.

### Mechanical Integration
The sensor enclosure includes dedicated threaded mounting points and clearly marked body axes. The measurement origin is defined by the accelerometer location inside the device. During integration, the IMU is rigidly mounted to the robot base, and its orientation and position relative to the robot reference frame are explicitly modeled in the system’s static transforms.

### Scope of Use
Within the Cepheus planar space emulator, the Sirius IMU is employed as a high-rate inertial sensor providing angular velocity measurements, primarily about the vertical (yaw) axis. Absolute position and orientation corrections are provided by an external motion capture system, allowing the IMU to be used without reliance on magnetically referenced heading estimation.

---

## 2. Kit Contents

- Sirius rugged sensor module
- RS-232 → USB cable (**CA-MP-USB-422**)
- 12-pin open-ended host interface cable (**CA-MP-12-OPEN**)

Implication:
- Easy bench bring-up via USB
- Direct embedded integration via RS-232 / CAN without extra adapters

---

## 3. Internal Architecture (Conceptual)

- 3-axis gyroscope
- 3-axis accelerometer
- 3-axis magnetometer
- High-accuracy crystal oscillator
- Onboard MCU performing:
  - Factory calibration (bias, gain, alignment, temperature)
  - High-rate strapdown integration (up to 10 kHz internal)
  - Coning & sculling compensation
  - Xsens sensor-fusion engine (orientation up to 400 Hz)

Note:
- Even in IMU mode, outputs are calibrated and temperature compensated

---

## 4. Operating Modes

### IMU Mode
- Outputs calibrated:
  - Angular rate
  - Linear acceleration
  - Magnetic field
  - (optionally) Δq, Δv increments
- No orientation estimate
- Maximum transparency, minimum assumptions

### VRU Mode
- Gravity-referenced roll & pitch (drift-free)
- Yaw unreferenced (gyro integration + internal fusion)
- Can output free acceleration (gravity removed)

### AHRS Mode
- Full roll / pitch / yaw
- Yaw magnetically referenced
- Sensitive to magnetic disturbances

Key point:
- Same hardware, different onboard processing

---

## 5. Performance Snapshot (Relevant)

### Gyroscope
- Range: ±300 °/s
- Noise density: 0.003 °/s/√Hz
- In-run bias stability: 7 °/h
- Bandwidth: 400 Hz

### Accelerometer
- Range: ±8 g
- Noise density: 15 µg/√Hz
- In-run bias stability: 15 µg
- Bandwidth: 470 Hz

Sufficient margin for planar yaw-rate estimation and diagnostics

---

## 6. Coordinate Frames & Mounting

- Right-handed sensor coordinate system
- Axes printed on device housing
- Measurement origin: accelerometer location
- Mounting:
  - 3 × M3 threaded holes
  - Flat, rigid surface recommended
  - Orientation must be recorded and reflected in TF

---

## 7. Interfaces & Electrical (Integration Notes)

### Power
- VIN: 4.5–24 V
- Typical consumption: ~400 mW

### Communication
- RS-232 (used in current setup)
- RS-422 (not used)
- CAN / CAN-FD (available)
- USB via integrated converter cable

### RS-232 Host Interface (key signals)
- TxD, RxD, RTS, CTS
- SYNC_IN / SYNC_OUT available
- Dual ground pins

---

## 8. Environmental Considerations

### Vibration
- Internal coning/sculling compensation present
- Still sensitive to:
  - Accelerometer saturation
  - High-frequency vibration near bandwidth limit
- Xsens recommends vibration dampers (e.g. Norelem 26102-00800855)

### Magnetic Environment
- Magnetometer easily distorted by:
  - Ferromagnetic materials
  - Motors
  - High currents
- Magnetically referenced yaw unreliable in lab/robot environments

---

## 9. Software Ecosystem

- **MT Software Suite**:
  - MT Manager
  - Firmware Updater
  - Magnetic Field Mapper
  - MT SDK (API + examples)

- **ROS drivers**:
  - ROS1: xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client
  - ROS2: same repository (ros2 branch)

---

## 10. Current Working Conclusions

- Device should be treated primarily as a **high-grade IMU**
- AHRS mode is physically unjustified due to magnetic disturbances
- VRU mode is usable for diagnostics but not core estimation
- IMU mode is the clean default for planar (x, y, θ) operation
- Mode selection should be fixed, not changed during experiments

---

## 11. Open Decisions (To Be Locked Later)

- Final operating mode (IMU vs VRU) → **leaning IMU**
- Output data rate (bounded by RS-232 stability)
- Exact ROS topic contract
- Bias handling strategy

