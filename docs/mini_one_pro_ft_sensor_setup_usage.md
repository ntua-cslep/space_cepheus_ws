# MiniONE Pro Force–Torque Sensor

This document describes the **setup, configuration, calibration, and runtime usage** of the **Bota Systems MiniONE Pro (Gen‑0, Serial)** force–torque sensor as integrated on the Cepheus robot.

The goal is to make the FT stack **reproducible, debuggable, and operator‑friendly**, without hidden firmware state.

---

## 1. File Structure

### Configuration
```
cepheus_bringup/config/mini_one.yaml
```

Contains:
- Serial device configuration
- Force–torque filter settings (SINC + FIR)
- Sensor flags (IMU disabled, calibration matrix enabled)
- Software offsets

### Launch File
```
cepheus_bringup/launch/ft_sensor.launch
```

Responsible for:
- Launching the Bota serial bus manager
- Loading the YAML configuration into the ROS parameter server
- Advertising FT topics and services

### Baseline Capture Script
```
cepheus_bringup/scripts/ft_baseline_capture.py
```

Utility script used to:
- Record raw FT values in a known static pose
- Compute and print a baseline wrench
- Assist in EEPROM or documentation‑level calibration

---

## 2. Launching the FT Sensor

Start the force–torque sensor stack:

```bash
roslaunch cepheus_bringup ft_sensor.launch
```

Verify topics:
```
/bus0/ft_sensor0/ft_sensor_readings/wrench
```

Verify services:
```
/bus0/ft_sensor0/reset_wrench
```

---

## 3. Baseline Measurement Procedure

### Command

```bash
rosrun cepheus_bringup ft_baseline_capture.py
```

### Measurement Conditions

- Robot mounted on table
- No external contact
- Joint pose: **(0°, 90°, 0°)**
- Date: **11/12/2025**

### Recorded Baseline

```
b,-9.928801,5.918258,2.375053,0.008588,-0.183896,0.019959
```

This baseline is **documented** and may be:
- Stored in EEPROM (optional)
- Used as reference for software offsets
- Used for validation and repeatability checks

---

## 4. Runtime Zeroing (Preferred Method)

Rather than relying on persistent firmware biasing, the system uses **runtime zeroing** via ROS services.

### Manual Zero Command (Shell)

```bash
rosservice call /bus0/ft_sensor0/reset_wrench \
"{desired_wrench: {force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
```

This sets the **current measured wrench** as the zero reference.

### Recommended Procedure

1. Move robot to documented neutral pose
2. Ensure no contact
3. Call `reset_wrench`
4. Start experiment or controller

This avoids hidden offsets and keeps behavior reproducible.

---

## 5. Subscribing to FT Data (C++)

Example callback extracting the relevant planar components:

```cpp
void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
    fts_force_z  = msg->wrench.force.z;
    fts_force_y  = msg->wrench.force.y;
    fts_torque_x = msg->wrench.torque.x;
}
```

Used signals:
- **Fy**: in‑plane lateral force
- **Fz**: in‑plane longitudinal force
- **Tx**: out‑of‑plane torque

---

## 6. Programmatic Zeroing (C++)

### Build Dependencies

#### `CMakeLists.txt`
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rokubimini_msgs
)
```

#### `package.xml`
```xml
<depend>rokubimini_msgs</depend>
```

---

### Includes

```cpp
#include <rokubimini_msgs/ResetWrench.h>
#include <ros/service_client.h>
```

---

### Service Client Initialization

```cpp
ros::ServiceClient ft_reset_client_;
std::string ft_reset_service_ = "/bus0/ft_sensor0/reset_wrench";

ft_reset_client_ = nh.serviceClient<rokubimini_msgs::ResetWrench>(ft_reset_service_);
```

---

### Runtime Zeroing Function

```cpp
bool resetFtWrenchToZero(ros::ServiceClient& client)
{
  rokubimini_msgs::ResetWrench srv;
  srv.request.desired_wrench.force.x  = 0.0;
  srv.request.desired_wrench.force.y  = 0.0;
  srv.request.desired_wrench.force.z  = 0.0;
  srv.request.desired_wrench.torque.x = 0.0;
  srv.request.desired_wrench.torque.y = 0.0;
  srv.request.desired_wrench.torque.z = 0.0;

  if (!client.exists()) {
    ROS_WARN("FT reset_wrench service not available.");
    return false;
  }

  if (!client.call(srv)) {
    ROS_ERROR("FT reset_wrench service call failed.");
    return false;
  }

  if (!srv.response.success) {
    ROS_WARN("FT reset_wrench returned success=false.");
    return false;
  }

  ROS_INFO("FT wrench reset OK (desired_wrench = 0).");
  return true;
}
```

### Usage

```cpp
resetFtWrenchToZero(ft_reset_client_);
```

This allows operator‑triggered or state‑machine‑triggered zeroing.

---

## 7. Filter Configuration Notes

- **SINC = 57**
- **FIR enabled**
- Effective output ≈ 100 Hz
- Cutoff ≈ 50 Hz

Chosen to balance:
- Low latency
- Sufficient smoothing
- Compatibility with 100 Hz control loops

---

## 8. Design Philosophy

- **YAML is the source of truth** for configuration
- **No hidden firmware state** required for normal operation
- **Runtime zeroing > persistent biasing**
- All steps documented and reproducible

---

## 9. Known Good State

As of this commit:
- FT sensor streaming correctly
- Axes verified
- Live zeroing validated in PlotJuggler
- C++ integration tested
- Ready for impedance control integration

