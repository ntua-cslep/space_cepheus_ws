# 🚀 Cepheus Lab – Helper Scripts, Layouts & Tools

This repository contains all the small but essential scripts used to run the **Cepheus planar robot**, manage ROS environments, sync code/bags, and visualize experiments.

---

## 📁 Directory Overview

```
bin/                                  # Executable scripts (host + robot helpers)
robot_home/                           # Environment scripts deployed on the PC-104 robot
config/terminator/                    # Terminator layout & profiles for the lab UI
Desktop/                              # Desktop launchers for quick startup
cepheus_ws_v2/bags/converters/        # Bag → MAT converters & diagnostic plotting
Documents/                            # PlotJuggler layouts & visualization configs
```

---

# 🖥️ HOST-SIDE SCRIPTS (bin/)

### ros_env.sh
Sets up the **host ROS environment** (Noetic + ~/cepheus_ws).  
Used by the Terminator lab layout and by various startup scripts.

---

### start_cepheus_lab.sh
Launches the full **Terminator lab interface** using the config from ~/.config/terminator.  
Checks that ros_env.sh exists before launching.

---

### with_roscore.sh
Helper used inside the Terminator layout.  
Blocks until roscore is online, then executes the remaining startup commands.

---

### autosync_exp
Sync utility that automatically:
- Pushes code host → robot  
- Builds the workspace on the robot  
- Retrieves newly created ROS bags robot → host  

---

# 🧰 TERMINATOR CONFIGURATION

### config  
(~/.config/terminator/)  
Defines:
- window layouts  
- tab structure  
- profile commands  
- integration with with_roscore.sh

---

### cepheus_lab.desktop  
(~/Desktop/)  
Desktop shortcut that launches the full lab UI via start_cepheus_lab.sh.

---

# 🤖 ROBOT-SIDE SCRIPTS (robot_home/)

### robot_env.sh  
Environment setup for the robot’s **new workspace** (~/cepheus_ws_v2).

---

### robot_env_old.sh  
Legacy version for the **old workspace** (~/cepheus_ws).  
Kept for backward compatibility.

---

# 📊 BAG CONVERSION & DIAGNOSTIC PLOTTING  
(cepheus_ws_v2/bags/converters/)

### bag2mat_plotter.py

Converts a ROS bag into a MATLAB `.mat` file and generates a **4×4 diagnostic plot**:
1. Position (actual vs desired)  
2. Velocity (actual vs ref)  
3. Position error  
4. Torque  

For joints q1, q2, q3, and θ₀.  
Assumes **100 Hz**.

#### Usage

```
python3 bag2mat_plotter.py BAGNAME [--bagdir PATH] [--no-plot]
```

#### Arguments

| Argument | Description |
|---------|-------------|
| BAGNAME | Bag name without `.bag` |
| --bagdir PATH | Optional bag directory (default: ~/cepheus_ws_v2/bags/) |
| --no-plot | Only generate `.mat` (no PNG plot) |

#### Example

```
python3 bag2mat_plotter.py test_run_42
```

Outputs:
- test_run_42.mat  
- test_run_42.png  

---

# 📈 PLOTJUGGLER LAYOUTS  
(Documents/)

### arm_visualizer.xml  
PlotJuggler layout with:
- joint angles  
- velocities  
- Vicon base data  
- rad→deg math snippets  

---

