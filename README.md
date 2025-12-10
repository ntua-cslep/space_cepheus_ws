# 🚀 **Cepheus ROS1 Workspace**

This repository contains the ROS1 (Noetic) workspace for **Cepheus**, the laboratory’s planar micro-gravity **space-robot emulator** used for research in *On-Orbit Servicing (OOS)*, free-floating manipulation, and advanced control strategies.

---

## 🛰️ **About the Cepheus Planar Space Emulator**

**Cepheus** is a 2-DoF free-floating robotic platform operating on air bearings to emulate spacecraft motion with *near-frictionless* dynamics in a 2D plane. This setup enables realistic experimental validation of algorithms for:

- autonomous capture and servicing of orbital targets
- cooperative manipulation between free-flying robots
- impact-minimizing rendezvous and docking interactions
- free-floating manipulators with angular momentum coupling
- reactionless actuation mechanisms
- debris-capture and OOS operations

Key system characteristics:

- **Lightweight mechanical design** for accurate inertial behavior
- **Compact transmission mechanisms**
- **Full onboard autonomy** (propulsion, computing, powern)
- **High-fidelity sensing** (Vicon MoCap, Xsens IMU)

The system currently consists of an **active planar robot** capable of controlled free-floating motion and onboard decision-making and a **passive planar robot** used as a target.
---

## 🧭 **Repository Structure**

Based on the live GitHub structure:

```
space_cepheus_ws/
├── CMakeLists.txt        # Top-level catkin workspace CMake file
├── README.md             # This document
├── docker/               # Dockerfile, compose, Docker_README.md, Makefile
├── docs/                 # Documentation (system, architecture, etc.)
├── model/                # URDFs and robot models
├── src/                  # ROS packages (legacy + refactored)
│   ├── cepheus_robot/
│   ├── cepheus_robot_new/
│   ├── exp_* (experiment packages)
│   ├── vicon_bridge/
│   └── external drivers
├── tools/                # Plotting, sync scripts, terminator layouts
└── .gitignore
```

No `devel/` or `build/` directories appear in the repo—they're generated locally by `catkin_make` and ignored by Git.

---

## 📦 **ROS Packages (`src/`)**

A mix of legacy and refactored packages currently coexist during the transition.

Categories include:

- **cepheus\_robot/** — robot models, hardware drivers, interfastructure nodes

- **experiment packages (exp_*)/** — joint-space and Cartesian-space experiments

- **vicon\_bridge/** — motion capture interface

- **external drivers** — Xsens IMU, auxiliary sensors

A complete package reference will be added once the architecture stabilizes.

---

## 🛠️ **Tools (`tools/`)**

- **plotting/** — ROS bag → `.mat` converters and plotting scripts
- **setup/** — ROS environment scripts (`ros_env.sh`, robot env setup)
- **sync/** — automated experiment/bag synchronization utilities
- **terminator\_setup/** — preconfigured multi-pane terminal layouts

---

## 🐋 **Docker Environment**

A reproducible ROS1 development environment is provided under `docker/` using Docker Compose + a Makefile wrapper.

### 1. Prepare configuration

```bash
cp .env.example .env
```

Edit `.env` to match your network:

```
ROS_MASTER_URI=http://<MASTER_IP>:11311
ROS_IP=<YOUR_IP>
UID=1000
GID=1000
```

### 2. Run the environment

```bash
make up        # Build + start + enter container
make upfast    # Start without rebuild
make shell     # Enter running container
make down      # Stop/remove container
```

The workspace is mounted inside the container at:

```
/home/pilot/space_cepheus_ws
```

For GUI tools (RViz, rqt, PlotJuggler):

```bash
xhost +local:docker
```

---

## 🔧 **Building (catkin\_make)**

This workspace uses **catkin\_make**:

```bash
cd space_cepheus_ws
catkin_make
source devel/setup.bash
```

---

## 🎓 **Audience**

This repository is designed for:

- **Master’s students** working on estimation, control, and robotics
- **Researchers** validating OOS-related algorithms on a free-floating robot
- **Developers** extending the Cepheus software stack

Documentation is written to be technically precise while remaining accessible to those with basic ROS1 and robotics experience.
