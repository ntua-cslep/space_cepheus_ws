# Cepheus ROS1 Workspace

This is a ROS1 catkin workspace for the **Cepheus** planar space emulator robot.

> **Status:** Under active refactoring. Structure, package roles, and documentation will change.

This README is intentionally minimal and editable as the workspace evolves.

---

## Workspace Layout (current snapshot)

```
.
├── .catkin_tools/      # catkin_tools metadata
├── .git/               # Git repo
├── build/              # build artifacts (ignored)
├── devel/              # devel space (ignored)
├── docker/             # Dockerfile, compose, Docker_README.md
├── docs/               # system/software docs (TBD)
├── logs/               # catkin build logs
├── model/              # top-level URDF(s)
├── src/                # ROS packages and external drivers
├── tools/              # plotting, sync, env and terminal setup
├── .dockerignore       # Docker ignore rules
└── .gitignore          # Git ignore rules
```

---

## Ignore Rules

This workspace uses the following ignore patterns (merged from your uploaded `.gitignore` and `.dockerignore`):

### Git Ignore Highlights

* Catkin and build artifacts (`build/`, `devel/`, `.catkin_tools/`)
* Logs (`logs/`)
* Generated ROS files (`msg_gen/`, `srv_gen/`, `*.pyc`, `*.pcd`)
* IDE/editor configs (`.vscode/`, `*.user`, `qtcreator-*`)
* Generated CMake artifacts
* Bagfiles, large data, temporary files

### Docker Ignore Highlights

* Entire Git repo metadata (`.git/`)
* Build outputs (`build/`, `devel/`)
* Logs, editor configs, data dumps
* Anything not required for container build context

These will be updated as the refactor progresses.

---

## ROS Packages (src/)

The `src/` directory currently contains a **mix of old (legacy) and new (refactored) packages and nodes**.
This is intentional during the restructuring process. Multiple versions of planners, controllers,
hardware interfaces, and experiment packages coexist temporarily.

Only a high‑level grouping is provided for now:

* **cepheus_robot/** – robot models, hardware drivers, legacy + partial refactor code
* **cepheus_control/** – higher-level control nodes (old + WIP)
* **exp_* packages** – joint‑space and Cartesian‑space experiments (old and new variants)
* **hw_interface_new/** – new hardware interface under development
* **external drivers** – Vicon bridge, Xsens IMU driver, force/torque sensor

A detailed package description will be added once the structure stabilises.

---

## Tools

Located in `tools/`:

* **plotting/** — bag → .mat conversion and plotting
* **setup/** — environment scripts (`ros_env.sh`, robot env scripts)
* **sync/** — experiment sync helper (`autosync_exp`)
* **terminator_setup/** — terminal layouts and launch helpers

---

## Docker Environment

The `docker/` directory contains everything for running the workspace in a reproducible container:

* `Dockerfile`
* `compose.yml`
* `Docker_README.md`
* `.env.example`

Used for running ROS1 tooling outside the host OS and keeping builds isolated.

Copy `.env.example` to `.env` and set:
- ROS_MASTER_URI = address of your ROS master
- ROS_IP = your machine’s IP on the same network

🚀 ROS1 Docker Environment

This repository includes a fully containerized ROS1 (Noetic) development environment used for all software work on the Cepheus planar space emulator.
The setup uses Docker Compose + Makefile automation to ensure a clean, reproducible workflow without modifying user shell configs.

1. Setup Instructions
1.1 Create your local .env file

The repo contains a template:

.env.example


Copy it and fill in your machine-specific ROS networking values:

cp .env.example .env


Edit .env:

# IP of the ROS master (host or lab PC)
ROS_MASTER_URI=http://<MASTER_IP>:11311

# The IP of *your* machine on the same network
ROS_IP=<YOUR_IP>

# Host UID/GID for correct file permissions inside the container
UID=1000
GID=1000


.env is gitignored.
Each user maintains their own ROS_IP / MASTER_URI based on their network setup.

2. Running the Environment

All interaction with the Docker environment is handled through a Makefile located in the repo root.

Available commands:
make up        # Build + start + enter container
make upfast    # Start without rebuilding + enter container
make shell     # Open a shell in the running container
make down      # Stop and remove the container

Typical workflow

Start the system:

make up


Re-enter the running container:

make shell


Stop everything:

make down

3. Notes

The container uses host networking for ROS1 compatibility.

Your workspace is mounted inside the container at:

/home/pilot/space_cepheus_ws


GUI tools (rviz, rqt_graph, PlotJuggler, etc.) work through X11.
If required, allow X access:

xhost +local:docker


No ROS variables (ROS_MASTER_URI, ROS_IP) should be added to .bashrc or .zshrc.
Docker handles all networking config via .env.

4. Verification

After make up, inside the container:

echo $ROS_MASTER_URI
echo $ROS_IP
rosnode list


These should reflect your .env settings and connect to the ROS master correctly.

---

## Building

Standard catkin workspace:

```
cd /path/to/workspace
catkin init   # optional, first time
catkin build
source devel/setup.bash
```

---

## Notes / TODO (editable)

* [ ] Decide active packages vs legacy packages
* [ ] Unify experiment structure
* [ ] Clean hardware interface into a single entry point
* [ ] Clarify Vicon/IMU interface responsibilities
* [ ] Move high-level docs into `docs/`
* [ ] Add minimal launch instructions once stable

This document stays short until the workspace stabilizes.
