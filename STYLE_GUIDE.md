# Cepheus Project — Naming & Architecture Style Guide

> **Purpose**: This document defines the **naming conventions, architectural principles, and structural rules** for the *clean* Cepheus codebase.
>
> It exists to prevent regression into legacy chaos and to ensure the system remains **readable, teachable, and thesis‑grade**.

---

## 1. Core Philosophy

- **Clarity over cleverness**
- **Consistency over personal taste**
- **Architecture over patches**

If something is ambiguous, choose the option that:
1. Reads better to a new engineer
2. Matches ROS and C++ conventions
3. Minimizes cognitive load

---

## 2. Hyphens vs Underscores — The Golden Rule

> **Hyphens (`-`) are for labels and phrases**  
> **Underscores (`_`) are for identifiers**

This rule applies differently depending on *role*.

### Repository Policy Files (Exception)

Files that define **repository-wide rules or contracts** intentionally follow a different convention:

```text
README.md
LICENSE
CONTRIBUTING.md
STYLE_GUIDE.md
CODE_OF_CONDUCT.md
```

These files:
- Live at the **repository root**
- Are treated as canonical policy, not narrative documentation
- Commonly use **ALL CAPS** and may use underscores

This is a deliberate exception and does **not** apply to general documentation.

---

## 3. Git Branch Naming


## 3. Git Branch Naming

**Use hyphens (`-`)** — branches are human-facing.

### Format
```text
<category>/<short-descriptive-name>
```

### Examples
```text
clean/exp-hw-arch
feature/sensor-fusion
tooling/post-processing
fix/imu-frame
```

### Disallowed
```text
clean_exp_hw_arch
refactor/stuff
new/v2
```

Branches must read like **short English phrases**.

---

## 4. Repository & Package Architecture

### Package Naming (ROS)

**Use snake_case (`_`) only.**

```text
cepheus_experiments
cepheus_robot_clean
cepheus_bringup_clean
```

Rationale:
- Matches ROS conventions
- Avoids shell and launch-file issues
- Maps cleanly to namespaces

---

## 5. Code Naming Conventions (C++ / Python)

### 5.1 Variables & Class Members

**snake_case**

```cpp
double base_pose_x;
Eigen::Vector3d imu_accel_body;
ros::Publisher reaction_wheel_cmd_pub;
```

---

### 5.2 Functions & Methods

**snake_case**

```cpp
void update_state_estimate();
bool reset_ft_wrench();
Eigen::Vector3d compute_body_velocity();
```

---

### 5.3 Classes & Structs

**PascalCase**

```cpp
class ExperimentNode;
class NodeContext;
struct ExperimentConfig;
```

Classes must represent **clear concepts**, not procedural blobs.

---

### 5.4 Constants

**SCREAMING_SNAKE_CASE**

```cpp
constexpr double GRAVITY = 9.80665;
constexpr int CONTROL_RATE_HZ = 100;
```

---

### 5.5 Namespaces

**snake_case**

```cpp
namespace cepheus_experiments {
namespace utils {
```

---

## 6. File Naming

### Source Code

**snake_case**

```text
experiment_node.cpp
node_context.hpp
imu_preintegration.cpp
reaction_wheel_model.cpp
```

---

### Documentation & Diagrams

**hyphen-separated**

```text
experiment-architecture.md
system-layout.md
legacy-notes.md
```

Docs are human-facing — readability wins.

---

## 7. ROS-Specific Naming Rules

### Node Names

```cpp
ros::init(argc, argv, "cepheus_experiment_node");
```

---

### Topics

```text
/base/pose_estimate
/imu/data_raw
/reaction_wheel/command_torque
```

---

### Parameters (YAML & Code)

```yaml
control_rate_hz: 100
use_vicon_pose: true
reaction_wheel_max_torque: 0.2
```

```cpp
nh.param("control_rate_hz", control_rate_hz, 100);
```

Never use hyphens in parameters.

---

## 8. Architecture Rules (Non-Negotiable)

### 8.1 No Monolithic Nodes

- One node = one responsibility
- No "god nodes"
- No experiment logic mixed with hardware drivers

---

### 8.2 Clear Separation of Concerns

- **Experiments** → sequencing, logic, triggers
- **Robot interface** → hardware I/O, limits, safety
- **Estimation** → sensor fusion, filtering
- **Visualization / logging** → read-only

---

### 8.3 Configuration Lives in YAML

- Gains
- Thresholds
- Durations
- Limits

**Never hardcode tunables in C++.**

---

### 8.4 TF Is a First-Class Citizen

All frames must be:
- Explicit
- Documented
- Static or dynamic by design

No magic transforms.

---

## 9. Third-Party Libraries

Do **not** rename or fight external APIs.

Instead:
- Adapt at the boundary
- Wrap if needed
- Keep core code consistent

---

## 10. Enforcement

This style guide applies to:
- All new code
- All clean-branch development
- All student contributions

Legacy code is exempt **only until it is replaced**.

---

## 11. Final Principle

> **If you have to think about naming, the system is already failing.**

This guide exists so you don’t have to.

Keep it boring. Keep it clean. Keep it correct.

