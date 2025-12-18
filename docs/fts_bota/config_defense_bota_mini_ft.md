# Cepheus Force Torque Sensor Configuration

**Sensor:** Bota Sys MiniOne Pro Gen-0 Serial  
**Project:** Cepheus Planar Space Emulator  
**ROS:** ROS1  
**Last validated:** 2025-12-16

This document describes and defends the **force–torque sensor filtering configuration** used on the Cepheus planar space‑robot platform. It serves as **internal engineering documentation** supporting configuration choices, maintenance, servicing, and system‑level control design decisions related to the MiniONE Pro Gen‑0 force–torque sensor.

The emphasis is on **why the configuration is what it is**, not merely what values were chosen.

---

## 1. System Context

The Cepheus robot operates strictly in a **horizontal planar workspace**, with translational motion constrained to the plane and a single rotational degree of freedom about the vertical axis. Nominal motion is deliberately slow (e.g., a 30 cm trajectory executed over 10–20 seconds), resulting in task‑level motion frequencies on the order of **0.05–0.2 Hz**.

Despite the slow kinematics, the system requires **accurate, low‑noise, and responsive force–torque measurements** during:

- contact initiation,
- frictional transitions,
- small external disturbances, and
- impedance‑based interaction control.

The Bota Systems **MiniONE Pro Gen‑0 (serial)** force–torque sensor provides configurable internal digital filtering via **SINC decimation** and an optional **FIR low‑pass stage**. These settings directly affect bandwidth, latency, and control‑relevant signal quality.

---

## 2. Relevant Frequency Characteristics

### 2.1 Motion‑Induced Frequencies

Robot motion is quasi‑static (< 1 Hz). Any force signal content above a few hertz is not related to commanded trajectories and must therefore originate from noise, contact dynamics, or disturbances.

### 2.2 Interaction and Contact Dynamics

Even under slow motion, interaction forces can vary rapidly:

- contact onset occurs within tens of milliseconds,
- stick–slip effects introduce mid‑frequency components,
- operator or environment disturbances often fall within the **10–60 Hz** range.

The filtering strategy must therefore:

- **preserve transient force information** relevant to interaction control, while
- **attenuating high‑frequency noise** that degrades controller stability.

---

## 3. Filtering Capabilities of the MiniONE Pro

The MiniONE Pro supports:

- **SINC (CIC) decimation filtering** with selectable window sizes (57–512), and
- an optional **FIR low‑pass filter** applied after decimation.

### 3.1 SINC‑Only Filtering (FIR Disabled)

- Implements long‑window averaging
- Produces smooth signals with reduced noise
- Introduces **significant group delay** at large SINC values
- Exhibits weak stop‑band attenuation
- Smears short‑duration force transients

### 3.2 SINC + FIR Filtering

- Uses a short SINC stage for decimation
- FIR stage provides sharper cutoff and better stop‑band rejection
- Latency is lower and more predictable
- Better suited for real‑time force feedback and impedance control

---

## 4. Candidate Configurations Evaluated

Two configurations with comparable nominal output rates were evaluated:

### A. SINC = 512, FIR Disabled

- Output rate: ~97 Hz
- Cutoff frequency: ~48.8 Hz

**Advantages:**

- Very smooth output
- Low apparent noise floor

**Disadvantages:**

- **High internal latency** due to long averaging window
- Poor transient fidelity
- Reduced achievable control gains

---

### B. SINC = 57, FIR Enabled (Selected)

- Output rate: ~101 Hz
- Cutoff frequency: ~50 Hz

**Advantages:**

- Clear and timely impulse response
- Strong high‑frequency noise suppression
- **Lower and more predictable latency**
- Natural alignment with a 100 Hz control loop

**Trade‑off:**

- Slightly higher raw noise floor than SINC‑512, effectively mitigated by the FIR stage

Although the cutoff frequencies appear similar on paper, the internal signal behavior and latency characteristics differ substantially.

---

## 5. Rationale for the Selected Configuration

### 5.1 Control‑Loop Synchronization

The Cepheus control loop operates at approximately **100 Hz**. A sensor output rate of ~101 Hz ensures:

- one wrench measurement per control cycle,
- no interpolation or buffering,
- deterministic timing between sensing and actuation.

### 5.2 Responsiveness to Contact Events

A usable bandwidth of ~50 Hz is well above the system’s motion frequencies while remaining sensitive to contact and interaction transients. This is critical for stable impedance behavior.

### 5.3 Latency as a Stability Constraint

In force and impedance control, **latency dominates noise** as a limiting factor. Excessive group delay reduces achievable stiffness and damping gains and increases oscillatory risk.

The SINC = 57 + FIR configuration minimizes delay while maintaining sufficient smoothing.

### 5.4 Noise Suppression Without Over‑Averaging

The FIR stage provides aggressive attenuation above the cutoff frequency without relying on long SINC windows. This prevents spurious force spikes and torque jitter from entering the controller.

### 5.5 Suitability for Planar Operation

In the planar Cepheus configuration:

- gravitational loading on the FT sensor is static,
- gravity compensation is unnecessary,
- bias handling is decoupled from filtering and performed via runtime zeroing.

The filter therefore focuses exclusively on noise suppression and transient fidelity.

---

## 6. Summary

The selected **Cepheus force–torque sensor configuration**—**SINC = 57 with FIR enabled**—provides:

- ~101 Hz output rate,
- ~50 Hz effective bandwidth,
- low and predictable latency,
- strong noise attenuation,
- reliable transient force detection.

This configuration represents a deliberate and defensible balance between bandwidth, latency, and signal quality, and forms a stable foundation for future impedance and interaction control development.

---

Future extensions of this document may include explicit parameter listings, firmware versions, and validation plots to support long‑term maintenance and reproducibility.