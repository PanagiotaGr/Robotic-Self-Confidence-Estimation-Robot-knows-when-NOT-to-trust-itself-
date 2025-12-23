# Robotic Self-Confidence Estimation  
### *A Framework for Confidence-Aware Robot Navigation*  

---

## 1. Overview

This repository contains a ROS 2 (Jazzy) implementation of a **self-confidence estimation framework** for mobile robots.  
The core objective is to enable a robot to **reason about its own reliability** and adapt its navigation strategy when its perception and localization systems become uncertain or degraded.

Instead of assuming that the robot always “trusts” its sensors and algorithms, we introduce an explicit **confidence signal**:

\[
C(t) \in [0, 1]
\]

which is computed online and used to modulate the robot’s behaviour between:
- **Aggressive navigation** (high confidence)
- **Conservative navigation** (medium confidence)
- **Safe / fallback mode** (low confidence; eg. stop, slow down, or ask for help)

This idea is particularly relevant to **autonomous driving, field robotics, and safety-critical navigation**, where overconfident systems can be dangerous and underconfident systems can be inefficient.

---

## 2. Problem Formulation & Motivation

Modern robotic systems rely on a complex stack of perception and localization components (e.g., VO, SLAM, sensor fusion, mapping). In practice, these components:

- Are affected by sensor noise, occlusions, lighting conditions, dynamic obstacles.
- Can degrade or fail silently, causing catastrophic behaviour if the planner keeps trusting them.
- Typically do **not** expose an explicit, calibrated “trust” signal to downstream modules.

The key research question is:

> **Can we design a principled confidence estimator that aggregates uncertainty cues from the perception stack and exposes a scalar signal \(C(t)\), which can be used by the navigation system to improve safety and robustness?**

This repository implements a **first, modular baseline** for such a system.

---

## 3. Methodology

### 3.1 Confidence Inputs

The confidence estimator ingests four normalized signals:

- **Visual Odometry Uncertainty**  
  `vo_uncertainty ∈ [0, 1]`  
  Interpreted as: 0 = highly reliable VO, 1 = highly uncertain VO.

- **SLAM Quality**  
  `slam_quality ∈ [0, 1]`  
  Interpreted as: 0 = poor SLAM tracking / mapping, 1 = high-quality SLAM.

- **Drift Estimate (Normalized)**  
  `drift_norm ∈ [0, 1]`  
  0 = negligible drift; 1 = severe drift (e.g., large discrepancy between odometry and ground truth or loop closures).

- **Sensor Noise Level**  
  `sensor_noise ∈ [0, 1]`  
  0 = clean measurements; 1 = very noisy sensing (e.g., heavy LiDAR or camera noise).

All inputs are assumed to be **pre-normalized** to \([0, 1]\) by upstream modules or simple normalization heuristics.

---

### 3.2 Confidence Computation

The confidence estimator computes a scalar score:

\[
C(t) = \text{clip}_{[0,1]} \big(
w_{\text{vo}} (1 - u_{\text{vo}}) + 
w_{\text{slam}} q_{\text{slam}} +
w_{\text{drift}} (1 - d) +
w_{\text{noise}} (1 - n)
\big)
\]

where:

- \(u_{\text{vo}}\) = `vo_uncertainty`  
- \(q_{\text{slam}}\) = `slam_quality`  
- \(d\) = `drift_norm`  
- \(n\) = `sensor_noise`  
- \(w_{\text{vo}}, w_{\text{slam}}, w_{\text{drift}}, w_{\text{noise}}\) are non-negative weights
- The weights are normalized internally such that \(\sum_i w_i = 1\).

Intuitively:

- High uncertainty, drift, and noise **decrease** confidence.
- High SLAM quality **increases** confidence.

This design is deliberately simple and transparent, making it easy to extend later with:
- Learning-based models,
- Bayesian fusion,
- Calibration techniques.

---

### 3.3 Behaviour Switching

The confidence score is mapped to **navigation modes** via two thresholds:

- `th_aggressive` (default: 0.7)
- `th_conservative` (default: 0.4)

The mapping is:

- If \(C(t) \geq \text{th_aggressive}\) → `AGGRESSIVE`
- Else if \(C(t) \geq \text{th_conservative}\) → `CONSERVATIVE`
- Else → `SAFE`

Each mode can be associated with different:
- Maximum linear/Angular velocity,
- Obstacle clearance margins,
- Planner cost parameters,
- Fallback or stop behaviours.

In this baseline implementation, the mode directly controls a **velocity limit**.

---

## 4. ROS 2 System Architecture

The system is organized into two core ROS 2 nodes:

### 4.1 `confidence_estimator` Node

**Responsibilities:**
- Subscribe to perception and localization quality indicators.
- Compute a normalized confidence score \(C(t)\).
- Publish:
  - `/robot_confidence` (Float32)
  - `/navigation_mode` (String)

**Subscriptions:**

| Topic              | Type               | Description                               |
|--------------------|--------------------|-------------------------------------------|
| `/vo_uncertainty`  | `std_msgs/Float32` | VO uncertainty, 0 = good, 1 = bad        |
| `/slam_quality`    | `std_msgs/Float32` | SLAM quality, 0 = poor, 1 = excellent    |
| `/drift_norm`      | `std_msgs/Float32` | Normalized drift [0, 1]                  |
| `/sensor_noise`    | `std_msgs/Float32` | Normalized noise level [0, 1]            |

**Publications:**

| Topic               | Type               | Description                                       |
|---------------------|--------------------|---------------------------------------------------|
| `/robot_confidence` | `std_msgs/Float32` | Scalar confidence \(C(t)\) ∈ [0, 1]              |
| `/navigation_mode`  | `std_msgs/String`  | `AGGRESSIVE`, `CONSERVATIVE`, or `SAFE`          |

---

### 4.2 `navigation_behavior` Node

**Responsibilities:**
- Subscribe to `/navigation_mode`.
- Map the current mode to a **velocity limit**.
- Publish:
  - `/cmd_vel_limit` (Float32)

**Publications:**

| Topic            | Type               | Description                                 |
|------------------|--------------------|---------------------------------------------|
| `/cmd_vel_limit` | `std_msgs/Float32` | Maximum allowed linear velocity (m/s)       |

This node is intended to be integrated with:
- A navigation stack (e.g., Nav2) as a constraint,
- A velocity filter that clips `/cmd_vel` commands according to `/cmd_vel_limit`.

---

## 5. Implementation Details

- **ROS 2 Distribution:** Jazzy
- **Language:** Python (`rclpy`)
- **Package name:** `robot_confidence`
- **Build type:** `ament_python`

The configurable parameters (weights, thresholds, speed limits) are provided via a YAML configuration file and loaded in the launch file.

---

## 6. Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Lily-Evan/Robotic-Self-Confidence-Estimation-Robot-knows-when-NOT-to-trust-itself-.git
cd ..
colcon build --packages-select robot_confidence
source install/setup.bash
