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
- Maximum linear/angular velocity,
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
```

---

## 7. Launch

To start the full confidence-aware behaviour system:

```bash
ros2 launch robot_confidence confidence_system.launch.py
```

This launches:
- `confidence_estimator`
- `navigation_behavior`

with parameters loaded from `config/confidence_params.yaml`.

---

## 8. Quick Testing (Synthetic Inputs)

To quickly test the pipeline, you can manually publish test values.

### 8.1 Publish perception signals

In a second terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Then, for example:

```bash
ros2 topic pub /vo_uncertainty std_msgs/Float32 "data: 0.1" -1
ros2 topic pub /slam_quality std_msgs/Float32 "data: 0.9" -1
ros2 topic pub /drift_norm std_msgs/Float32 "data: 0.2" -1
ros2 topic pub /sensor_noise std_msgs/Float32 "data: 0.3" -1
```

### 8.2 Observe outputs

```bash
ros2 topic echo /robot_confidence
ros2 topic echo /navigation_mode
ros2 topic echo /cmd_vel_limit
```

With the default parameters, this configuration should produce:
- High confidence (≈ 0.8+)
- Mode: `AGGRESSIVE`
- Velocity limit: ≈ 0.8 m/s

---

## 9. Evaluation Plan (Research-Oriented)

This repository provides an implementation that can be used as the basis for an empirical study.

### 9.1 Experimental Conditions

Compare:

1. **Baseline navigation**  
   - No confidence estimation; fixed navigation parameters.

2. **Confidence-aware navigation (this system)**  
   - Confidence estimator enabled.
   - Behaviour and velocity limits adapted from `/navigation_mode`.

### 9.2 Suggested Metrics

- **Collision Rate**  
  Number of collisions per episode / trajectory.

- **Mission Success Rate**  
  Percentage of trials where the robot reaches its goal.

- **Recovery Success**  
  Probability that the robot recovers from degraded perception by entering conservative or safe mode instead of failing.

- **Safety Under Degraded Perception**  
  Performance when:
  - VO uncertainty is artificially increased.
  - SLAM tracking quality is degraded.
  - Sensor noise is injected or simulated.

### 9.3 Ablation & Sensitivity

- Effect of different weight configurations \(w_i\).
- Effect of different thresholds (`th_aggressive`, `th_conservative`).
- Offset or delay between perception degradation and mode adaptation.

---

## 10. Extensions & Future Work

This baseline is intentionally simple and transparent, so that it can be extended in multiple directions, including:

- **Learning-based Confidence Estimation**  
  Training a small neural network to predict \(C(t)\) from a richer feature set (e.g., covariance matrices, tracking status, loop closures, etc.).

- **Calibration Analysis**  
  Evaluating how well the predicted confidence correlates with actual failure probabilities (e.g., via reliability diagrams, Brier score).

- **Task-Aware Confidence**  
  Extending from generic navigation confidence to task-specific confidence (e.g., exploration, manipulation, human-robot interaction).

- **Human-in-the-Loop**  
  Integrating confidence as a signal for requesting operator assistance or supervision.

---

## 11. Repository Structure

```text
robot_confidence/
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  ├── resource/
  │   └── robot_confidence
  ├── robot_confidence/
  │   ├── __init__.py
  │   ├── confidence_node.py
  │   └── navigation_behavior_node.py
  ├── config/
  │   └── confidence_params.yaml
  └── launch/
      └── confidence_system.launch.py
```

---

## 12. Author & Acknowledgements

**Author:**  
Panagiota Grosdouli  

This work is part of an academic exploration of **confidence-aware robotics** and is designed to be easily integrated into existing ROS 2 navigation and SLAM pipelines for research and experimentation.

---

## 13. License

This project is released under the **MIT License**.  
You are free to use, modify, and extend it for research or educational purposes, with appropriate attribution.
