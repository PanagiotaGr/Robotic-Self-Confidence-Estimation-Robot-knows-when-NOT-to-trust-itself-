# Robotic Self-Confidence Estimation

### A Framework for Confidence-Aware Robot Navigation (ROS 2 Jazzy)

> A modular ROS 2 (Jazzy) baseline that estimates a robot’s **self-confidence** online and adapts navigation behaviour to improve safety and robustness under degraded perception/localization.

---

## Table of Contents

* [Overview](#overview)
* [Motivation](#motivation)
* [Methodology](#methodology)

  * [Confidence Inputs](#confidence-inputs)
  * [Confidence Computation](#confidence-computation)
  * [Behaviour Switching](#behaviour-switching)
* [ROS 2 System Architecture](#ros-2-system-architecture)

  * [`confidence_estimator` Node](#confidence_estimator-node)
  * [`navigation_behavior` Node](#navigation_behavior-node)
* [Implementation Details](#implementation-details)
* [Installation](#installation)
* [Launch](#launch)
* [Quick Testing (Synthetic Inputs)](#quick-testing-synthetic-inputs)
* [Evaluation Plan](#evaluation-plan)
* [Extensions & Future Work](#extensions--future-work)
* [Repository Structure](#repository-structure)
* [Author](#author)
* [License](#license)

---

## Overview

This repository contains a ROS 2 (Jazzy) implementation of a **self-confidence estimation framework** for mobile robots.
The objective is to enable a robot to **reason about its own reliability** and adapt navigation strategy when perception and localization become uncertain or degraded.

We introduce an explicit confidence signal:

[
C(t) \in [0, 1]
]

computed online and used to modulate behaviour between:

* **Aggressive navigation** (high confidence)
* **Conservative navigation** (medium confidence)
* **Safe / fallback mode** (low confidence; e.g., stop, slow down, or request help)

This is especially relevant to **autonomous driving, field robotics, and safety-critical navigation**, where overconfidence can be dangerous and underconfidence can be inefficient.

---

## Motivation

Modern robotic stacks depend on perception and localization modules (VO, SLAM, sensor fusion, mapping). In practice, these components:

* Are affected by noise, occlusions, lighting changes, and dynamic obstacles.
* Can degrade or fail silently, while the planner keeps “trusting” the outputs.
* Typically do **not** expose a calibrated “trust” signal to downstream navigation.

**Research question:**

> Can we design a principled confidence estimator that aggregates uncertainty cues from the perception stack and exposes a scalar signal (C(t)) usable by navigation to improve safety and robustness?

This repository implements a **first, modular baseline**.

---

## Methodology

### Confidence Inputs

The confidence estimator ingests four normalized signals:

* **Visual Odometry Uncertainty**
  `vo_uncertainty ∈ [0, 1]`
  0 = highly reliable VO, 1 = highly uncertain VO.

* **SLAM Quality**
  `slam_quality ∈ [0, 1]`
  0 = poor SLAM tracking/mapping, 1 = high-quality SLAM.

* **Drift Estimate (Normalized)**
  `drift_norm ∈ [0, 1]`
  0 = negligible drift, 1 = severe drift (e.g., discrepancy between odometry and ground truth or loop closures).

* **Sensor Noise Level**
  `sensor_noise ∈ [0, 1]`
  0 = clean measurements, 1 = very noisy sensing (e.g., heavy LiDAR or camera noise).

All inputs are assumed to be **pre-normalized** to ([0,1]) by upstream modules or simple heuristics.

---

### Confidence Computation

The scalar confidence score is computed as:

[
C(t) = \text{clip}*{[0,1]} \Big(
w*{\text{vo}} (1 - u_{\text{vo}}) +
w_{\text{slam}} q_{\text{slam}} +
w_{\text{drift}} (1 - d) +
w_{\text{noise}} (1 - n)
\Big)
]

where:

* (u_{\text{vo}}) = `vo_uncertainty`
* (q_{\text{slam}}) = `slam_quality`
* (d) = `drift_norm`
* (n) = `sensor_noise`
* (w_{\text{vo}}, w_{\text{slam}}, w_{\text{drift}}, w_{\text{noise}}) are non-negative weights
* Weights are normalized internally so that (\sum_i w_i = 1)

**Intuition**

* High VO uncertainty, drift, and noise **decrease** confidence.
* High SLAM quality **increases** confidence.

This design is deliberately simple and transparent, enabling future extensions (learning-based models, Bayesian fusion, calibration).

---

### Behaviour Switching

Confidence is mapped to **navigation modes** via thresholds:

* `th_aggressive` (default: 0.7)
* `th_conservative` (default: 0.4)

Mapping:

* If (C(t) \ge \text{th_aggressive}) → `AGGRESSIVE`
* Else if (C(t) \ge \text{th_conservative}) → `CONSERVATIVE`
* Else → `SAFE`

Each mode can correspond to different:

* Maximum linear/angular velocity
* Obstacle clearance margins
* Planner cost parameters
* Fallback/stop behaviours

In this baseline, the mode directly controls a **velocity limit**.

---

## ROS 2 System Architecture

Two ROS 2 nodes:

```
/vo_uncertainty   /slam_quality   /drift_norm   /sensor_noise
      \              |             /              /
       \             |            /              /
        +---------------- confidence_estimator ----------------+
        |                  publishes:                          |
        |   /robot_confidence (Float32), /navigation_mode       |
        +------------------------+------------------------------+
                                 |
                                 v
                    navigation_behavior
                     publishes:
                     /cmd_vel_limit (Float32)
```

---

### `confidence_estimator` Node

**Responsibilities**

* Subscribes to perception/localization indicators
* Computes (C(t))
* Publishes confidence and navigation mode

**Subscriptions**

| Topic             | Type               | Description                           |
| ----------------- | ------------------ | ------------------------------------- |
| `/vo_uncertainty` | `std_msgs/Float32` | VO uncertainty, 0 = good, 1 = bad     |
| `/slam_quality`   | `std_msgs/Float32` | SLAM quality, 0 = poor, 1 = excellent |
| `/drift_norm`     | `std_msgs/Float32` | Normalized drift [0, 1]               |
| `/sensor_noise`   | `std_msgs/Float32` | Normalized noise level [0, 1]         |

**Publications**

| Topic               | Type               | Description                          |
| ------------------- | ------------------ | ------------------------------------ |
| `/robot_confidence` | `std_msgs/Float32` | Scalar confidence (C(t)) ∈ [0,1]     |
| `/navigation_mode`  | `std_msgs/String`  | `AGGRESSIVE`, `CONSERVATIVE`, `SAFE` |

---

### `navigation_behavior` Node

**Responsibilities**

* Subscribes to `/navigation_mode`
* Maps mode to a velocity limit
* Publishes `/cmd_vel_limit`

**Publications**

| Topic            | Type               | Description                       |
| ---------------- | ------------------ | --------------------------------- |
| `/cmd_vel_limit` | `std_msgs/Float32` | Max allowed linear velocity (m/s) |

Integration targets:

* Nav2 constraints
* A velocity filter that clips `/cmd_vel` according to `/cmd_vel_limit`

---

## Implementation Details

* **ROS 2 Distribution:** Jazzy
* **Language:** Python (`rclpy`)
* **Package:** `robot_confidence`
* **Build type:** `ament_python`

Configurable parameters (weights, thresholds, speed limits) are provided via YAML and loaded in the launch file.

---

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Lily-Evan/Robotic-Self-Confidence-Estimation-Robot-knows-when-NOT-to-trust-itself-.git
cd ..
colcon build --packages-select robot_confidence
source install/setup.bash
```

---

## Launch

```bash
ros2 launch robot_confidence confidence_system.launch.py
```

Launches:

* `confidence_estimator`
* `navigation_behavior`

with parameters loaded from `config/confidence_params.yaml`.

---

## Quick Testing (Synthetic Inputs)

### 1) Publish perception signals

In a second terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Then publish example values:

```bash
ros2 topic pub /vo_uncertainty std_msgs/Float32 "data: 0.1" -1
ros2 topic pub /slam_quality std_msgs/Float32 "data: 0.9" -1
ros2 topic pub /drift_norm std_msgs/Float32 "data: 0.2" -1
ros2 topic pub /sensor_noise std_msgs/Float32 "data: 0.3" -1
```

### 2) Observe outputs

```bash
ros2 topic echo /robot_confidence
ros2 topic echo /navigation_mode
ros2 topic echo /cmd_vel_limit
```

With default parameters, this should yield:

* High confidence (≈ 0.8+)
* Mode: `AGGRESSIVE`
* Velocity limit: ≈ 0.8 m/s

---

## Evaluation Plan

### Experimental Conditions

Compare:

1. **Baseline navigation**

   * No confidence estimation; fixed navigation parameters.

2. **Confidence-aware navigation (this system)**

   * Estimator enabled.
   * Behaviour adapted via `/navigation_mode` (velocity limit baseline).

### Suggested Metrics

* **Collision Rate**: collisions per episode/trajectory
* **Mission Success Rate**: percentage of trials reaching goal
* **Recovery Success**: probability of safe adaptation under degradation
* **Safety Under Degraded Perception**:

  * Increase VO uncertainty
  * Degrade SLAM tracking quality
  * Inject sensor noise

### Ablation & Sensitivity

* Different weights (w_i)
* Different thresholds (`th_aggressive`, `th_conservative`)
* Delay between degradation and mode adaptation

---

## Extensions & Future Work

* **Learning-based Confidence Estimation**
  Train a small model to predict (C(t)) from richer features (covariances, tracking flags, loop closures).

* **Calibration Analysis**
  Evaluate confidence vs. failure probability (reliability diagrams, Brier score).

* **Task-Aware Confidence**
  Extend from generic navigation confidence to task-specific confidence.

* **Human-in-the-Loop**
  Use confidence to trigger operator assistance or supervision.

---

## Repository Structure

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

## Author

**Panagiota Grosdouli**

This repository reflects my experimental work on confidence-aware robotics and integrating uncertainty/reliability into ROS 2 navigation and SLAM systems.

---

## License

Released under the **MIT License**.
Free to use, modify, and extend for research or educational purposes with appropriate attribution.
