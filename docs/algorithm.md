# Algorithm

This project implements a RealSense-based visual obstacle avoidance algorithm for the Clearpath Jackal unmanned ground vehicle. The navigation system relies primarily on depth perception from an Intel RealSense D435i stereo camera, while optional YOLO-based object detection is incorporated to improve obstacle recognition in challenging scenarios.

Unlike conventional reactive navigation methods such as the Artificial Potential Field (APF), the proposed approach adopts a **gap-based navigation strategy** that searches for traversable free space in the depth image and guides the robot toward a predefined goal while avoiding obstacles smoothly.

---

# 1. Navigation Objective

The objective of the system is to autonomously navigate from the starting position to a predefined exit while avoiding static obstacles such as traffic cones and boundary boards.

At the current stage, the destination is represented as a predefined waypoint located approximately:

```text
6.5 meters in front of the starting position
```

Although an exit recognition module has been developed using depth information and color segmentation, the detected exit position is not yet sufficiently stable for real-time trajectory replanning. Therefore, the current navigation framework primarily follows the predefined goal direction.

---

# 2. System Overview

The complete navigation pipeline is illustrated below.

```text
                 RealSense D435i
            (Depth + RGB Images)
                     │
                     ▼
          Free-Space Extraction
                     │
                     ▼
              Gap Detection
                     │
                     ▼
          Candidate Gap Evaluation
                     │
                     ▼
        Direction Memory Filtering
                     │
                     ▼
          Velocity Command Generation
                     │
                     ▼
               Jackal Motion Control
```

When enabled, YOLO detection provides semantic information about obstacles and assists the avoidance module in generating safer steering commands.

---

# 3. Sensor Inputs

## 3.1 Depth Image

The aligned depth image provided by the Intel RealSense D435i serves as the primary perception source.

ROS topic:

```text
/camera/camera/aligned_depth_to_color/image_raw
```

Depth information is used to:

* Detect nearby obstacles
* Estimate traversable free space
* Find candidate gaps
* Determine obstacle distance
* Generate obstacle avoidance commands

---

## 3.2 RGB Image and YOLO Detection

An optional YOLO detector recognizes obstacles such as traffic cones.

ROS topic:

```text
/yolo26/detections
```

This semantic information complements the depth image because the lower base of an obstacle may occasionally fall outside the effective depth sensing region. Once an object is classified as an obstacle, the controller increases the avoidance response to reduce the risk of collision.

---

# 4. Free-Space Extraction

The aligned depth image is divided into multiple horizontal sampling regions. Each region is evaluated to determine whether sufficient clearance exists for safe traversal.

A pixel column is considered traversable when

```text
depth > safe_distance
```

Typical values are

```text
safe_distance = 1.0–1.5 m
```

Columns with insufficient depth are classified as occupied.

The resulting binary free-space map becomes the input for gap detection.

---

# 5. Gap Detection

Adjacent free columns are grouped into connected traversable regions referred to as **gaps**.

A candidate gap must satisfy several geometric constraints:

* Sufficient width for the robot
* Adequate average depth
* Safe clearance from nearby obstacles

A minimum allowable gap width is defined as

```text
min_gap_width_ratio = 0.20
```

meaning that the gap should occupy at least **20% of the image width** before being considered traversable.

This filtering prevents the robot from entering narrow passages that have a high probability of collision.

---

# 6. Gap Evaluation and Selection

Instead of always selecting the largest visible gap, the algorithm evaluates all candidate gaps using a weighted scoring strategy.

The evaluation considers:

* Gap width
* Average gap depth
* Distance from the image center
* Alignment with the navigation target
* Previous steering direction

The preferred gap is therefore

```text
A sufficiently wide and safe gap that best aligns with the desired travel direction.
```

rather than simply the widest gap.

This strategy reduces unnecessary detours and improves navigation stability in cluttered environments.

---

# 7. Direction Memory

Reactive obstacle avoidance often suffers from rapid left-right oscillations because the camera view changes continuously during turning.

To alleviate this issue, the controller maintains a short-term steering memory.

Example:

```text
Turn Left
     │
     ▼
Another obstacle appears
     │
     ▼
Avoid immediate large right turn
     │
     ▼
Continue following the previous safe direction
```

The steering direction is only changed when the previously selected direction is no longer safe.

Direction memory effectively suppresses oscillatory behavior and produces smoother trajectories.

---

# 8. Distance-Based Obstacle Avoidance

The avoidance behavior changes according to the measured distance to the nearest obstacle.

## 8.1 Normal Navigation

When the path ahead is clear, the robot moves directly toward the target at normal speed.

---

## 8.2 Early Avoidance

If an obstacle enters the early warning region, steering begins before the robot gets too close.

Example:

```text
early_avoid_distance = 1.35 m
```

---

## 8.3 Hard Avoidance

When the obstacle becomes closer, the robot decreases forward speed while increasing angular velocity.

Example:

```text
hard_avoid_distance = 0.90 m
```

---

## 8.4 Emergency Stop

If the obstacle approaches the safety limit, the robot immediately stops or performs an in-place rotation.

Example:

```text
emergency_stop_distance = 0.32 m
```

This hierarchical strategy enables smooth transitions between normal navigation and emergency collision avoidance.

---

# 9. Velocity Control

The navigation controller publishes velocity commands through

```text
/j100_0000/platform/cmd_vel_unstamped
```

Each command contains:

* Linear velocity (`linear.x`)
* Angular velocity (`angular.z`)

Typical parameters are

```text
linear_speed = 0.28 m/s
avoid_speed  = 0.14 m/s
creep_speed  = 0.08 m/s
```

Forward velocity is automatically reduced during obstacle avoidance while steering velocity increases proportionally to obstacle proximity.

---

# 10. Exit Detection (Additional Feature)

An additional exit recognition module has been implemented as an extension to the navigation system.

The method combines

* Depth information
* Color segmentation
* Geometric constraints of the exit structure

The workflow is

```text
Detect Side Boards
        │
        ▼
Estimate Opening Between Boards
        │
        ▼
Verify Opening Using Depth
        │
        ▼
Estimate Exit Direction
```

Although the exit can be detected reliably in many situations, integrating the detected exit direction into real-time navigation is still under development.

Therefore, the current system primarily relies on the predefined goal position.

---

# 11. Comparison with Artificial Potential Field

The Artificial Potential Field (APF) method models the target as an attractive force while representing obstacles as repulsive forces.

Although computationally efficient, APF exhibited several undesirable behaviors during our experiments:

* Frequent left-right oscillations
* Repeated in-place rotations
* Local minima in cluttered environments
* Unstable steering caused by continuously changing camera viewpoints
* Attraction back toward previously avoided obstacles

To overcome these limitations, the proposed system adopts a gap-based navigation strategy that explicitly searches for traversable free space instead of computing virtual force fields.

---

# 12. Current Limitations and Future Improvements

The current implementation still has several limitations:

* Real-time navigation using detected exits is not yet sufficiently robust.
* The field of view of the depth camera is inherently limited.
* The lower portions of obstacles may occasionally fall outside the effective sensing region.
* YOLO inference introduces additional computational latency on embedded hardware.
* Extremely narrow or densely cluttered environments may still produce suboptimal steering decisions.

Future work will focus on:

* Integrating exit detection directly into online path planning.
* Improving navigation robustness in highly cluttered environments.
* Optimizing obstacle avoidance using more adaptive gap evaluation strategies.
* Reducing perception latency while maintaining real-time performance.
* Extending the framework to dynamic obstacle avoidance.

