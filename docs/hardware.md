# Hardware

This document describes the hardware platform used for the RealSense-based visual obstacle avoidance project.

---

# 1. Hardware Overview

The system consists of four main components:

* Clearpath Jackal UGV
* Intel RealSense D435i depth camera
* NVIDIA Jetson Orin NX onboard computer
* USB 3.0 connection between the camera and Jetson

The RealSense camera captures synchronized RGB and depth images. The Jetson processes the sensor data, runs the obstacle avoidance algorithm, and publishes velocity commands to the Jackal chassis through ROS 2.

---

# 2. Hardware Components

| Component        | Model                 | Purpose                                    |
| ---------------- | --------------------- | ------------------------------------------ |
| Mobile Robot     | Clearpath Jackal UGV  | Mobile platform                            |
| Depth Camera     | Intel RealSense D435i | RGB and depth perception                   |
| Onboard Computer | NVIDIA Jetson Orin NX | Run ROS 2 and obstacle avoidance algorithm |
| Operating System | Ubuntu 22.04          | Software platform                          |
| ROS Version      | ROS 2 Humble          | Robot middleware                           |

---

# 3. Hardware Connection

The hardware is connected as follows:

```text
                 +----------------------+
                 | Intel RealSense D435i|
                 +----------+-----------+
                            |
                         USB 3.0
                            |
                 +----------v-----------+
                 | NVIDIA Jetson Orin NX|
                 +----------+-----------+
                            |
                  ROS 2 Velocity Commands
                            |
                 +----------v-----------+
                 |   Clearpath Jackal   |
                 +----------------------+
```

The Jetson receives RGB and depth images from the RealSense camera and computes obstacle avoidance commands, which are sent to the Jackal through ROS 2.

---

# 4. Camera Configuration

The project uses the Intel RealSense D435i with the following recommended configuration.

| Parameter        | Value     |
| ---------------- | --------- |
| Color Resolution | 640 × 480 |
| Depth Resolution | 640 × 480 |
| Frame Rate       | 30 FPS    |
| Depth Alignment  | Enabled   |
| RGB Stream       | Enabled   |
| Depth Stream     | Enabled   |
| IMU              | Optional  |

The aligned depth image is used for obstacle detection and free-space estimation.

---

# 5. Camera Mounting

For reliable obstacle detection:

* Mount the camera at the front of the robot.
* Keep the camera approximately horizontal.
* Ensure that the ground immediately in front of the robot is visible.
* Avoid excessive vibration during operation.
* Make sure the camera has an unobstructed forward field of view.

---

# 6. USB Connection

The Intel RealSense D435i must be connected using a USB 3.0 port.

Recommended connection:

```text
RealSense D435i
      │
 USB 3.0 Cable
      │
Jetson Orin NX
```

Using a USB 2.0 connection may result in:

* Camera not being detected
* Reduced frame rate
* Missing depth stream

If the camera cannot be detected, first verify that it is connected to a USB 3.0 port.

---

# 7. Recommended System Requirements

| Item    | Recommendation        |
| ------- | --------------------- |
| CPU     | NVIDIA Jetson Orin NX |
| Memory  | ≥ 8 GB                |
| Storage | ≥ 20 GB available     |
| Ubuntu  | 22.04                 |
| ROS     | Humble                |
| Python  | 3.10                  |

---

# 8. Notes

* Always connect the RealSense camera using USB 3.0.
* Secure the camera firmly to avoid motion blur caused by vibration.
* Ensure sufficient lighting for stable RGB image quality.
* Keep the camera lens clean to maintain reliable depth measurements.
* Verify that the camera is detected before launching the obstacle avoidance node.
