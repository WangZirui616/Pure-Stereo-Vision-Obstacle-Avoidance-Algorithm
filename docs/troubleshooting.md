# Troubleshooting

This document summarizes the most common issues encountered during the development of the RealSense-based visual obstacle avoidance system and provides practical solutions based on our development experience.

---

# Quick Troubleshooting Workflow

```text
System Startup
      │
      ▼
Check whether OpenClaw is installed
      │
      ├── If installed → Ensure the OpenClaw gateway is disabled
      │
      ▼
Network connected?
      │
      ├── No → Check Wi-Fi antenna and network configuration
      │
      ▼
Select Ubuntu / ROS version
      │
      ▼
Jackal chassis connected?
      │
      ▼
RealSense detected?
      │
      ▼
Depth image available?
      │
      ▼
YOLO node running?
      │
      ▼
Robot moving?
      │
      ▼
Algorithm tuning
```

---

# 1. OpenClaw Gateway Check (Recommended)

## Background

OpenClaw is **not required** for this project.

However, some Jetson development boards are shipped with OpenClaw pre-installed, and its gateway service may be enabled by default. If left enabled, the device may become accessible from the public network after connecting to the Internet.

## Recommendation

Before connecting the Jetson to the Internet:

* Check whether OpenClaw is installed.
* If it is installed, verify that the OpenClaw gateway has been disabled unless remote access is intentionally required.

This is a security recommendation and is independent of this project.

---

# 2. Network Connection Failure

## Symptoms

* Unable to access the Internet.
* Unable to install ROS packages.
* Unable to clone GitHub repositories.

## Possible Causes

* Wi-Fi antenna is not installed correctly.
* Incorrect network configuration.
* Network authentication or security restrictions.

## Solutions

Check the following items in order:

1. Verify that the Wi-Fi antenna is securely connected.
2. Reconnect to the wireless network.
3. Verify Internet connectivity using another device.

If the device remains unable to access the network because of local authentication or security policies, reinstalling Ubuntu and reconfiguring the network may restore connectivity.

---

# 3. Choosing the Correct Operating System

The selected Ubuntu version determines which ROS version can be installed.

| Ubuntu Version          | ROS Version       | Jackal Firmware           |
| ----------------------- | ----------------- | ------------------------- |
| Ubuntu 20.04 or earlier | ROS Noetic (ROS1) | Original firmware         |
| Ubuntu 22.04            | ROS 2 Humble      | Firmware upgrade required |

If maintaining the original Jackal firmware is preferred, Ubuntu 20.04 with ROS1 is recommended.

Ubuntu 22.04 officially supports ROS2, which requires updating the Jackal firmware.

Running Ubuntu 20.04 inside a virtual machine is theoretically possible but was not evaluated in this project.

---

# 4. Jackal Chassis Cannot Be Controlled

## Symptoms

* Velocity commands are published, but the robot does not move.
* The controller appears to be running normally.

---

## Quick Diagnosis

| Symptom                                            | Possible Cause                                      | Solution                                                                |
| -------------------------------------------------- | --------------------------------------------------- | ----------------------------------------------------------------------- |
| `ros2 topic info /cmd_vel` shows **0 subscribers** | No controller receives velocity commands.           | Check `twist_mux` configuration and topic remapping.                    |
| Subscribers exist but the robot does not move      | MCU firmware is incompatible with ROS2.             | Update the firmware.                                                    |
| `/j100_0000/platform/mcu/status` has no output     | Hardware interface cannot communicate with the MCU. | Check USB connection, MCU Disconnect button and firmware compatibility. |

---

## Step 1. Verify Velocity Topic

Check whether the controller subscribes to the velocity command.

```bash
ros2 topic info /cmd_vel
```

If no subscriber exists:

* Verify that `twist_mux` is running.
* Verify that the topic name matches the controller configuration.
* Check whether `cmd_vel` or `platform/cmd_vel_unstamped` is being used.

---

## Step 2. Check MCU Communication

Run:

```bash
ros2 topic echo /j100_0000/platform/mcu/status --once
```

If no data is received:

The hardware interface is loaded but communication with the MCU has failed.

This usually indicates either:

* Firmware incompatibility.
* USB communication failure.

---

## Step 3. Verify MCU Communication

If the velocity topic is correctly subscribed but the robot still does not move, verify that the ROS driver can communicate with the MCU.

Run:

```bash
ros2 topic echo /j100_0000/platform/mcu/status --once
```

If no data is received, communication between the ROS driver and the MCU has not been established.

Possible causes include:

Incompatible MCU firmware
USB communication failure
MCU not initialized correctly

If the USB connection is confirmed to be normal, updating the MCU firmware is recommended.

---

## Step 4. Update MCU Firmware

If the MCU cannot communicate with ROS2, update the firmware.

Official firmware update guide:

https://docs.clearpathrobotics.com/docs/ros2humble/ros/installation/robot/

---

## Step 5. Verify Firmware Communication

If the robot still does not respond after the firmware update, verify that the MCU status topic is available:

```bash
ros2 topic echo /j100_0000/platform/mcu/status --once
```

If no data is published, the firmware update may not have completed successfully, or communication with the MCU has not been established. In this case, repeat the firmware update procedure and restart the Jackal.

---

# 5. Jackal Cannot Stay Powered On

## Symptoms

* The robot powers on briefly and then shuts down.
* LEDs flash for several seconds before turning off.

## Possible Cause

Power management instability after updating the firmware.

## Solution

During our development, repeatedly reflashing the firmware followed by a complete reboot temporarily restored stable operation.

Although this workaround improved stability, the preferred solution is to retain the original firmware whenever ROS1 is sufficient for the project.

---

# 6. RealSense Camera Cannot Be Detected

## Symptoms

The RealSense camera is not detected.

The following command returns no connected devices:

```bash
rs-enumerate-devices
```

Possible Cause

The camera is connected through a USB 2.0 port instead of a USB 3.0 port.

Solution

During our development, this issue was caused by using a USB 2.0 connection.

Reconnect the RealSense camera to a USB 3.0 port, then verify that it is detected:

```bash

rs-enumerate-devices
```

If the camera is detected successfully, launch:

```bash
realsense-viewer
```

to confirm that both the color and depth streams are available.

---

# 7. No Depth Image Received

## Symptoms

```
Waiting for depth image...
```

or

```
No fresh depth image.
```

## Solutions

Verify available topics:

```bash
ros2 topic list
```

Check the publishing rate:

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

A frame rate close to **30 Hz** is recommended.

---

# 8. YOLO Node Cannot Start

## Symptoms

* YOLO node exits immediately after startup.
* No detection messages are published.
* Obstacles are ignored.

## Possible Causes

* ONNX Runtime version is incompatible.
* Incorrect ONNX model.
* Incorrect image topic.

## Solutions

Verify that the image topic and model path are correct.

If the node crashes during startup, check the ONNX Runtime version.

During our development, this issue was resolved by downgrading ONNX Runtime to **version 1.16.3**, which restored compatibility with the exported ONNX model.

Verify the detection topic:

```bash
ros2 topic echo /yolo26/detections
```

---

# 9. Navigation Performance Issues

## Typical Symptoms

* Robot oscillates left and right.
* Robot collides with obstacle bases.
* Robot becomes trapped between multiple obstacles.

## Recommended Improvements

* Increase `early_avoid_distance`.
* Increase `safe_distance`.
* Increase `robot_width_ratio`.
* Introduce steering hysteresis.
* Add direction memory.
* Prefer the nearest feasible gap instead of always selecting the largest gap.
* Smooth steering commands over consecutive frames.

---

# General Debugging Checklist

Before running the complete system, verify the following:

| Item                                    | Status |
| --------------------------------------- | :----: |
| OpenClaw gateway checked (if installed) |    □   |
| Network connection available            |    □   |
| Correct Ubuntu / ROS version selected   |    □   |
| Jackal firmware compatible              |    □   |
| RealSense camera detected               |    □   |
| Depth images published                  |    □   |
| YOLO node running normally              |    □   |
| Velocity commands published             |    □   |
| MCU communication established           |    □   |
| Controller active                       |    □   |
| Emergency stop released                 |    □   |

Following this checklist resolved the majority of issues encountered during the development of this project.
