# Pure-Stereo-Vision-Obstacle-Avoidance-Algorithm
A pure stereo vision obstacle avoidance framework for mobile robots based on Intel RealSense D435i, YOLO2026, TensorRT, and ROS 2 Humble.

---

## Overview

This project implements a real-time obstacle avoidance system for the Clearpath Jackal robot using only stereo vision sensing.

The system combines:

* Intel RealSense D435i stereo depth camera
* YOLO2026 object detection
* TensorRT accelerated inference
* Gap Navigation obstacle avoidance
* ROS 2 Humble

The robot is capable of:

* Detecting obstacles using stereo depth perception
* Identifying obstacle categories with YOLO2026
* Selecting traversable gaps from depth images
* Avoiding obstacles in real time
* Reaching the target exit area without LiDAR

---

## Hardware Platform

| Component  | Model                 |
| ---------- | --------------------- |
| Robot      | Clearpath Jackal      |
| Camera     | Intel RealSense D435i |
| Compute    | NVIDIA Jetson Orin NX |
| OS         | Ubuntu 22.04          |
| Middleware | ROS 2 Humble          |

---

## Software Stack

* ROS 2 Humble
* RealSense ROS Driver
* OpenCV
* NumPy
* YOLO2026
* TensorRT
* Python 3.10

---

## System Architecture

```text
RealSense D435i
        │
        ▼
Depth Image + RGB Image
        │
        ├──────────────► YOLO2026 + TensorRT
        │                        │
        ▼                        ▼
Depth-based Gap Detection    Obstacle Detection
        │                        │
        └──────────┬─────────────┘
                   ▼
         Gap Selection Module
                   ▼
         Motion Controller
                   ▼
             Jackal Robot
```

## Algorithm Description

### Depth-based Gap Navigation

The obstacle avoidance algorithm consists of the following steps:

1. Acquire aligned depth images from RealSense.
2. Extract traversable regions based on depth values.
3. Identify continuous free-space gaps.
4. Evaluate gap width, depth, and heading direction.
5. Select the optimal gap.
6. Generate velocity commands for the robot.

### YOLO-assisted Obstacle Avoidance

Although depth images provide distance information, obstacle bases are sometimes outside the camera field of view.

YOLO2026 is therefore used to:

* Detect obstacle categories
* Increase avoidance confidence
* Trigger earlier avoidance maneuvers
* Prevent collisions with obstacle bases

### TensorRT Optimization

To achieve real-time performance on Jetson Orin NX, the YOLO2026 model is accelerated using TensorRT.

Benefits include:

* Lower inference latency
* Higher FPS
* Reduced CPU utilization
* Stable deployment on embedded platforms

---

## Installation

### Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/pure-stereo-vision-obstacle-avoidance.git

cd pure-stereo-vision-obstacle-avoidance
```

### Build Workspace

```bash
cd ~/ros2_ws

colcon build --symlink-install

source install/setup.bash
```

---

## Running the System

### Step 1. Launch RealSense

```bash
ros2 launch realsense2_camera rs_launch.py \
enable_color:=true \
enable_depth:=true \
align_depth.enable:=true \
enable_gyro:=true \
enable_accel:=true \
unite_imu_method:=2 \
pointcloud.enable:=false \
enable_sync:=true \
depth_module.depth_profile:=640x480x30 \
rgb_camera.color_profile:=640x480x30
```

### Step 2. Launch YOLO2026 Node

```bash
ros2 run jackal_cone_nav yolo_onnx_node \
  --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p model_path:=/home/nx2026/yolo_model.onnx \
  -p detections_topic:=/yolo26/detections \
  -p confidence_threshold:=0.35
```

TensorRT optimized model is recommended for deployment on Jetson Orin NX.

### Step 3. Launch Obstacle Avoidance Node

```bash
ros2 run realsense_depth_avoidance depth_gap_avoidance \
  --ros-args \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p yolo_topic:=/yolo26/detections \
  -p cmd_vel_topic:=/j100_0000/platform/cmd_vel_unstamped \
  -p yolo_enable:=true \
  -p target_distance:=6.5 \
  -p linear_speed:=0.28 \
  -p avoid_speed:=0.14 \
  -p creep_speed:=0.08 \
  -p early_avoid_distance:=1.35 \
  -p hard_avoid_distance:=0.90 \
  -p emergency_stop_distance:=0.32 \
  -p safe_distance:=1.05
```

---

## Demonstration Video

### Obstacle Avoidance Demo

After uploading a video to GitHub, place it under:

```text
media/demo.mp4
```

Then add:

```markdown
## Demonstration Video

https://github.com/YOUR_USERNAME/pure-stereo-vision-obstacle-avoidance/blob/main/media/demo.mp4
```

Alternatively, upload the video to YouTube and embed:

```markdown
## Demonstration Video

[Watch Demo Video](https://youtu.be/xxxxxxxx)
```

GitHub renders YouTube links much more cleanly than local video files.

---

## Repository Structure

```text
pure-stereo-vision-obstacle-avoidance
│
├── README.md
├── docs
│   ├── algorithm.md
│   ├── installation.md
│   ├── run_commands.md
│   ├── troubleshooting.md
│   └── hardware.md
│
├── src
├── launch
├── config
├── media
│   ├── demo.mp4
│   └── screenshots
│
└── images
```

---

## Troubleshooting

See:

```text
docs/troubleshooting.md
```

Included issues:

* Network connection failure
* ROS1 / ROS2 compatibility
* Jackal firmware update
* RealSense driver issues
* Vehicle cannot move
* Vehicle spins in place
* Consecutive obstacle avoidance failure
* TensorRT deployment issues

---

## Other Features

### Exit Recognition

In addition to obstacle avoidance, an exit recognition module has also been developed.

The current approach combines:

- Stereo depth information from the RealSense D435i
- Color detection of the exit side boards
- Maximum-depth region analysis

The exit direction is estimated by identifying the open area located between the two colored boards and selecting the corresponding depth gap.

### Current Limitation

Although exit recognition has been successfully implemented, directly updating the navigation target using real-time exit observations is not yet sufficiently robust.

In cluttered environments, temporary occlusions and viewpoint changes may cause fluctuations in the estimated exit direction, which can negatively affect navigation stability.

### Current Navigation Strategy

To ensure reliable performance during experiments, the current system uses a predefined exit position.

The robot:

1. Starts with a known exit location.
2. Uses depth-based gap navigation for obstacle avoidance.
3. Continuously moves toward the predefined goal position.
4. Reaches the exit after avoiding obstacles.

### Future Improvement

Future work will focus on integrating:

- Real-time exit recognition
- Exit direction tracking
- Dynamic goal updating
- Exit-guided navigation

This would allow the robot to navigate completely based on visual perception without relying on a predefined exit location.

---

## License

MIT License
