# Installation

This document describes how to install and configure the software environment for the RealSense-based visual obstacle avoidance project on the Clearpath Jackal robot.

---

## 1. System Requirements

Recommended platform:

- NVIDIA Jetson Orin NX
- Ubuntu 22.04
- ROS 2 Humble
- Intel RealSense D435i
- Clearpath Jackal UGV
- Python 3.10

---

## 2. Install ROS 2 Humble

Source ROS 2 before building or running the project:

```bash
source /opt/ros/humble/setup.bash
```

To avoid sourcing ROS 2 manually every time, add it to your `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Create a ROS 2 Workspace

Create a ROS 2 workspace if you do not already have one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone this repository into the `src` directory:

```bash
git clone https://github.com/WangZirui616/Pure-Stereo-Vision-Obstacle-Avoidance-Algorithm.git
```

Return to the workspace root:

```bash
cd ~/ros2_ws
```

Optionally verify that the ROS package exists:

```bash
find ~/ros2_ws/src -name package.xml
```

You should see a package named:

```text
realsense_depth_avoidance
```

---

## 4. Install Dependencies

Update the package index:

```bash
sudo apt update
```

Install common development tools:

```bash
sudo apt install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions
```

Install ROS image-processing dependencies:

```bash
sudo apt install -y \
    ros-humble-cv-bridge \
    python3-opencv \
    python3-numpy
```

Install the Intel RealSense ROS 2 driver:

```bash
sudo apt install -y ros-humble-realsense2-camera
```

If you plan to use ONNX Runtime or TensorRT for YOLO inference, install the corresponding runtime according to your Jetson environment.

> **Note**
>
> `cv_bridge` should be installed from the ROS apt package instead of using `pip`.

---

## 5. Build the Workspace

Build the entire workspace:

```bash
cd ~/ros2_ws

colcon build --symlink-install

source install/setup.bash
```

Or build only the obstacle avoidance package:

```bash
cd ~/ros2_ws

colcon build \
    --packages-select realsense_depth_avoidance \
    --symlink-install

source install/setup.bash
```

---

## 6. Verify Package Installation

Verify that ROS 2 can find the package:

```bash
ros2 pkg list | grep realsense_depth_avoidance
```

Expected output:

```text
realsense_depth_avoidance
```

Verify that the executable has been installed:

```bash
ros2 pkg executables realsense_depth_avoidance
```

Expected output:

```text
realsense_depth_avoidance depth_gap_avoidance
```

---

## 7. Verify the RealSense Camera

First, check that the camera is detected:

```bash
rs-enumerate-devices
```

Launch the RealSense ROS 2 driver:

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

Verify the depth image topic:

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

Verify the RGB image topic:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

Typical frame rates should be approximately **30 Hz**.

---

## 8. Verify Jackal Communication

List the available velocity command topics:

```bash
ros2 topic list | grep cmd_vel
```

For this project, the primary command topic is typically:

```text
/j100_0000/platform/cmd_vel_unstamped
```

Verify that odometry is available:

```bash
ros2 topic echo /j100_0000/platform/odom/filtered
```

If odometry messages are continuously published, the robot base is communicating correctly.

---

## 9. Next Step

After completing the installation and verification steps, proceed to:

- `docs/run_commands.md` — Launch commands for the camera, YOLO detector, and obstacle avoidance node.
- `docs/algorithm.md` — Overview of the obstacle avoidance algorithm.
- `docs/troubleshooting.md` — Common issues and solutions.
