# Run Commands

This document describes the complete workflow for launching the visual obstacle avoidance system, including building the package, starting the RealSense camera, launching the YOLO detector, running the obstacle avoidance algorithm, and verifying that all components are operating correctly.

---

# 1. Prerequisites

Before running the project, ensure that:

- ROS 2 Humble is installed.
- The workspace has been built successfully.
- Intel RealSense D435i is connected via USB 3.0.
- Jackal robot is powered on.
- The ROS 2 environment has been sourced.

---

# 2. Source the ROS 2 Workspace

Open a terminal and source the environment:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

This step must be performed before running any ROS 2 node.

---

# 3. Build the Package

Rebuild the package after modifying the source code.

```bash
cd ~/ros2_ws

colcon build \
    --packages-select realsense_depth_avoidance \
    --symlink-install

source install/setup.bash
```

Expected output:

```
Finished <<< realsense_depth_avoidance
Summary: 1 package finished
```

---

# 4. Launch the RealSense Camera

Start the Intel RealSense D435i driver.

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

The camera publishes:

- RGB images
- Aligned depth images
- IMU data
- Camera calibration information

---

# 5. Verify Camera Topics

List all camera topics:

```bash
ros2 topic list | grep camera
```

Check the depth image frequency:

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

Expected frequency:

```
≈30 Hz
```

Check the RGB image frequency:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

---

# 6. Launch the YOLO Detection Node

If obstacle classification is enabled, start the YOLO ONNX detector.

```bash
ros2 run jackal_cone_nav yolo_onnx_node \
    --ros-args \
    -p image_topic:=/camera/camera/color/image_raw \
    -p model_path:=/home/nx2026/yolo_model.onnx \
    -p detections_topic:=/yolo26/detections \
    -p confidence_threshold:=0.35
```

Verify the detection output:

```bash
ros2 topic echo /yolo26/detections
```

The detector should continuously publish obstacle detections.

---

# 7. Launch the Obstacle Avoidance Node

Run the depth-gap navigation algorithm.

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
    -p safe_distance:=1.05 \
    -p front_width_ratio:=0.58 \
    -p robot_width_ratio:=0.28 \
    -p min_gap_width_ratio:=0.20 \
    -p prefer_nearest_gap:=true \
    -p nearest_gap_offset_limit:=0.55
```

The robot should:

- Move toward the predefined goal.
- Detect free-space gaps.
- Avoid nearby obstacles.
- Use YOLO detections to increase the avoidance margin around recognized obstacles.

---

# 8. Verify Motion Commands

Check whether the navigation node is publishing velocity commands.

```bash
ros2 topic echo /j100_0000/platform/cmd_vel_unstamped
```

Typical output:

```text
linear:
  x: 0.28

angular:
  z: 0.12
```

If `linear.x` and `angular.z` remain zero, verify that:

- the depth image is being received,
- the camera is running correctly,
- obstacle avoidance has not triggered an emergency stop.

---

# 9. Verify Robot Odometry

Monitor the robot's estimated position.

```bash
ros2 topic echo /j100_0000/platform/odom/filtered
```

This topic is useful for verifying that the robot is moving and for debugging navigation behavior.

---

# 10. Check Robot Diagnostics

Monitor the Jackal platform status.

```bash
ros2 topic echo /j100_0000/diagnostics
```

To display only warnings and errors:

```bash
ros2 topic echo /j100_0000/diagnostics \
| grep -Ei "error|warn|stale|battery|motor|driver|mcu|estop"
```

---

# 11. Emergency Stop

Immediately stop the robot by publishing zero velocity.

```bash
ros2 topic pub --once \
/j100_0000/platform/cmd_vel_unstamped \
geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

This command is useful whenever unexpected robot behavior occurs.

---

# 12. Direct Motion Test

Before testing the obstacle avoidance algorithm, verify that the Jackal base accepts velocity commands.

Move forward slowly:

```bash
ros2 topic pub --rate 10 \
/j100_0000/platform/cmd_vel_unstamped \
geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Rotate in place:

```bash
ros2 topic pub --rate 10 \
/j100_0000/platform/cmd_vel_unstamped \
geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.3}}"
```

Press **Ctrl+C** to stop publishing, then send a zero-velocity command.

---

# 13. Recommended Terminal Layout

It is recommended to use four separate terminals during experiments.

| Terminal | Command |
|----------|---------|
| Terminal 1 | Launch RealSense camera |
| Terminal 2 | Launch YOLO detector |
| Terminal 3 | Run obstacle avoidance |
| Terminal 4 | Monitor topics and diagnostics |

---

# 14. Useful Debug Commands

List active nodes:

```bash
ros2 node list
```

List active topics:

```bash
ros2 topic list
```

List active services:

```bash
ros2 service list
```

Check topic information:

```bash
ros2 topic info /camera/camera/aligned_depth_to_color/image_raw
```

Check topic publishing frequency:

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

---

# 15. Typical Startup Sequence

The recommended startup order is:

1. Build the workspace (if necessary)
2. Source the ROS 2 environment
3. Launch the RealSense camera
4. Verify camera topics
5. Launch the YOLO detector
6. Verify detection output
7. Launch the obstacle avoidance node
8. Monitor diagnostics during robot operation