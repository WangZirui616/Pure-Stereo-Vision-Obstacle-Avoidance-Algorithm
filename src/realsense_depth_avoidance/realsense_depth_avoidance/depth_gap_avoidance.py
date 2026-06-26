#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


@dataclass
class Gap:
    left: int
    right: int
    center: int
    width: int
    offset: float
    depth: float
    score: float


@dataclass
class YoloObstacle:
    offset: float
    depth: float
    confidence: float
    stamp: float


class DepthYoloAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_gap_avoidance')

        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('yolo_topic', '/yolo26/detections')
        self.declare_parameter('cmd_vel_topic', '/j100_0000/platform/cmd_vel_unstamped')

        self.declare_parameter('roi_top_ratio', 0.40)
        self.declare_parameter('roi_bottom_ratio', 0.98)

        self.declare_parameter('min_valid_depth', 0.20)
        self.declare_parameter('max_valid_depth', 6.0)

        self.declare_parameter('emergency_stop_distance', 0.32)
        self.declare_parameter('hard_avoid_distance', 0.90)
        self.declare_parameter('early_avoid_distance', 1.35)
        self.declare_parameter('safe_distance', 1.05)

        self.declare_parameter('linear_speed', 0.28)
        self.declare_parameter('avoid_speed', 0.14)
        self.declare_parameter('creep_speed', 0.08)

        self.declare_parameter('max_turn_speed', 0.60)
        self.declare_parameter('min_avoid_turn_speed', 0.18)
        self.declare_parameter('mid_avoid_turn_speed', 0.40)
        self.declare_parameter('strong_avoid_turn_speed', 0.56)

        self.declare_parameter('mid_turn_distance', 0.50)
        self.declare_parameter('strong_turn_distance', 0.30)

        self.declare_parameter('early_turn_gain', 0.65)
        self.declare_parameter('hard_turn_gain', 0.88)
        self.declare_parameter('cruise_turn_gain', 0.04)

        self.declare_parameter('front_width_ratio', 0.58)
        self.declare_parameter('robot_width_ratio', 0.28)
        self.declare_parameter('min_gap_width_ratio', 0.20)

        self.declare_parameter('gap_width_weight', 4.0)
        self.declare_parameter('gap_depth_weight', 1.5)
        self.declare_parameter('center_weight', 1.00)
        self.declare_parameter('narrow_gap_penalty', 2.0)

        # 优先选择最近可通行 gap，避免追最大 gap 走到死角
        self.declare_parameter('prefer_nearest_gap', True)
        self.declare_parameter('nearest_gap_offset_limit', 0.55)
        self.declare_parameter('nearest_gap_min_depth', 1.10)
        self.declare_parameter('nearest_gap_min_width_ratio', 0.18)

        self.declare_parameter('direction_lock_time', 1.80)
        self.declare_parameter('angular_smooth_alpha', 0.82)
        self.declare_parameter('target_smooth_alpha', 0.82)

        # 禁止短时间反向避障，防止扫到刚绕过的障碍
        self.declare_parameter('reverse_forbid_time', 1.50)
        self.declare_parameter('reverse_forbid_distance', 0.55)
        self.declare_parameter('reverse_keep_offset', 0.32)

        self.declare_parameter('yolo_enable', True)
        self.declare_parameter('yolo_min_confidence', 0.35)
        self.declare_parameter('yolo_effect_distance', 1.80)
        self.declare_parameter('yolo_stop_distance', 0.75)
        self.declare_parameter('yolo_timeout', 0.45)

        self.declare_parameter('target_distance', 6.5)
        self.declare_parameter('control_rate_hz', 15.0)

        for name in [
            'depth_topic', 'yolo_topic', 'cmd_vel_topic',
            'roi_top_ratio', 'roi_bottom_ratio',
            'min_valid_depth', 'max_valid_depth',
            'emergency_stop_distance', 'hard_avoid_distance',
            'early_avoid_distance', 'safe_distance',
            'linear_speed', 'avoid_speed', 'creep_speed',
            'max_turn_speed',
            'min_avoid_turn_speed', 'mid_avoid_turn_speed',
            'strong_avoid_turn_speed', 'mid_turn_distance',
            'strong_turn_distance',
            'early_turn_gain', 'hard_turn_gain', 'cruise_turn_gain',
            'front_width_ratio', 'robot_width_ratio', 'min_gap_width_ratio',
            'gap_width_weight', 'gap_depth_weight', 'center_weight',
            'narrow_gap_penalty',
            'prefer_nearest_gap', 'nearest_gap_offset_limit',
            'nearest_gap_min_depth', 'nearest_gap_min_width_ratio',
            'direction_lock_time', 'angular_smooth_alpha', 'target_smooth_alpha',
            'reverse_forbid_time', 'reverse_forbid_distance', 'reverse_keep_offset',
            'yolo_enable', 'yolo_min_confidence', 'yolo_effect_distance',
            'yolo_stop_distance', 'yolo_timeout',
            'target_distance', 'control_rate_hz',
        ]:
            setattr(self, name, self.get_parameter(name).value)

        self.depth_topic = str(self.depth_topic)
        self.yolo_topic = str(self.yolo_topic)
        self.cmd_vel_topic = str(self.cmd_vel_topic)
        self.yolo_enable = bool(self.yolo_enable)

        self.bridge = CvBridge()
        self.last_depth: np.ndarray | None = None
        self.last_depth_time = 0.0
        self.closest_yolo_obstacle: YoloObstacle | None = None

        self.last_time = time.monotonic()
        self.travel_distance = 0.0
        self.finished = False

        self.locked_offset = 0.0
        self.lock_time = 0.0
        self.filtered_target_offset = 0.0
        self.last_angular = 0.0

        self.last_avoid_sign = 0.0
        self.last_avoid_time = 0.0
        self.last_avoid_distance = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        if self.yolo_enable:
            self.yolo_sub = self.create_subscription(
                Detection2DArray,
                self.yolo_topic,
                self.yolo_callback,
                10,
            )

        self.timer = self.create_timer(
            1.0 / max(float(self.control_rate_hz), 1.0),
            self.control_loop,
        )

        self.get_logger().info(
            'Depth-YOLO avoidance started: nearest-gap priority + reverse-forbid memory.'
        )

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert depth image: {exc}')
            return

        depth = np.asarray(depth)

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)

        depth[~np.isfinite(depth)] = 0.0
        self.last_depth = depth
        self.last_depth_time = time.monotonic()

    def control_loop(self) -> None:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if self.finished or self.last_depth is None:
            self.publish_stop()
            return

        if now - self.last_depth_time > 0.7:
            self.publish_stop()
            return

        best_gap, front_depth, obstacle_offset = self.find_largest_gap(self.last_depth)

        yolo = self.get_recent_yolo_obstacle(now)
        obstacle_depth = front_depth

        if yolo is not None and yolo.depth < obstacle_depth:
            obstacle_depth = yolo.depth
            obstacle_offset = yolo.offset

        avoiding = obstacle_depth < float(self.early_avoid_distance)
        reverse_blocked = False

        if avoiding:
            if best_gap is not None:
                new_offset = best_gap.offset
            else:
                new_offset = 0.55 if obstacle_offset < 0.0 else -0.55

            desired_sign = float(np.sign(new_offset)) if abs(new_offset) > 0.05 else 0.0

            if desired_sign != 0.0:
                reverse_time_ok = (now - self.last_avoid_time) > float(self.reverse_forbid_time)
                reverse_dist_ok = (
                    self.travel_distance - self.last_avoid_distance
                ) > float(self.reverse_forbid_distance)

                if (
                    self.last_avoid_sign != 0.0
                    and desired_sign == -self.last_avoid_sign
                    and not (reverse_time_ok and reverse_dist_ok)
                ):
                    reverse_blocked = True
                    keep_mag = max(abs(self.locked_offset), float(self.reverse_keep_offset))
                    new_offset = self.last_avoid_sign * keep_mag
                    desired_sign = self.last_avoid_sign

                if desired_sign != 0.0 and not reverse_blocked:
                    self.last_avoid_sign = desired_sign
                    self.last_avoid_time = now
                    self.last_avoid_distance = self.travel_distance

            if now - self.lock_time > float(self.direction_lock_time):
                self.locked_offset = new_offset
                self.lock_time = now
            else:
                if np.sign(new_offset) == np.sign(self.locked_offset):
                    self.locked_offset = 0.70 * self.locked_offset + 0.30 * new_offset
                elif reverse_blocked:
                    self.locked_offset = new_offset

            target_offset = self.locked_offset

        else:
            target_offset = 0.0
            self.locked_offset = 0.0

        self.filtered_target_offset = (
            float(self.target_smooth_alpha) * self.filtered_target_offset
            + (1.0 - float(self.target_smooth_alpha)) * target_offset
        )

        cmd = Twist()

        if obstacle_depth < float(self.emergency_stop_distance):
            turn_limit = self.dynamic_turn_limit(obstacle_depth)
            cmd.linear.x = 0.0
            cmd.angular.z = -np.sign(self.filtered_target_offset) * turn_limit

            if abs(cmd.angular.z) < 0.1:
                cmd.angular.z = turn_limit

        elif obstacle_depth < float(self.hard_avoid_distance):
            turn_limit = self.dynamic_turn_limit(obstacle_depth)
            cmd.linear.x = float(self.creep_speed)
            cmd.angular.z = float(np.clip(
                -float(self.hard_turn_gain) * self.filtered_target_offset,
                -turn_limit,
                turn_limit,
            ))

        elif obstacle_depth < float(self.early_avoid_distance):
            turn_limit = self.dynamic_turn_limit(obstacle_depth)
            cmd.linear.x = float(self.avoid_speed)
            cmd.angular.z = float(np.clip(
                -float(self.early_turn_gain) * self.filtered_target_offset,
                -turn_limit,
                turn_limit,
            ))

        else:
            cmd.linear.x = float(self.linear_speed)
            cmd.angular.z = -float(self.cruise_turn_gain) * self.filtered_target_offset

        if yolo is not None and yolo.depth < float(self.yolo_effect_distance):
            yolo_dir = -1.0 if yolo.offset > 0.0 else 1.0

            if self.last_avoid_sign != 0.0 and yolo_dir == -self.last_avoid_sign:
                reverse_time_ok = (now - self.last_avoid_time) > float(self.reverse_forbid_time)
                reverse_dist_ok = (
                    self.travel_distance - self.last_avoid_distance
                ) > float(self.reverse_forbid_distance)

                if reverse_time_ok and reverse_dist_ok:
                    cmd.angular.z += yolo_dir * 0.08
                else:
                    cmd.angular.z += self.last_avoid_sign * 0.04
            else:
                cmd.angular.z += yolo_dir * 0.08

            cmd.linear.x = min(cmd.linear.x, float(self.avoid_speed))

        if yolo is not None and yolo.depth < float(self.yolo_stop_distance):
            cmd.linear.x = min(cmd.linear.x, float(self.creep_speed))

        cmd.linear.x = float(np.clip(cmd.linear.x, 0.0, float(self.linear_speed)))
        cmd.angular.z = float(np.clip(
            cmd.angular.z,
            -float(self.max_turn_speed),
            float(self.max_turn_speed),
        ))

        cmd.angular.z = (
            float(self.angular_smooth_alpha) * self.last_angular
            + (1.0 - float(self.angular_smooth_alpha)) * cmd.angular.z
        )
        self.last_angular = cmd.angular.z

        self.travel_distance += max(cmd.linear.x, 0.0) * dt

        if self.travel_distance >= float(self.target_distance):
            self.finished = True
            self.publish_stop()
            return

        self.get_logger().info(
            f'v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}, '
            f'front={front_depth:.2f}, obs={obstacle_depth:.2f}, '
            f'gap_offset={best_gap.offset if best_gap else 99:.2f}, '
            f'gap_width={best_gap.width if best_gap else 0}, '
            f'turn_limit={self.dynamic_turn_limit(obstacle_depth):.2f}, '
            f'locked={self.locked_offset:.2f}, '
            f'last_sign={self.last_avoid_sign:.0f}, '
            f'rev_block={reverse_blocked}, '
            f'dist={self.travel_distance:.2f}/{float(self.target_distance):.2f}',
            throttle_duration_sec=0.5,
        )

        self.cmd_pub.publish(cmd)

    def dynamic_turn_limit(self, obstacle_depth: float) -> float:
        d = float(obstacle_depth)

        if d <= float(self.strong_turn_distance):
            return float(self.strong_avoid_turn_speed)

        if d <= float(self.mid_turn_distance):
            ratio = (
                float(self.mid_turn_distance) - d
            ) / max(
                float(self.mid_turn_distance) - float(self.strong_turn_distance),
                0.01,
            )
            ratio = float(np.clip(ratio, 0.0, 1.0))

            return float(self.mid_avoid_turn_speed) + (
                float(self.strong_avoid_turn_speed) - float(self.mid_avoid_turn_speed)
            ) * ratio

        if d <= float(self.early_avoid_distance):
            ratio = (
                float(self.early_avoid_distance) - d
            ) / max(
                float(self.early_avoid_distance) - float(self.mid_turn_distance),
                0.01,
            )
            ratio = float(np.clip(ratio, 0.0, 1.0))

            return float(self.min_avoid_turn_speed) + (
                float(self.mid_avoid_turn_speed) - float(self.min_avoid_turn_speed)
            ) * ratio

        return float(self.min_avoid_turn_speed)

    def find_largest_gap(self, depth: np.ndarray) -> tuple[Gap | None, float, float]:
        h, w = depth.shape[:2]

        y1 = int(np.clip(h * float(self.roi_top_ratio), 0, h - 1))
        y2 = int(np.clip(h * float(self.roi_bottom_ratio), y1 + 1, h))
        roi = depth[y1:y2, :]

        valid = (
            (roi >= float(self.min_valid_depth))
            & (roi <= float(self.max_valid_depth))
            & np.isfinite(roi)
        )

        col_depth = np.full(w, float(self.max_valid_depth), dtype=np.float32)

        for x in range(w):
            vals = roi[:, x][valid[:, x]]
            if vals.size > 4:
                col_depth[x] = float(np.percentile(vals, 10))
            else:
                col_depth[x] = float(self.max_valid_depth)

        kernel = max(5, int(w * 0.02))
        if kernel % 2 == 0:
            kernel += 1

        col_depth = np.convolve(col_depth, np.ones(kernel) / kernel, mode='same')

        front_l = int(w * (0.5 - float(self.front_width_ratio) * 0.5))
        front_r = int(w * (0.5 + float(self.front_width_ratio) * 0.5))
        front_depths = col_depth[front_l:front_r]

        front_depth = (
            float(np.percentile(front_depths, 5))
            if front_depths.size > 0
            else float(self.max_valid_depth)
        )

        close_cols = np.where(front_depths < float(self.early_avoid_distance))[0]

        if close_cols.size > 0:
            obstacle_offset = float(((front_l + np.mean(close_cols)) - w * 0.5) / (w * 0.5))
        else:
            obstacle_offset = 0.0

        free = col_depth >= float(self.safe_distance)

        inflate_px = max(10, int(w * float(self.robot_width_ratio) * 0.5))
        blocked = ~free

        padded = np.pad(blocked.astype(np.uint8), (inflate_px, inflate_px), mode='edge')
        inflated = np.zeros_like(blocked, dtype=bool)

        for i in range(w):
            inflated[i] = np.any(padded[i:i + 2 * inflate_px + 1] > 0)

        free = ~inflated

        min_gap_width = max(25, int(w * float(self.min_gap_width_ratio)))
        gaps: list[Gap] = []

        x = 0
        while x < w:
            while x < w and not free[x]:
                x += 1

            left = x

            while x < w and free[x]:
                x += 1

            right = x - 1
            width = right - left + 1

            if width >= min_gap_width:
                center = (left + right) // 2
                offset = float((center - w * 0.5) / (w * 0.5))
                gap_depth = float(np.percentile(col_depth[left:right + 1], 35))

                width_score = width / w
                depth_score = float(np.clip(gap_depth / float(self.max_valid_depth), 0.0, 1.0))
                center_score = 1.0 - abs(offset)
                narrow_penalty = max(0.0, 0.28 - width_score)

                score = (
                    float(self.gap_width_weight) * width_score
                    + float(self.gap_depth_weight) * depth_score
                    + float(self.center_weight) * center_score
                    - float(self.narrow_gap_penalty) * narrow_penalty
                )

                if abs(offset) > 0.82:
                    score *= 0.55

                gaps.append(
                    Gap(
                        left=left,
                        right=right,
                        center=center,
                        width=width,
                        offset=offset,
                        depth=gap_depth,
                        score=float(score),
                    )
                )

        if not gaps:
            return None, front_depth, obstacle_offset

        if bool(self.prefer_nearest_gap):
            nearest_candidates = [
                g for g in gaps
                if abs(g.offset) <= float(self.nearest_gap_offset_limit)
                and g.depth >= float(self.nearest_gap_min_depth)
                and g.width >= int(w * float(self.nearest_gap_min_width_ratio))
            ]

            if nearest_candidates:
                best_gap = min(
                    nearest_candidates,
                    key=lambda g: (
                        abs(g.offset),
                        -g.depth,
                        -g.width,
                    ),
                )
            else:
                best_gap = max(gaps, key=lambda g: g.score)
        else:
            best_gap = max(gaps, key=lambda g: g.score)

        return best_gap, front_depth, obstacle_offset

    def yolo_callback(self, msg: Detection2DArray) -> None:
        if self.last_depth is None:
            return

        h, w = self.last_depth.shape[:2]
        now = time.monotonic()
        best: YoloObstacle | None = None

        for det in msg.detections:
            conf = self.get_detection_confidence(det)

            if conf < float(self.yolo_min_confidence):
                continue

            cx, cy, sx, sy = self.get_bbox(det)

            if sx <= 2 or sy <= 2:
                continue

            x1 = int(np.clip(cx - sx * 0.50, 0, w - 1))
            x2 = int(np.clip(cx + sx * 0.50, x1 + 1, w))
            y1 = int(np.clip(cy + sy * 0.10, 0, h - 1))
            y2 = int(np.clip(cy + sy * 0.75, y1 + 1, h))

            patch = self.last_depth[y1:y2, x1:x2]

            valid = patch[
                (patch >= float(self.min_valid_depth))
                & (patch <= float(self.max_valid_depth))
                & np.isfinite(patch)
            ]

            if valid.size < 10:
                continue

            obs = YoloObstacle(
                offset=float((cx - w * 0.5) / (w * 0.5)),
                depth=float(np.percentile(valid, 10)),
                confidence=conf,
                stamp=now,
            )

            if best is None or obs.depth < best.depth:
                best = obs

        if best is not None:
            self.closest_yolo_obstacle = best

    def get_recent_yolo_obstacle(self, now: float) -> YoloObstacle | None:
        if not bool(self.yolo_enable):
            return None
        if self.closest_yolo_obstacle is None:
            return None
        if now - self.closest_yolo_obstacle.stamp > float(self.yolo_timeout):
            return None
        return self.closest_yolo_obstacle

    def get_detection_confidence(self, det) -> float:
        if not det.results:
            return 0.0

        scores = []

        for r in det.results:
            if hasattr(r, 'score'):
                scores.append(float(r.score))
            elif hasattr(r, 'hypothesis') and hasattr(r.hypothesis, 'score'):
                scores.append(float(r.hypothesis.score))

        return max(scores) if scores else 0.0

    def get_bbox(self, det) -> tuple[float, float, float, float]:
        bbox = det.bbox
        size_x = float(bbox.size_x)
        size_y = float(bbox.size_y)
        center = bbox.center

        if hasattr(center, 'position'):
            cx = float(center.position.x)
            cy = float(center.position.y)
        else:
            cx = float(center.x)
            cy = float(center.y)

        return cx, cy, size_x, size_y

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthYoloAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()