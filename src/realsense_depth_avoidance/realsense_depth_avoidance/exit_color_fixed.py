cat > ~/jackal_ws/src/jackal_chase/jackal_chase/exit_color_fixed.py << 'EOF'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from collections import deque
import time

class ExitDetector:
    def __init__(self):
        # 扫描行适配1m蓝色围栏，只在画面下半有效区域
        self.scan_lines = [300, 320, 340, 360, 380]
        self.depth_jump_threshold = 0.25
        self.min_width = 140
        self.max_width = 270
        self.min_rows = 3

        # 蓝色HSV阈值
        self.blue_lower = np.array([100, 50, 50])
        self.blue_upper = np.array([130, 255, 255])

        # 平滑滤波
        self.angle_buffer = deque(maxlen=10)
        self.dist_buffer = deque(maxlen=10)

        # 多帧确认防误检
        self.confirm_buffer = deque(maxlen=5)
        self.confirm_threshold = 3
        self.max_angle_change = 12.0
        self.last_valid_result = None
        self.last_valid_count = 0
        self.last_detect_time = 0.0
        self.exit_timeout = 2.5

    def get_blue_mask(self, color_image):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        kernel_big = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_big)
        return mask

    def scan_row(self, depth_row, blue_row, y):
        w = len(depth_row)
        candidates = []
        blue_regions = []
        in_blue = False
        start = 0
        for x in range(w):
            is_blue = blue_row[x] > 0
            if is_blue and not in_blue:
                start = x
                in_blue = True
            if not is_blue and in_blue:
                blue_regions.append((start, x))
                in_blue = False
        if in_blue:
            blue_regions.append((start, w-1))

        for i in range(len(blue_regions) - 1):
            left_blue = blue_regions[i]
            right_blue = blue_regions[i+1]
            gap_start = left_blue[1]
            gap_end = right_blue[0]
            width = gap_end - gap_start
            if self.min_width < width < self.max_width:
                center = (gap_start + gap_end) // 2
                depth_inside = depth_row[center]
                left_check = min(left_blue[1] - 5, w - 1)
                right_check = min(right_blue[0] + 5, w - 1)
                depth_left = depth_row[left_check]
                depth_right = depth_row[right_check]
                depth_inside_m = depth_inside / 1000.0
                gap_min_depth_diff = 0.22 + depth_inside_m * 0.06
                if depth_inside > depth_left + gap_min_depth_diff and depth_inside > depth_right + gap_min_depth_diff:
                    candidates.append({
                        'center': center,
                        'width': width,
                        'depth': depth_inside,
                        'y': y
                    })
        return candidates

    def feed_frame(self, color_image, depth_image):
        """外部投喂图像，不再自行读取相机"""
        if color_image is None or depth_image is None:
            return None
        h, w = depth_image.shape
        roi_top = int(h * 0.55)
        color_roi = color_image[roi_top:h, :].copy()
        depth_roi = depth_image[roi_top:h, :].copy()
        h_roi, w_roi = depth_roi.shape
        depth_m = depth_roi / 1000.0
        blue_mask = self.get_blue_mask(color_roi)
        all_candidates = []
        for y_full in self.scan_lines:
            y_roi = y_full - roi_top
            if not (0 <= y_roi < h_roi):
                continue
            depth_row = depth_m[y_roi, :]
            blue_row = blue_mask[y_roi, :]
            candidates = self.scan_row(depth_row, blue_row, y_full)
            all_candidates.extend(candidates)
        now = time.time()
        if not all_candidates:
            if self.last_valid_result is not None and (now - self.last_detect_time) > self.exit_timeout:
                self.confirm_buffer.clear()
                self.last_valid_count = 0
                self.last_valid_result = None
            return None
        all_candidates.sort(key=lambda x: x['depth'], reverse=True)
        best = all_candidates[0]
        raw_distance = best['depth'] / 1000.0
        if raw_distance < 0.6 or raw_distance > 4.2:
            self.confirm_buffer.clear()
            self.last_valid_count = 0
            self.last_valid_result = None
            return None
        raw_angle = (best['center'] - 320) / 320 * 34.5
        self.angle_buffer.append(raw_angle)
        self.dist_buffer.append(raw_distance)
        smooth_angle = np.median(self.angle_buffer)
        smooth_distance = np.median(self.dist_buffer)
        if self.last_valid_result is not None:
            angle_change = abs(smooth_angle - self.last_valid_result['angle'])
            if angle_change > self.max_angle_change:
                return None
        self.confirm_buffer.append({
            'angle': smooth_angle,
            'distance': smooth_distance,
            'center_x': best['center'],
            'y': best['y']
        })
        res_out = None
        if len(self.confirm_buffer) >= self.confirm_threshold:
            angles = [r['angle'] for r in self.confirm_buffer]
            angle_std = np.std(angles)
            if angle_std < 4.0:
                self.last_valid_count += 1
                res_out = {
                    'angle': np.mean(angles),
                    'distance': np.mean([r['distance'] for r in self.confirm_buffer]),
                    'center_x': best['center'],
                    'y': best['y'],
                    'raw_angle': raw_angle,
                    'raw_distance': raw_distance,
                    'confirmed': True,
                    'confirm_count': self.last_valid_count
                }
                self.last_valid_result = res_out
                self.last_detect_time = now
                self.confirm_buffer.clear()
        if res_out is None and self.last_valid_result is not None:
            if (now - self.last_detect_time) < self.exit_timeout:
                res_out = self.last_valid_result
        return res_out

    def stop(self):
        """无内置相机，空接口兼容调用"""
        pass
EOF
echo "✅ exit_color_fixed.py 写入完成"