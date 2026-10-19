#!/usr/bin/env python3
"""
OpenCV-based Industrial Welding Seam Detector from Overhead RGB-D Camera.
Processes 2D RGB image and 3D depth map to locate the physical welding seam groove,
converts pixel coordinates to 3D metric camera coordinates, transforms them to
the robot base/world frame via TF, and produces certified Cartesian waypoints.
"""

import math
import numpy as np
import cv2

from geometry_msgs.msg import Pose, Point, Quaternion
from abb_welding_standalone.trajectory_utils import compute_torch_orientation


class SeamDetector:
    def __init__(self, step_size=0.005, standoff_clearance=0.003, push_angle_deg=12.0):
        self.step_size = step_size
        self.standoff_clearance = standoff_clearance
        self.push_angle_rad = math.radians(push_angle_deg)

    def generate_waypoints_from_camera(
        self,
        color_img,
        depth_img,
        camera_info,
        transform_cam_to_world
    ):
        """
        Complete vision-guided pipeline:
        1. Segment the elevated workpiece resting on the welding table via depth & contours.
        2. Detect the interior physical weld seam groove separating the metal plates.
        3. Deproject seam endpoints to 3D metric camera coordinates using camera intrinsics.
        4. Transform 3D coordinates into robot base/world frame via TF matrix.
        5. Generate interpolated Cartesian waypoints with industrial standoff and push angle.
        
        Returns:
            waypoints (list of Pose), metadata (dict), annotated_img (cv2 BGR image)
        """
        if color_img is None or depth_img is None:
            return None, None, color_img

        annotated = color_img.copy()
        h, w = depth_img.shape[:2]

        # 1. Depth gating: isolate table and workpiece region (0.50m - 1.15m)
        valid_mask = (~np.isnan(depth_img)) & (depth_img > 0.50) & (depth_img < 1.15)
        if np.count_nonzero(valid_mask) < 2000:
            return None, None, annotated

        # Table surface depth distribution (dominant surface around 0.90m - 0.95m)
        table_candidates = depth_img[valid_mask]
        table_depth = float(np.percentile(table_candidates, 60))

        # 2. Segment elevated workpiece resting on table (thickness 12mm to 40mm)
        wp_mask = (depth_img > 0.60) & (depth_img < table_depth - 0.012) & (~np.isnan(depth_img))
        wp_uint8 = (wp_mask * 255).astype(np.uint8)

        # Morphological filtering to close gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        wp_clean = cv2.morphologyEx(wp_uint8, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(wp_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_cnt = None
        max_area = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area > 1000.0 and area > max_area:
                max_area = area
                best_cnt = c

        if best_cnt is None:
            return None, None, annotated

        bx, by, bw, bh = cv2.boundingRect(best_cnt)

        # 3. Detect physical seam groove inside workpiece bounding box
        roi_rgb = color_img[by:by+bh, bx:bx+bw]
        roi_gray = cv2.cvtColor(roi_rgb, cv2.COLOR_BGR2GRAY)
        roi_filtered = cv2.GaussianBlur(roi_gray, (3, 3), 0)
        edges = cv2.Canny(roi_filtered, 20, 60)

        min_line_len = int(min(bw, bh) * 0.30)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=18, minLineLength=min_line_len, maxLineGap=15
        )

        if lines is None or len(lines) == 0:
            return None, None, annotated

        # Filter lines to interior of the workpiece (rejecting outer plate boundaries)
        margin_x = 0.08 * bw
        margin_y = 0.08 * bh
        seam_candidates = []
        for l in lines:
            lx1, ly1, lx2, ly2 = l[0]
            mid_lx = (lx1 + lx2) / 2.0
            mid_ly = (ly1 + ly2) / 2.0
            if (margin_x < mid_lx < bw - margin_x) and (margin_y < mid_ly < bh - margin_y):
                length = math.hypot(lx2 - lx1, ly2 - ly1)
                angle = math.atan2(ly2 - ly1, lx2 - lx1)
                seam_candidates.append((lx1, ly1, lx2, ly2, length, angle))

        if not seam_candidates:
            return None, None, annotated

        # Sort by length and pick best interior seam groove
        seam_candidates.sort(key=lambda item: item[4], reverse=True)
        lx1, ly1, lx2, ly2, slen, sang = seam_candidates[0]

        gx1, gy1 = int(lx1 + bx), int(ly1 + by)
        gx2, gy2 = int(lx2 + bx), int(ly2 + by)

        # Extend seam across workpiece bounding box along fitted line direction
        cos_a = math.cos(sang)
        sin_a = math.sin(sang)

        if abs(cos_a) >= abs(sin_a):
            # Primarily horizontal seam in image (along Y in world coordinates)
            p1_x = bx + int(bw * 0.04)
            p2_x = bx + bw - int(bw * 0.04)
            tan_a = sin_a / cos_a if abs(cos_a) > 1e-4 else 0.0
            p1_y = int(gy1 + (p1_x - gx1) * tan_a)
            p2_y = int(gy1 + (p2_x - gx1) * tan_a)
        else:
            # Primarily vertical seam in image
            p1_y = by + int(bh * 0.04)
            p2_y = by + bh - int(bh * 0.04)
            cot_a = cos_a / sin_a if abs(sin_a) > 1e-4 else 0.0
            p1_x = int(gx1 + (p1_y - gy1) * cot_a)
            p2_x = int(gx1 + (p2_y - gy1) * cot_a)

        # Clip endpoints to within image
        p1_x = int(np.clip(p1_x, 0, w - 1))
        p2_x = int(np.clip(p2_x, 0, w - 1))
        p1_y = int(np.clip(p1_y, 0, h - 1))
        p2_y = int(np.clip(p2_y, 0, h - 1))

        # In the ROI around the workpiece, determine the table top surface depth
        roi_depth = depth_img[by:by+bh, bx:bx+bw]
        valid_roi = roi_depth[(roi_depth > 0.60) & (~np.isnan(roi_depth))]
        if len(valid_roi) == 0:
            return None, None, annotated
        table_top = float(np.percentile(valid_roi, 70))

        # Workpiece plates are elevated relative to table_top by > 8mm
        plate_pixels = roi_depth[(roi_depth > 0.60) & (roi_depth < table_top - 0.008) & (~np.isnan(roi_depth))]
        if len(plate_pixels) > 0:
            seam_depth = float(np.median(plate_pixels))
        else:
            seam_depth = table_top - 0.016

        # Camera intrinsics matrix K
        if hasattr(camera_info, 'k') and len(camera_info.k) >= 9:
            K = np.array(camera_info.k).reshape((3, 3))
        else:
            fx = 554.25469
            K = np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1.0]])

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        def deproject(u, v, z):
            return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])

        # Deproject to 3D and transform to world frame
        pt1_w = (transform_cam_to_world @ deproject(p1_x, p1_y, seam_depth))[:3]
        pt2_w = (transform_cam_to_world @ deproject(p2_x, p2_y, seam_depth))[:3]

        # Ensure consistent travel direction (smaller Y to larger Y, or smaller X to larger X)
        if pt1_w[1] > pt2_w[1] or (abs(pt1_w[1] - pt2_w[1]) < 0.01 and pt1_w[0] > pt2_w[0]):
            pt1_w, pt2_w = pt2_w, pt1_w
            p1_x, p1_y, p2_x, p2_y = p2_x, p2_y, p1_x, p1_y

        seam_vec = pt2_w - pt1_w
        seam_len_m = float(np.linalg.norm(seam_vec))

        # Check physically plausible seam dimensions (40mm to 400mm)
        if seam_len_m < 0.04 or seam_len_m > 0.40:
            return None, None, annotated

        travel_dir = seam_vec / seam_len_m
        num_waypoints = max(2, int(np.ceil(seam_len_m / self.step_size)))

        # Arc orientation with push angle
        orientation_msg = compute_torch_orientation(
            travel_dir[0], travel_dir[1], push_angle_rad=self.push_angle_rad
        )

        waypoints = []
        for i in range(num_waypoints + 1):
            s = float(i) / float(num_waypoints)
            pos = pt1_w + s * seam_vec
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2]) + self.standoff_clearance
            pose.orientation = orientation_msg
            waypoints.append(pose)

        # Annotate image
        cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.line(annotated, (p1_x, p1_y), (p2_x, p2_y), (0, 0, 255), 3)
        cv2.circle(annotated, (p1_x, p1_y), 5, (255, 0, 0), -1)    # Blue = start
        cv2.circle(annotated, (p2_x, p2_y), 5, (0, 255, 255), -1)  # Yellow = end
        cv2.putText(
            annotated,
            f"SEAM: {seam_len_m * 1000.0:.1f}mm",
            (bx, max(20, by - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

        metadata = {
            "seam_length_mm": float(seam_len_m * 1000.0),
            "start_world": (float(pt1_w[0]), float(pt1_w[1]), float(pt1_w[2])),
            "end_world": (float(pt2_w[0]), float(pt2_w[1]), float(pt2_w[2])),
            "start_pixel": (p1_x, p1_y),
            "end_pixel": (p2_x, p2_y),
            "waypoint_count": len(waypoints),
            "workpiece_bbox": (bx, by, bw, bh),
            "seam_depth_m": seam_depth,
            "confidence": 1.0,
        }

        return waypoints, metadata, annotated
