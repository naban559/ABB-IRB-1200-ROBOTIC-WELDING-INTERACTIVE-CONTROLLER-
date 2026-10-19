"""
Trajectory and Geometric Utilities for Industrial Robotic Welding Simulation.
Provides quaternion math, torch orientation alignment, physical seam waypoint generators,
and analytical TCP-to-seam tracking error evaluation.
"""

import math
from geometry_msgs.msg import Pose, Point, Quaternion


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to geometry_msgs.msg.Quaternion.
    Rotation order: Z-Y-X (yaw, pitch, roll).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-9:
        norm = 1.0

    q = Quaternion()
    q.w = qw / norm
    q.x = qx / norm
    q.y = qy / norm
    q.z = qz / norm
    return q


def compute_torch_orientation(travel_dir_x, travel_dir_y, push_angle_rad=0.209):
    """
    Compute torch orientation quaternion for welding along a direction in the XY plane.
    The torch points downward (-Z), tilted along the travel direction by push_angle_rad.
    push_angle_rad = 12 deg (0.209 rad) standard GMAW industrial push angle.
    """
    heading = math.atan2(travel_dir_y, travel_dir_x)
    roll = 0.0
    pitch = math.pi - push_angle_rad
    yaw = heading
    return quaternion_from_euler(roll, pitch, yaw)


def generate_straight_seam_waypoints(start_pt, end_pt, step_size=0.005, push_angle_rad=0.209):
    """
    Generate dense Cartesian waypoints along a straight physical seam.
    Orientation is strictly locked to the travel heading and push angle.
    """
    dx = end_pt[0] - start_pt[0]
    dy = end_pt[1] - start_pt[1]
    dz = end_pt[2] - start_pt[2]
    total_len = math.sqrt(dx*dx + dy*dy + dz*dz)

    if total_len < 1e-5:
        num_steps = 1
    else:
        num_steps = max(2, int(math.ceil(total_len / step_size)))

    q_orient = compute_torch_orientation(dx, dy, push_angle_rad)

    waypoints = []
    for i in range(num_steps + 1):
        t = float(i) / float(num_steps)
        p = Pose()
        p.position.x = start_pt[0] + t * dx
        p.position.y = start_pt[1] + t * dy
        p.position.z = start_pt[2] + t * dz
        p.orientation = q_orient
        waypoints.append(p)

    return waypoints


def generate_l_shaped_seam_waypoints(pt_start, pt_corner, pt_end, step_size=0.005, push_angle_rad=0.209, blend_radius=0.006):
    """
    Generate dense Cartesian waypoints along an L-shaped physical joint with a smooth
    industrial corner blend radius.
    Segment 1: Approaching along X towards corner.
    Fillet: Circular blend turning smoothly from +X to +Y.
    Segment 2: Continuing along Y along main fillet plate.
    """
    R = blend_radius
    x1, y1, z1 = pt_start
    xc, yc, zc = pt_corner
    x2, y2, z2 = pt_end

    p_blend_start = (xc - R, yc, zc)
    p_blend_end = (xc, yc + R, zc)

    waypoints = []

    # 1. Segment 1: from pt_start to p_blend_start
    len1 = math.sqrt((p_blend_start[0] - x1)**2 + (p_blend_start[1] - y1)**2)
    n1 = max(2, int(math.ceil(len1 / step_size)))
    heading1 = math.atan2(yc - y1, xc - x1)
    q1 = quaternion_from_euler(0.0, math.pi - push_angle_rad, heading1)

    for i in range(n1):
        t = float(i) / float(n1)
        p = Pose()
        p.position.x = x1 + t * (p_blend_start[0] - x1)
        p.position.y = y1 + t * (p_blend_start[1] - y1)
        p.position.z = zc
        p.orientation = q1
        waypoints.append(p)

    # 2. Filleted Circular Blend from p_blend_start to p_blend_end
    arc_center_x = xc - R
    arc_center_y = yc + R
    arc_len = 0.5 * math.pi * R
    n_arc = max(3, int(math.ceil(arc_len / step_size)))

    for b in range(n_arc + 1):
        theta = -0.5 * math.pi + (float(b) / float(n_arc)) * (0.5 * math.pi)
        px = arc_center_x + R * math.cos(theta)
        py = arc_center_y + R * math.sin(theta)
        h = theta + 0.5 * math.pi
        p = Pose()
        p.position.x = px
        p.position.y = py
        p.position.z = zc
        p.orientation = quaternion_from_euler(0.0, math.pi - push_angle_rad, h)
        waypoints.append(p)

    # 3. Segment 2: from p_blend_end to pt_end
    len2 = math.sqrt((x2 - p_blend_end[0])**2 + (y2 - p_blend_end[1])**2)
    n2 = max(2, int(math.ceil(len2 / step_size)))
    heading2 = math.atan2(y2 - yc, x2 - xc)
    q2 = quaternion_from_euler(0.0, math.pi - push_angle_rad, heading2)

    for j in range(1, n2 + 1):
        t = float(j) / float(n2)
        p = Pose()
        p.position.x = p_blend_end[0] + t * (x2 - p_blend_end[0])
        p.position.y = p_blend_end[1] + t * (y2 - p_blend_end[1])
        p.position.z = zc
        p.orientation = q2
        waypoints.append(p)

    return waypoints


def generate_multi_waypoint_seam(points, step_size=0.005, push_angle_rad=0.209):
    """
    Generate continuous Cartesian waypoints through a sequence of seam control points.
    Torch orientation smoothly guides through segment transitions.
    """
    full_waypoints = []
    for i in range(len(points) - 1):
        p_a = points[i]
        p_b = points[i+1]
        seg = generate_straight_seam_waypoints(p_a, p_b, step_size, push_angle_rad)
        if i == 0:
            full_waypoints.extend(seg)
        else:
            full_waypoints.extend(seg[1:])
    return full_waypoints


# -----------------------------------------------------------------------------
# ANALYTICAL SEAM TRACKING ERROR EVALUATION
# -----------------------------------------------------------------------------

def dist_point_to_segment_3d(px, py, pz, x1, y1, z1, x2, y2, z2):
    """Compute minimum Euclidean distance from point P to line segment AB."""
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    len_sq = dx*dx + dy*dy + dz*dz

    if len_sq < 1e-10:
        return math.sqrt((px - x1)**2 + (py - y1)**2 + (pz - z1)**2)

    # Parameter t of projection onto line
    t = ((px - x1) * dx + (py - y1) * dy + (pz - z1) * dz) / len_sq
    t = max(0.0, min(1.0, t))  # Clamp to segment bounds

    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    proj_z = z1 + t * dz

    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2 + (pz - proj_z)**2)


def compute_tcp_to_seam_error_mm(tcp_pos, pattern_type, z_seam=0.519, x_seam=0.60):
    """
    Calculates the exact perpendicular distance in millimeters from the actual
    torch TCP position to the physical workpiece seam.
    """
    if tcp_pos is None:
        return 0.0

    px, py, pz = tcp_pos

    if pattern_type == 'straight':
        # Straight physical fillet seam from Y = -0.09 to Y = +0.09 at X = x_seam
        p1 = (x_seam, -0.09, z_seam)
        p2 = (x_seam,  0.09, z_seam)
        d_m = dist_point_to_segment_3d(px, py, pz, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])
        return d_m * 1000.0  # Return in millimeters

    elif pattern_type == 'l_shaped':
        # Segment 1: from (0.53, -0.06) to (0.60, -0.06)
        d1 = dist_point_to_segment_3d(px, py, pz, 0.53, -0.06, z_seam, x_seam, -0.06, z_seam)
        # Segment 2: from (0.60, -0.06) to (0.60, 0.06)
        d2 = dist_point_to_segment_3d(px, py, pz, x_seam, -0.06, z_seam, x_seam, 0.06, z_seam)
        return min(d1, d2) * 1000.0

    elif pattern_type == 'multi_waypoint':
        pts = [
            (x_seam - 0.005, -0.08, z_seam),
            (x_seam + 0.005, -0.04, z_seam),
            (x_seam - 0.005,  0.00, z_seam),
            (x_seam + 0.005,  0.04, z_seam),
            (x_seam,          0.08, z_seam),
        ]
        min_d = float('inf')
        for i in range(len(pts) - 1):
            p1 = pts[i]
            p2 = pts[i+1]
            d = dist_point_to_segment_3d(px, py, pz, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])
            if d < min_d:
                min_d = d
        return min_d * 1000.0

    else:
        # Fallback to straight seam
        p1 = (x_seam, -0.09, z_seam)
        p2 = (x_seam,  0.09, z_seam)
        return dist_point_to_segment_3d(px, py, pz, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]) * 1000.0
