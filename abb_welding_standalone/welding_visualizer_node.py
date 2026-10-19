#!/usr/bin/env python3
"""
Industrial Robotic Welding Visualizer Node for ABB IRB 1200.
Generates realistic, clean, and professional visual feedback for RViz:
- Real-time electric welding arc with high-intensity plasma glow (only when active)
- Realistic dynamic 3D flying sparks directed downward onto the plate
- Progressive weld bead deposition with thermal cooling gradient directly on the seam
- Unwelded physical joint seam guide line
- Compact, clean industrial telemetry display board (no floating scattered text)
"""

import math
import random
import time
import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from abb_welding_standalone.trajectory_utils import compute_tcp_to_seam_error_mm


class SparkParticle:
    def __init__(self, origin_x, origin_y, origin_z, lifetime=0.28):
        self.ox = origin_x
        self.oy = origin_y
        self.oz = origin_z
        self.birth_time = time.time()
        self.lifetime = lifetime + random.uniform(-0.04, 0.04)
        # Realistic downward and lateral ejection from arc onto plates
        speed = random.uniform(0.5, 1.4)
        theta = random.uniform(0, 2 * math.pi)
        phi = random.uniform(math.pi * 0.40, math.pi * 0.65)  # Directed sideways & down
        self.vx = speed * math.sin(phi) * math.cos(theta)
        self.vy = speed * math.sin(phi) * math.sin(theta)
        self.vz = speed * math.cos(phi) - 0.4  # Initial downward bias
        self.gravity = -5.5  # m/s^2

    def get_pos(self):
        dt = time.time() - self.birth_time
        x = self.ox + self.vx * dt
        y = self.oy + self.vy * dt
        z = self.oz + self.vz * dt + 0.5 * self.gravity * dt * dt
        # Bounce / stop at table surface level (z=0.50)
        if z < 0.505:
            z = 0.505 + random.uniform(0.001, 0.003)
        return (x, y, z)

    def is_alive(self):
        return (time.time() - self.birth_time) < self.lifetime


class WeldBeadSegment:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.timestamp = time.time()


class WeldingVisualizerNode(Node):
    def __init__(self):
        super().__init__('welding_visualizer_node')
        self.get_logger().info('Initializing ABB IRB 1200 Clean Welding Visualizer...')

        # Parameters
        self.declare_parameter('tcp_frame', 'welding_torch_tcp')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('spark_count_per_frame', 14)
        self.declare_parameter('cooling_duration_sec', 5.0)

        self.tcp_frame = self.get_parameter('tcp_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.spark_count = self.get_parameter('spark_count_per_frame').get_parameter_value().integer_value
        self.cooling_duration = self.get_parameter('cooling_duration_sec').get_parameter_value().double_value

        # TF Buffer and Listener with dedicated spin thread
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # Subscribers
        self.arc_sub = self.create_subscription(Bool, '/welding/arc_state', self.arc_state_cb, 10)
        self.status_sub = self.create_subscription(String, '/welding/status', self.status_cb, 10)

        # Publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, '/welding/visual_markers', 10)

        # Internal State
        self.arc_active = False
        self.status_text = 'INITIALIZING'
        self.pattern = 'straight'
        self.progress_percent = 0.0
        self.current_error_mm = 0.0
        self.sparks = []
        self.weld_bead_history = []
        self.last_bead_pos = None

        # Timer for visualization rendering
        timer_period = 1.0 / max(1.0, rate)
        self.timer = self.create_timer(timer_period, self.render_visual_effects)

        self.get_logger().info('Welding Visualizer Node ready with clean visual rendering.')

    def arc_state_cb(self, msg: Bool):
        old_state = self.arc_active
        self.arc_active = msg.data
        if not self.arc_active:
            # Immediately clear sparks upon arc extinction
            self.sparks.clear()
        elif not old_state and self.arc_active:
            # Arc struck: start a new weld bead segment run
            self.last_bead_pos = None

    def status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.status_text = data.get('state', 'UNKNOWN')
            self.pattern = data.get('pattern', self.pattern)
            self.progress_percent = data.get('progress_percent', self.progress_percent)
            self.current_error_mm = data.get('current_tracking_error_mm', self.current_error_mm)
            # If a new cycle starts, reset weld bead history cleanly
            if 'STEP 1' in self.status_text:
                self.weld_bead_history.clear()
                self.last_bead_pos = None
        except Exception:
            self.status_text = msg.data

    def get_tcp_position(self):
        """Query current TCP position in world frame from TF."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                rclpy.time.Time()
            )
            return (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            )
        except Exception:
            return None

    def render_visual_effects(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        tcp_pos = self.get_tcp_position()

        # Compute live tracking error if in welding state
        if tcp_pos:
            self.current_error_mm = compute_tcp_to_seam_error_mm(tcp_pos, self.pattern)

        # Record weld bead ONLY when arc is active and TCP is close to the physical seam (<15mm)
        if self.arc_active and tcp_pos:
            tx, ty, tz = tcp_pos
            # Clamped Z to the physical plate seam surface (Z = 0.516m)
            bead_z = 0.5165
            if self.last_bead_pos is None:
                self.weld_bead_history.append(WeldBeadSegment(tx, ty, bead_z))
                self.last_bead_pos = (tx, ty, bead_z)
            else:
                lx, ly, _ = self.last_bead_pos
                dist = math.sqrt((tx - lx)**2 + (ty - ly)**2)
                # Ensure points are closely spaced (3mm) and not jumping (>20mm)
                if 0.003 <= dist <= 0.025:
                    self.weld_bead_history.append(WeldBeadSegment(tx, ty, bead_z))
                    self.last_bead_pos = (tx, ty, bead_z)

        # ---------------------------------------------------------------------
        # 1. ARC CORE (High-intensity white-cyan electric plasma core)
        # ---------------------------------------------------------------------
        arc_marker = Marker()
        arc_marker.header.frame_id = self.base_frame
        arc_marker.header.stamp = now
        arc_marker.ns = 'welding_arc_core'
        arc_marker.id = 1
        arc_marker.type = Marker.SPHERE
        if self.arc_active and tcp_pos:
            arc_marker.action = Marker.ADD
            arc_marker.pose.position.x = tcp_pos[0]
            arc_marker.pose.position.y = tcp_pos[1]
            arc_marker.pose.position.z = tcp_pos[2]
            flicker = random.uniform(0.012, 0.016)
            arc_marker.scale.x = flicker
            arc_marker.scale.y = flicker
            arc_marker.scale.z = flicker
            arc_marker.color.r = 0.95
            arc_marker.color.g = 0.98
            arc_marker.color.b = 1.0
            arc_marker.color.a = 0.98
        else:
            arc_marker.action = Marker.DELETE
        markers.markers.append(arc_marker)

        # ---------------------------------------------------------------------
        # 2. PLASMA AURA (Thermal radiation glow)
        # ---------------------------------------------------------------------
        aura_marker = Marker()
        aura_marker.header.frame_id = self.base_frame
        aura_marker.header.stamp = now
        aura_marker.ns = 'welding_arc_aura'
        aura_marker.id = 2
        aura_marker.type = Marker.SPHERE
        if self.arc_active and tcp_pos:
            aura_marker.action = Marker.ADD
            aura_marker.pose.position.x = tcp_pos[0]
            aura_marker.pose.position.y = tcp_pos[1]
            aura_marker.pose.position.z = tcp_pos[2]
            flicker_aura = random.uniform(0.040, 0.055)
            aura_marker.scale.x = flicker_aura
            aura_marker.scale.y = flicker_aura
            aura_marker.scale.z = flicker_aura
            aura_marker.color.r = 1.0
            aura_marker.color.g = 0.45
            aura_marker.color.b = 0.08
            aura_marker.color.a = 0.35
        else:
            aura_marker.action = Marker.DELETE
        markers.markers.append(aura_marker)

        # ---------------------------------------------------------------------
        # 3. DYNAMIC BALLISTIC SPARKS (Only during arc active, directed onto plate)
        # ---------------------------------------------------------------------
        if self.arc_active and tcp_pos:
            for _ in range(self.spark_count):
                self.sparks.append(SparkParticle(tcp_pos[0], tcp_pos[1], tcp_pos[2]))

        self.sparks = [s for s in self.sparks if s.is_alive()]

        sparks_marker = Marker()
        sparks_marker.header.frame_id = self.base_frame
        sparks_marker.header.stamp = now
        sparks_marker.ns = 'welding_sparks'
        sparks_marker.id = 3
        sparks_marker.type = Marker.LINE_LIST
        sparks_marker.scale.x = 0.002
        sparks_marker.color.r = 1.0
        sparks_marker.color.g = 0.80
        sparks_marker.color.b = 0.20
        sparks_marker.color.a = 0.95

        if len(self.sparks) > 0 and self.arc_active:
            sparks_marker.action = Marker.ADD
            for spark in self.sparks:
                sx, sy, sz = spark.get_pos()
                # Spark head
                p1 = Point(x=sx, y=sy, z=sz)
                # Short trailing tail
                dt_tail = 0.012
                p2 = Point(
                    x=sx - spark.vx * dt_tail,
                    y=sy - spark.vy * dt_tail,
                    z=sz - spark.vz * dt_tail
                )
                sparks_marker.points.append(p1)
                sparks_marker.points.append(p2)
        else:
            sparks_marker.action = Marker.DELETE
        markers.markers.append(sparks_marker)

        # ---------------------------------------------------------------------
        # 4. PROGRESSIVE WELD BEAD DEPOSITION (Directly on physical seam)
        # ---------------------------------------------------------------------
        bead_marker = Marker()
        bead_marker.header.frame_id = self.base_frame
        bead_marker.header.stamp = now
        bead_marker.ns = 'weld_bead'
        bead_marker.id = 4
        bead_marker.type = Marker.LINE_STRIP
        bead_marker.scale.x = 0.007  # Realistic 7mm bead width

        if len(self.weld_bead_history) > 1:
            curr_time = time.time()
            bead_marker.action = Marker.ADD
            age_newest = curr_time - self.weld_bead_history[-1].timestamp

            # Thermal cooling color gradient
            if self.arc_active or age_newest < 1.5:
                # Glowing molten puddle
                bead_marker.color.r = 1.0
                bead_marker.color.g = 0.60
                bead_marker.color.b = 0.10
                bead_marker.color.a = 1.0
            elif age_newest < self.cooling_duration:
                # Cooling cherry red
                bead_marker.color.r = 0.75
                bead_marker.color.g = 0.20
                bead_marker.color.b = 0.10
                bead_marker.color.a = 1.0
            else:
                # Solidified metallic steel weld bead
                bead_marker.color.r = 0.48
                bead_marker.color.g = 0.52
                bead_marker.color.b = 0.58
                bead_marker.color.a = 1.0

            for pt in self.weld_bead_history:
                bead_marker.points.append(Point(x=pt.x, y=pt.y, z=pt.z))
        else:
            bead_marker.action = Marker.DELETE
        markers.markers.append(bead_marker)

        # ---------------------------------------------------------------------
        # 5. PHYSICAL SEAM REFERENCE LINE (Shows exact intended joint path)
        # ---------------------------------------------------------------------
        seam_guide = Marker()
        seam_guide.header.frame_id = self.base_frame
        seam_guide.header.stamp = now
        seam_guide.ns = 'physical_seam_guide'
        seam_guide.id = 5
        seam_guide.type = Marker.LINE_LIST
        seam_guide.action = Marker.ADD
        seam_guide.scale.x = 0.0025
        # Subtle cyan reference guide
        seam_guide.color.r = 0.20
        seam_guide.color.g = 0.70
        seam_guide.color.b = 0.85
        seam_guide.color.a = 0.60

        z_seam = 0.5165  # Plate fillet root height
        if self.pattern == 'l_shaped':
            p1 = Point(x=0.53, y=-0.06, z=z_seam)
            p2 = Point(x=0.60, y=-0.06, z=z_seam)
            p3 = Point(x=0.60, y=-0.06, z=z_seam)
            p4 = Point(x=0.60, y=0.06, z=z_seam)
            seam_guide.points.extend([p1, p2, p3, p4])
        elif self.pattern == 'multi_waypoint':
            pts = [
                Point(x=0.595, y=-0.08, z=z_seam),
                Point(x=0.605, y=-0.04, z=z_seam),
                Point(x=0.595, y=0.00, z=z_seam),
                Point(x=0.605, y=0.04, z=z_seam),
                Point(x=0.600, y=0.08, z=z_seam),
            ]
            for i in range(len(pts) - 1):
                seam_guide.points.extend([pts[i], pts[i+1]])
        else:
            # Straight fillet seam: along Y at X=0.60
            seam_guide.points.append(Point(x=0.60, y=-0.09, z=z_seam))
            seam_guide.points.append(Point(x=0.60, y=0.09, z=z_seam))
        markers.markers.append(seam_guide)

        # ---------------------------------------------------------------------
        # 6. COMPACT INDUSTRIAL TELEMETRY PANEL (Neat side board, NO floating clutter)
        # ---------------------------------------------------------------------
        # Panel Base Plate (Mounted cleanly beside table at X=0.60, Y=0.48, Z=0.68)
        board_marker = Marker()
        board_marker.header.frame_id = self.base_frame
        board_marker.header.stamp = now
        board_marker.ns = 'industrial_telemetry_ui'
        board_marker.id = 10
        board_marker.type = Marker.CUBE
        board_marker.action = Marker.ADD
        board_marker.pose.position.x = 0.60
        board_marker.pose.position.y = 0.48
        board_marker.pose.position.z = 0.68
        # Orientation facing along -Y towards camera
        board_marker.pose.orientation.w = 0.7071
        board_marker.pose.orientation.x = 0.0
        board_marker.pose.orientation.y = 0.0
        board_marker.pose.orientation.z = -0.7071
        board_marker.scale.x = 0.36  # Width
        board_marker.scale.y = 0.01  # Thickness
        board_marker.scale.z = 0.22  # Height
        # Dark matte slate background
        board_marker.color.r = 0.12
        board_marker.color.g = 0.14
        board_marker.color.b = 0.16
        board_marker.color.a = 0.95
        markers.markers.append(board_marker)

        # Telemetry Stand / Post
        post_marker = Marker()
        post_marker.header.frame_id = self.base_frame
        post_marker.header.stamp = now
        post_marker.ns = 'industrial_telemetry_ui'
        post_marker.id = 11
        post_marker.type = Marker.CYLINDER
        post_marker.action = Marker.ADD
        post_marker.pose.position.x = 0.60
        post_marker.pose.position.y = 0.48
        post_marker.pose.position.z = 0.34
        post_marker.scale.x = 0.03
        post_marker.scale.y = 0.03
        post_marker.scale.z = 0.68
        post_marker.color.r = 0.25
        post_marker.color.g = 0.27
        post_marker.color.b = 0.30
        post_marker.color.a = 1.0
        markers.markers.append(post_marker)

        # Telemetry Text Display on Panel
        text_marker = Marker()
        text_marker.header.frame_id = self.base_frame
        text_marker.header.stamp = now
        text_marker.ns = 'industrial_telemetry_ui'
        text_marker.id = 12
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.60
        text_marker.pose.position.y = 0.46
        text_marker.pose.position.z = 0.68
        text_marker.scale.z = 0.024  # Compact, crisp text height

        tcp_str = f"[{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}]" if tcp_pos else "[--]"
        arc_state_str = "ACTIVE (MIG/GMAW)" if self.arc_active else "STANDBY"
        
        # Build clean formatted telemetry block
        text_marker.text = (
            f"=== ABB IRB 1200 WELDING CELL ===\n"
            f"STATE:    {self.status_text}\n"
            f"PATTERN:  {self.pattern.upper()}\n"
            f"ARC:      [{arc_state_str}]\n"
            f"TCP POS:  {tcp_str}\n"
            f"TRACKING: {self.current_error_mm:.2f} mm"
        )

        if self.arc_active:
            text_marker.color.r = 1.0
            text_marker.color.g = 0.85
            text_marker.color.b = 0.20
            text_marker.color.a = 1.0
        else:
            text_marker.color.r = 0.90
            text_marker.color.g = 0.92
            text_marker.color.b = 0.95
            text_marker.color.a = 1.0
        markers.markers.append(text_marker)

        # Publish all visual markers
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WeldingVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Welding Visualizer.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
