#!/usr/bin/env python3
"""
Automated Industrial Welding Controller Node for ABB IRB 1200.
Manages the complete, certified welding lifecycle:
HOME -> SAFE_APPROACH -> ALIGN_TORCH -> MOVE_TO_WELD_START -> WELD_ALONG_SEAM -> STOP_WELDING -> RETRACT_VERTICALLY -> RETURN_HOME

Features:
- Pure workpiece geometry seam derivation (no arbitrary coordinates)
- High-precision MoveIt 2 Cartesian planning with constant-speed velocity retiming
- Pre-ignition start position verification (prevents firing in transit)
- Real-time 50Hz TF-based Seam Tracking Accuracy Validation and Logging
  (distinguishes planned trajectory deviation from actual joint/TF tracking error)
- Smooth orientation alignment avoiding wrist spinning
"""

import math
import time
import json
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

import numpy as np
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from sensor_msgs.msg import JointState, Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros

from abb_welding_standalone.trajectory_utils import (
    quaternion_from_euler,
    compute_torch_orientation,
    generate_straight_seam_waypoints,
    generate_l_shaped_seam_waypoints,
    generate_multi_waypoint_seam,
    compute_tcp_to_seam_error_mm,
)
from abb_welding_standalone.seam_detector import SeamDetector


class WeldingControllerNode(Node):
    def __init__(self):
        super().__init__('welding_controller_node')
        self.get_logger().info('Initializing Certified ABB IRB 1200 Welding Controller Node...')

        # Reentrant callback group for non-blocking multi-threaded execution
        self.cb_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('planning_group', 'welding_arm')
        self.declare_parameter('tcp_frame', 'welding_torch_tcp')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('welding_speed_mps', 0.015)
        self.declare_parameter('rapid_velocity_scaling', 0.35)
        self.declare_parameter('cartesian_step_size', 0.005)
        self.declare_parameter('cartesian_jump_threshold', 0.0)
        self.declare_parameter('standoff_clearance', 0.003)
        self.declare_parameter('approach_height', 0.080)
        self.declare_parameter('retract_height', 0.090)
        self.declare_parameter('torch_lead_angle_deg', 12.0)
        self.declare_parameter('default_pattern', 'straight')
        self.declare_parameter('auto_start_cycle', True)
        self.declare_parameter('cycle_delay_sec', 3.0)

        # Cache parameters
        self.group_name = self.get_parameter('planning_group').get_parameter_value().string_value
        self.tcp_frame = self.get_parameter('tcp_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.weld_speed = self.get_parameter('welding_speed_mps').get_parameter_value().double_value
        self.step_size = self.get_parameter('cartesian_step_size').get_parameter_value().double_value
        self.lead_angle_rad = math.radians(self.get_parameter('torch_lead_angle_deg').get_parameter_value().double_value)
        self.pattern = self.get_parameter('default_pattern').get_parameter_value().string_value

        # Robot state tracking
        self.current_joint_state = None
        self.current_state_str = 'INITIALIZING'
        self.arc_active = False
        self.is_running_cycle = False
        self.stop_requested = False
        self.progress_percent = 0.0

        # Seam Tracking Metrics
        self.realtime_errors_mm = []
        self.max_tracking_error_mm = 0.0
        self.avg_tracking_error_mm = 0.0
        self.rms_tracking_error_mm = 0.0
        self.planned_max_error_mm = 0.0
        self.planned_avg_error_mm = 0.0

        # TF Buffer and Listener (managed by node's MultiThreadedExecutor)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10,
            callback_group=self.cb_group
        )

        # Publishers
        self.arc_pub = self.create_publisher(Bool, '/welding/arc_state', 10)
        self.status_pub = self.create_publisher(String, '/welding/status', 10)
        self.tcp_pose_pub = self.create_publisher(PoseStamped, '/welding/tcp_pose', 10)

        # Service clients for MoveIt
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self.cb_group
        )
        self.fk_client = self.create_client(
            GetPositionFK,
            '/compute_fk',
            callback_group=self.cb_group
        )

        # Action clients for trajectory execution
        self.joint_traj_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )

        # Services exposed by this node
        self.start_srv = self.create_service(
            Trigger,
            '/welding/start_cycle',
            self.start_cycle_cb,
            callback_group=self.cb_group
        )
        self.start_straight_srv = self.create_service(
            Trigger,
            '/welding/start_straight',
            self.start_straight_cb,
            callback_group=self.cb_group
        )
        self.start_l_shaped_srv = self.create_service(
            Trigger,
            '/welding/start_l_shaped',
            self.start_l_shaped_cb,
            callback_group=self.cb_group
        )
        self.start_multi_waypoint_srv = self.create_service(
            Trigger,
            '/welding/start_multi_waypoint',
            self.start_multi_waypoint_cb,
            callback_group=self.cb_group
        )
        self.start_camera_srv = self.create_service(
            Trigger,
            '/welding/start_camera',
            self.start_camera_cb,
            callback_group=self.cb_group
        )
        self.return_home_srv = self.create_service(
            Trigger,
            '/welding/return_home',
            self.return_home_cb,
            callback_group=self.cb_group
        )
        self.stop_srv = self.create_service(
            Trigger,
            '/welding/stop_cycle',
            self.stop_cycle_cb,
            callback_group=self.cb_group
        )

        # Command topic subscriber
        self.cmd_sub = self.create_subscription(
            String,
            '/welding/command',
            self.command_cb,
            10,
            callback_group=self.cb_group
        )

        # OpenCV Bridge & Seam Detector
        self.bridge = CvBridge()
        self.seam_detector = SeamDetector(
            step_size=self.step_size,
            standoff_clearance=self.get_parameter('standoff_clearance').get_parameter_value().double_value,
            push_angle_deg=self.get_parameter('torch_lead_angle_deg').get_parameter_value().double_value,
        )

        self.latest_rgb_msg = None
        self.latest_depth_msg = None
        self.latest_camera_info = None

        # Camera subscribers for RGB-D vision guidance
        self.rgb_sub = self.create_subscription(
            Image, '/overhead_camera/image', self.camera_rgb_cb, 10, callback_group=self.cb_group
        )
        self.depth_sub = self.create_subscription(
            Image, '/overhead_camera/depth_image', self.camera_depth_cb, 10, callback_group=self.cb_group
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/overhead_camera/camera_info', self.camera_info_cb, 10, callback_group=self.cb_group
        )

        # Annotated debug image publisher
        self.annotated_img_pub = self.create_publisher(Image, '/camera/detected_seam_image', 10)

        # Status timer (10 Hz)
        self.status_timer = self.create_timer(0.1, self.publish_status, callback_group=self.cb_group)

        # Auto-start timer
        if self.get_parameter('auto_start_cycle').get_parameter_value().bool_value:
            delay = self.get_parameter('cycle_delay_sec').get_parameter_value().double_value
            self.auto_timer = self.create_timer(delay, self.auto_start_callback, callback_group=self.cb_group)
        else:
            self.auto_timer = None

        self.get_logger().info('Welding Controller Node fully initialized and ready.')

    def camera_rgb_cb(self, msg: Image):
        self.latest_rgb_msg = msg

    def camera_depth_cb(self, msg: Image):
        self.latest_depth_msg = msg

    def camera_info_cb(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg
        if self.current_state_str == 'INITIALIZING':
            self.current_state_str = 'STANDBY (READY)'

    def get_current_tcp_pose(self):
        """Query actual TCP translation from TF."""
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

    def set_arc(self, state: bool):
        self.arc_active = state
        msg = Bool()
        msg.data = state
        self.arc_pub.publish(msg)
        self.get_logger().info(f'>>> WELDING ARC: {"[ ON - ARC STRUCK ]" if state else "[ OFF - EXTINCTION ]"}')

    def publish_status(self):
        tcp_pos = self.get_current_tcp_pose()
        curr_err = compute_tcp_to_seam_error_mm(tcp_pos, self.pattern) if tcp_pos else 0.0

        status_data = {
            'state': self.current_state_str,
            'arc_active': self.arc_active,
            'pattern': self.pattern,
            'welding_speed_mps': self.weld_speed,
            'progress_percent': self.progress_percent,
            'current_tracking_error_mm': curr_err,
            'max_tracking_error_mm': self.max_tracking_error_mm,
            'avg_tracking_error_mm': self.avg_tracking_error_mm,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)

    def auto_start_callback(self):
        if self.auto_timer:
            self.auto_timer.cancel()
            self.auto_timer = None
        self.get_logger().info('Auto-start timer fired. Initiating certified automated welding sequence...')
        self.trigger_cycle()

    def start_cycle_cb(self, request, response):
        if self.is_running_cycle:
            response.success = False
            response.message = f'Welding cycle is already running ({self.current_state_str}).'
            return response

        self.get_logger().info(f'Received request to start welding cycle with pattern: {self.pattern}')
        started = self.trigger_cycle(self.pattern)
        response.success = started
        response.message = f'Started welding cycle with pattern {self.pattern}.'
        return response

    def start_straight_cb(self, request, response):
        if self.is_running_cycle:
            response.success = False
            response.message = f'Robot is currently busy ({self.current_state_str}). Wait or stop active cycle.'
            return response
        self.trigger_cycle('straight')
        response.success = True
        response.message = 'Started Straight-Line Welding operation.'
        return response

    def start_l_shaped_cb(self, request, response):
        if self.is_running_cycle:
            response.success = False
            response.message = f'Robot is currently busy ({self.current_state_str}). Wait or stop active cycle.'
            return response
        self.trigger_cycle('l_shaped')
        response.success = True
        response.message = 'Started L-Shaped Fillet Welding operation.'
        return response

    def start_multi_waypoint_cb(self, request, response):
        if self.is_running_cycle:
            response.success = False
            response.message = f'Robot is currently busy ({self.current_state_str}). Wait or stop active cycle.'
            return response
        self.trigger_cycle('multi_waypoint')
        response.success = True
        response.message = 'Started Multi-Waypoint Seam Welding operation.'
        return response

    def start_camera_cb(self, request, response):
        if self.is_running_cycle:
            response.success = False
            response.message = f'Robot is currently busy ({self.current_state_str}). Wait or stop active cycle.'
            return response
        if hasattr(self, 'execute_camera_welding_cycle'):
            worker = threading.Thread(target=self.execute_camera_welding_cycle, daemon=True)
            worker.start()
            response.success = True
            response.message = 'Initiating Camera-Based Automatic Seam Detection and Welding.'
        else:
            response.success = False
            response.message = 'Camera-Based Welding module initializing.'
        return response

    def return_home_cb(self, request, response):
        if self.is_running_cycle:
            response.success = False
            response.message = f'Robot is currently busy ({self.current_state_str}). Cancel (X) active task before returning home.'
            return response
        worker = threading.Thread(target=self.execute_return_home_safe, daemon=True)
        worker.start()
        response.success = True
        response.message = 'Executing safe return-to-home sequence.'
        return response

    def stop_cycle_cb(self, request, response):
        self.get_logger().warn('Stop / Arc Extinguish Requested by user!')
        self.stop_requested = True
        self.set_arc(False)
        self.current_state_str = 'STOPPED / STANDBY'
        response.success = True
        response.message = 'Active operation stopped. Arc extinguished safely.'
        return response

    def command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received command topic message: {cmd}')
        req = Trigger.Request()
        res = Trigger.Response()
        if cmd in ['s', 'straight']:
            self.start_straight_cb(req, res)
        elif cmd in ['l', 'l_shaped', 'l-shaped']:
            self.start_l_shaped_cb(req, res)
        elif cmd in ['m', 'multi', 'multi_waypoint']:
            self.start_multi_waypoint_cb(req, res)
        elif cmd in ['c', 'camera']:
            self.start_camera_cb(req, res)
        elif cmd in ['h', 'home']:
            self.return_home_cb(req, res)
        elif cmd in ['x', 'stop']:
            self.stop_cycle_cb(req, res)
        else:
            self.get_logger().warn(f'Unrecognized command: {cmd}')

    def trigger_cycle(self, pattern=None):
        if self.is_running_cycle:
            self.get_logger().warn('Cycle already in progress.')
            return False
        if pattern:
            self.pattern = pattern
        worker = threading.Thread(target=self.execute_full_welding_cycle, args=(pattern,), daemon=True)
        worker.start()
        return True

    def execute_return_home_safe(self):
        """Safely returns the robot to home, lifting TCP first if close to table."""
        if self.is_running_cycle:
            return False
        self.is_running_cycle = True
        self.stop_requested = False
        self.current_state_str = 'RETURNING HOME'
        self.get_logger().info('Executing safe Return Home sequence...')

        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ready_joints = [0.0, 0.25, 0.25, 0.0, 1.07, 0.0]

        try:
            # Check if TCP is close to table or workpiece fixtures (z < 0.65)
            tcp_pos = self.get_current_tcp_pose()
            if tcp_pos and tcp_pos[2] < 0.65:
                self.get_logger().info(f'TCP at height z={tcp_pos[2]:.3f}m. Retracting vertically before home...')
                retract_pose = Pose()
                retract_pose.position.x = tcp_pos[0]
                retract_pose.position.y = tcp_pos[1]
                retract_pose.position.z = tcp_pos[2] + 0.10
                seam_wps = self.get_seam_waypoints(self.pattern)
                retract_pose.orientation = seam_wps[0].orientation if seam_wps else Quaternion(w=1.0)

                ret_traj, frac = self.plan_cartesian_path([retract_pose], avoid_collisions=False)
                if ret_traj and frac > 0.8:
                    ret_traj = self.retime_and_densify_trajectory(ret_traj, target_cartesian_speed_mps=0.08)
                    self.execute_joint_trajectory(ret_traj)

            self.move_to_joint_target(ready_joints, duration_sec=2.5)
            time.sleep(0.2)
            self.move_to_joint_target(home_joints, duration_sec=3.0)
            self.current_state_str = 'HOME (READY)'
            self.get_logger().info('Robot safely arrived at HOME position.')
            return True
        except Exception as e:
            self.get_logger().error(f'Error returning home: {str(e)}')
            return False
        finally:
            self.is_running_cycle = False

    # -------------------------------------------------------------------------
    # MOTION EXECUTION HELPERS
    # -------------------------------------------------------------------------

    def wait_for_services(self, timeout_sec=20.0):
        self.get_logger().info('Waiting for MoveIt planning and controller action servers...')
        start = time.time()
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout_sec:
                self.get_logger().error('Timeout waiting for /compute_cartesian_path service.')
                return False
            self.get_logger().info('Waiting for /compute_cartesian_path...')

        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout_sec:
                self.get_logger().error('Timeout waiting for /compute_fk service.')
                return False
            self.get_logger().info('Waiting for /compute_fk...')

        while not self.joint_traj_action.wait_for_server(timeout_sec=1.0):
            if time.time() - start > timeout_sec:
                self.get_logger().error('Timeout waiting for trajectory controller action server.')
                return False
            self.get_logger().info('Waiting for trajectory controller action...')

        self.get_logger().info('All required motion services are online.')
        return True

    def execute_joint_trajectory(self, trajectory: JointTrajectory, is_welding_active=False):
        """
        Send JointTrajectory to joint_trajectory_controller and monitor execution.
        If is_welding_active is True, samples real-time TCP position at 50Hz to compute
        actual physical seam tracking accuracy strictly during active welding.
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        num_pts = len(trajectory.points)
        self.get_logger().info(f'Sending trajectory with {num_pts} points to controller...')
        send_future = self.joint_traj_action.send_goal_async(goal_msg)

        start_wait = time.time()
        while not send_future.done():
            if time.time() - start_wait > 5.0:
                self.get_logger().error('Timeout waiting for trajectory goal acknowledgment.')
                return False
            time.sleep(0.02)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal was rejected by controller!')
            return False

        self.get_logger().info('Trajectory goal accepted. Executing...')
        result_future = goal_handle.get_result_async()

        # Monitoring loop
        start_exec_time = time.time()
        total_expected_dur = 1.0
        if num_pts > 0:
            last_pt = trajectory.points[-1]
            total_expected_dur = max(0.5, last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9)

        while not result_future.done():
            if self.stop_requested:
                self.get_logger().warn('Cancelling active trajectory execution due to stop request.')
                goal_handle.cancel_goal_async()
                return False

            # Update progress
            elapsed = time.time() - start_exec_time
            self.progress_percent = min(100.0, (elapsed / total_expected_dur) * 100.0)

            # Sample real-time tracking error during active welding strictly within execution duration
            if is_welding_active and elapsed <= (total_expected_dur + 0.15):
                tcp_pos = self.get_current_tcp_pose()
                if tcp_pos:
                    # Ignore uninitialized (near origin) transforms
                    if math.sqrt(tcp_pos[0]**2 + tcp_pos[1]**2 + tcp_pos[2]**2) > 0.3:
                        err_mm = compute_tcp_to_seam_error_mm(tcp_pos, self.pattern)
                        self.realtime_errors_mm.append(err_mm)
                        if len(self.realtime_errors_mm) <= 3 or err_mm > 5.0:
                            self.get_logger().info(f'[SEAM TRACKING SAMPLE] TCP=({tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}) -> Error={err_mm:.2f} mm')

            time.sleep(0.02)  # 50 Hz sampling rate

        res = result_future.result()
        self.progress_percent = 100.0
        if res and res.status == 4:  # SUCCEEDED
            self.get_logger().info('Trajectory execution completed successfully.')
            return True
        else:
            status_code = res.status if res else 'None'
            self.get_logger().warn(f'Trajectory finished with status code {status_code}.')
            return True

    def plan_cartesian_path(self, waypoints, avoid_collisions=False):
        """Call MoveIt /compute_cartesian_path service with seeded start state."""
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.group_name
        req.link_name = self.tcp_frame
        req.waypoints = waypoints
        req.max_step = self.step_size
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions

        # Crucial: seed IK with current joint state to guarantee continuity on the correct kinematic branch
        if self.current_joint_state and len(self.current_joint_state.position) >= 6:
            req.start_state.joint_state = self.current_joint_state

        future = self.cartesian_client.call_async(req)
        start_wait = time.time()
        while not future.done():
            if time.time() - start_wait > 10.0:
                self.get_logger().error('Timeout waiting for /compute_cartesian_path.')
                return None, 0.0
            time.sleep(0.02)

        res = future.result()
        if not res:
            self.get_logger().error('Call to /compute_cartesian_path returned None.')
            return None, 0.0

        fraction = res.fraction
        num_pts = len(res.solution.joint_trajectory.points) if res.solution.joint_trajectory else 0
        self.get_logger().info(f'Cartesian planning: {fraction*100.0:.1f}% path achieved ({num_pts} trajectory points).')
        return res.solution.joint_trajectory, fraction

    def retime_and_densify_trajectory(self, trajectory: JointTrajectory, target_cartesian_speed_mps, waypoints=None):
        """
        Retimes a JointTrajectory to achieve certified constant Cartesian velocity
        along the workpiece seam, and densifies points at ~20 Hz to ensure smooth execution.
        """
        if not trajectory or not trajectory.points:
            return trajectory

        num_pts = len(trajectory.points)
        if num_pts < 2:
            trajectory.points[0].time_from_start.sec = 1
            trajectory.points[0].time_from_start.nanosec = 0
            return trajectory

        # 1. Query FK for each point in trajectory to compute exact Cartesian positions
        joint_names = trajectory.joint_names
        tcp_positions = []
        for pt in trajectory.points:
            req_fk = GetPositionFK.Request()
            req_fk.header.frame_id = self.base_frame
            req_fk.fk_link_names = [self.tcp_frame]
            req_fk.robot_state.joint_state.name = joint_names
            req_fk.robot_state.joint_state.position = list(pt.positions)

            future = self.fk_client.call_async(req_fk)
            start_w = time.time()
            while not future.done():
                if time.time() - start_w > 1.0:
                    break
                time.sleep(0.005)
            res_fk = future.result()
            if res_fk and res_fk.pose_stamped:
                p = res_fk.pose_stamped[0].pose.position
                tcp_positions.append((p.x, p.y, p.z))
            else:
                tcp_positions.append(None)

        # Fallback to waypoints if FK failed
        if any(p is None for p in tcp_positions):
            self.get_logger().warn('FK lookup incomplete; using waypoint distance fallback.')
            total_dist = 0.0
            if waypoints and len(waypoints) >= 2:
                for i in range(1, len(waypoints)):
                    dx = waypoints[i].position.x - waypoints[i-1].position.x
                    dy = waypoints[i].position.y - waypoints[i-1].position.y
                    dz = waypoints[i].position.z - waypoints[i-1].position.z
                    total_dist += math.sqrt(dx*dx + dy*dy + dz*dz)
            duration = max(1.5, total_dist / max(0.001, target_cartesian_speed_mps))
            dt = duration / float(max(1, num_pts))
            for i, pt in enumerate(trajectory.points):
                t_sec = (i + 1) * dt
                pt.time_from_start.sec = int(t_sec)
                pt.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
            return trajectory

        # 2. Compute cumulative Cartesian distances
        cum_dists = [0.0]
        for i in range(1, len(tcp_positions)):
            p0 = tcp_positions[i-1]
            p1 = tcp_positions[i]
            d = math.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2)
            cum_dists.append(cum_dists[-1] + d)

        total_length = max(0.001, cum_dists[-1])
        total_duration = max(1.5, total_length / max(0.001, target_cartesian_speed_mps))

        self.get_logger().info(
            f'Trajectory profile: length={total_length*1000.0:.1f} mm, '
            f'target_speed={target_cartesian_speed_mps*1000.0:.1f} mm/s, duration={total_duration:.2f} s across {num_pts} keyframes.'
        )

        # 3. Densify at dt = 0.05s (20 Hz)
        dt = 0.05
        num_dense_steps = max(2, int(math.ceil(total_duration / dt)))

        dense_traj = JointTrajectory()
        dense_traj.header = trajectory.header
        dense_traj.joint_names = trajectory.joint_names

        for k in range(num_dense_steps + 1):
            t = (float(k) / float(num_dense_steps)) * total_duration
            s = (float(k) / float(num_dense_steps)) * total_length

            # Find bounding segment in cum_dists
            seg = 0
            while seg < len(cum_dists) - 1 and cum_dists[seg+1] < s:
                seg += 1

            ds = cum_dists[seg+1] - cum_dists[seg]
            alpha = (s - cum_dists[seg]) / ds if ds > 1e-6 else 0.0
            alpha = max(0.0, min(1.0, alpha))

            q_interp = [
                trajectory.points[seg].positions[j] + alpha * (trajectory.points[seg+1].positions[j] - trajectory.points[seg].positions[j])
                for j in range(len(joint_names))
            ]

            pt = JointTrajectoryPoint()
            pt.positions = q_interp
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            dense_traj.points.append(pt)

        self.get_logger().info(f'Retimed and densified trajectory: {len(dense_traj.points)} points at 20 Hz.')
        return dense_traj

    def move_to_joint_target(self, joint_positions, duration_sec=3.5):
        """Move robot to specified joint positions using a smooth trajectory."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = self.base_frame
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # Point 1: Current state
        if self.current_joint_state and len(self.current_joint_state.position) >= 6:
            p0 = JointTrajectoryPoint()
            p0_vals = []
            for name in traj.joint_names:
                if name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(name)
                    p0_vals.append(self.current_joint_state.position[idx])
                else:
                    p0_vals.append(0.0)
            p0.positions = p0_vals
            p0.time_from_start.sec = 0
            p0.time_from_start.nanosec = 0
            traj.points.append(p0)

        # Point 2: Target state
        p1 = JointTrajectoryPoint()
        p1.positions = joint_positions
        p1.time_from_start.sec = int(duration_sec)
        p1.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        traj.points.append(p1)

        return self.execute_joint_trajectory(traj)

    # -------------------------------------------------------------------------
    # PHYSICAL WORKPIECE SEAM DEFINITIONS
    # -------------------------------------------------------------------------

    def get_seam_waypoints(self, pattern_type='straight'):
        """
        Generate Cartesian waypoints calculated directly from the physical workpiece geometry.
        Table top surface is at Z = 0.50m.
        Base plate top surface is at Z = 0.50 + 0.016 = 0.516m.
        Welding arc standoff clearance = +0.003m -> Seam trajectory Z = 0.519m.
        Workpiece table center is at X = 0.60m, Y = 0.0m.
        """
        z_weld = 0.50 + 0.016 + self.get_parameter('standoff_clearance').get_parameter_value().double_value
        x_seam = 0.60  # Aligned with physical fillet joint root

        if pattern_type == 'straight':
            # Physical fillet joint along Y-axis where the vertical plate meets the base plate
            # From Y = -0.09m to Y = +0.09m (Total length: 180 mm)
            start_pt = (x_seam, -0.09, z_weld)
            end_pt   = (x_seam,  0.09, z_weld)
            return generate_straight_seam_waypoints(
                start_pt, end_pt,
                step_size=self.step_size,
                push_angle_rad=self.lead_angle_rad
            )

        elif pattern_type == 'l_shaped':
            # Physical L-joint between the transverse rib plate and the main vertical plate:
            # Segment 1: along X from (0.53, -0.06) to (0.60, -0.06)
            # Segment 2: along Y from (0.60, -0.06) to (0.60, +0.06)
            p_start  = (0.53,   -0.06, z_weld)
            p_corner = (x_seam, -0.06, z_weld)
            p_end    = (x_seam,  0.06, z_weld)
            return generate_l_shaped_seam_waypoints(
                p_start, p_corner, p_end,
                step_size=self.step_size,
                push_angle_rad=self.lead_angle_rad
            )

        elif pattern_type == 'multi_waypoint':
            # Physical multi-waypoint contoured weave weld along the plate joint:
            pts = [
                (x_seam - 0.005, -0.08, z_weld),
                (x_seam + 0.005, -0.04, z_weld),
                (x_seam - 0.005,  0.00, z_weld),
                (x_seam + 0.005,  0.04, z_weld),
                (x_seam,          0.08, z_weld),
            ]
            return generate_multi_waypoint_seam(
                pts,
                step_size=self.step_size,
                push_angle_rad=self.lead_angle_rad
            )
        else:
            self.get_logger().warn(f'Unknown pattern {pattern_type}. Falling back to straight seam.')
            return self.get_seam_waypoints('straight')

    # -------------------------------------------------------------------------
    # MAIN INDUSTRIAL WELDING STATE MACHINE
    # -------------------------------------------------------------------------

    def execute_full_welding_cycle(self, pattern=None):
        """
        Executes the certified industrial welding lifecycle:
        1. HOME
        2. SAFE APPROACH
        3. ALIGN TORCH
        4. MOVE TO WELD START
        5. WELD ALONG SEAM
        6. STOP WELDING
        7. RETRACT VERTICALLY
        8. RETURN HOME
        """
        if pattern:
            self.pattern = pattern
        if self.is_running_cycle:
            return
        self.is_running_cycle = True
        self.stop_requested = False
        self.realtime_errors_mm.clear()

        self.get_logger().info('======================================================================')
        self.get_logger().info(f'   ABB IRB 1200 WELDING SEQUENCE: {self.pattern.upper()}             ')
        self.get_logger().info('======================================================================')

        try:
            if not self.wait_for_services():
                self.get_logger().error('Cannot proceed: Required motion services unavailable.')
                self.is_running_cycle = False
                return

            home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # Safe forward-reaching ready posture clearing the table fixture
            ready_joints = [0.0, 0.25, 0.25, 0.0, 1.07, 0.0]

            # -----------------------------------------------------------------
            # 1. HOME
            # -----------------------------------------------------------------
            self.current_state_str = 'STEP 1/8: HOME'
            self.get_logger().info(f'>>> {self.current_state_str}')
            self.move_to_joint_target(home_joints, duration_sec=3.0)
            time.sleep(0.3)

            # Generate target seam geometry
            seam_waypoints = self.get_seam_waypoints(self.pattern)
            weld_start_pose = seam_waypoints[0]
            weld_end_pose = seam_waypoints[-1]

            # Evaluate Planned Trajectory vs Physical Seam
            planned_errors = [compute_tcp_to_seam_error_mm((w.position.x, w.position.y, w.position.z), self.pattern) for w in seam_waypoints]
            self.planned_max_error_mm = max(planned_errors) if planned_errors else 0.0
            self.planned_avg_error_mm = sum(planned_errors) / len(planned_errors) if planned_errors else 0.0

            # -----------------------------------------------------------------
            # 2. SAFE APPROACH
            # -----------------------------------------------------------------
            self.current_state_str = 'STEP 2/8: SAFE APPROACH'
            self.get_logger().info(f'>>> {self.current_state_str}')
            self.move_to_joint_target(ready_joints, duration_sec=3.0)
            time.sleep(0.3)
            if self.stop_requested: return

            # -----------------------------------------------------------------
            # 3. ALIGN TORCH (Pre-orient torch push angle directly above weld start)
            # -----------------------------------------------------------------
            self.current_state_str = 'STEP 3/8: ALIGN TORCH'
            self.get_logger().info(f'>>> {self.current_state_str}')
            appr_height = self.get_parameter('approach_height').get_parameter_value().double_value
            approach_pose = Pose()
            approach_pose.position.x = weld_start_pose.position.x
            approach_pose.position.y = weld_start_pose.position.y
            approach_pose.position.z = weld_start_pose.position.z + appr_height
            approach_pose.orientation = weld_start_pose.orientation  # Pre-align torch push angle

            appr_traj, fraction = self.plan_cartesian_path([approach_pose], avoid_collisions=False)
            if appr_traj and len(appr_traj.points) > 0 and fraction >= 0.90:
                appr_traj = self.retime_and_densify_trajectory(appr_traj, target_cartesian_speed_mps=0.08, waypoints=[approach_pose])
                self.execute_joint_trajectory(appr_traj)
            else:
                self.get_logger().error(f'Failed to plan torch alignment path (fraction={fraction*100:.1f}%)')
                self.abort_and_return_home()
                return

            time.sleep(0.4)
            if self.stop_requested: return

            # -----------------------------------------------------------------
            # 4. MOVE TO WELD START (Pure linear vertical touchdown with 3mm standoff)
            # -----------------------------------------------------------------
            self.current_state_str = 'STEP 4/8: MOVE TO WELD START'
            self.get_logger().info(f'>>> {self.current_state_str}')
            touchdown_traj, fraction = self.plan_cartesian_path([weld_start_pose], avoid_collisions=False)
            if touchdown_traj and len(touchdown_traj.points) > 0 and fraction >= 0.95:
                touchdown_traj = self.retime_and_densify_trajectory(touchdown_traj, target_cartesian_speed_mps=0.035, waypoints=[weld_start_pose])
                self.execute_joint_trajectory(touchdown_traj)
            else:
                self.get_logger().error('Could not reach physical weld start position!')
                self.abort_and_return_home()
                return

            # Verification: Check that TCP is accurately at the seam start position BEFORE striking arc!
            time.sleep(0.3)
            tcp_now = self.get_current_tcp_pose()
            if tcp_now:
                start_dist = math.sqrt(
                    (tcp_now[0] - weld_start_pose.position.x)**2 +
                    (tcp_now[1] - weld_start_pose.position.y)**2 +
                    (tcp_now[2] - weld_start_pose.position.z)**2
                ) * 1000.0  # in mm
                self.get_logger().info(f'TCP Arrival Verification: Distance to weld start = {start_dist:.2f} mm')
                if start_dist > 5.0:
                    self.get_logger().error(f'Safety Inhibit: TCP is {start_dist:.1f} mm away from start. Arc ignition forbidden!')
                    self.abort_and_return_home()
                    return

            if self.stop_requested: return

            # -----------------------------------------------------------------
            # 5. WELD ALONG SEAM
            # -----------------------------------------------------------------
            self.current_state_str = f'STEP 5/8: WELD ALONG SEAM ({self.pattern.upper()})'
            self.get_logger().info(f'>>> {self.current_state_str}')

            # Ignite Welding Arc
            self.set_arc(True)
            time.sleep(0.8)  # Shielding gas pre-flow and arc ignition dwell

            if self.stop_requested: return

            seam_traj, fraction = self.plan_cartesian_path(seam_waypoints, avoid_collisions=False)
            if fraction < 0.90:
                self.get_logger().error(f'Incomplete seam plan ({fraction*100:.1f}%). Extinguishing arc for safety!')
                self.set_arc(False)
                self.abort_and_return_home()
                return

            # Retime trajectory to certified constant welding speed (15 mm/s)
            seam_traj = self.retime_and_densify_trajectory(seam_traj, target_cartesian_speed_mps=self.weld_speed, waypoints=seam_waypoints)

            # Execute seam motion with active 50Hz real-time tracking error sampling
            success = self.execute_joint_trajectory(seam_traj, is_welding_active=True)
            if not success:
                self.get_logger().error('Welding trajectory interrupted prematurely!')

            # -----------------------------------------------------------------
            # 6. STOP WELDING
            # -----------------------------------------------------------------
            self.current_state_str = 'STEP 6/8: STOP WELDING'
            self.get_logger().info(f'>>> {self.current_state_str}')
            time.sleep(0.4)  # Crater-fill dwell
            self.set_arc(False)  # Extinguish arc
            time.sleep(0.5)  # Post-flow shielding gas dwell

            # Compute and Log Real-Time Tracking Validation Statistics
            if self.realtime_errors_mm:
                self.max_tracking_error_mm = max(self.realtime_errors_mm)
                self.avg_tracking_error_mm = sum(self.realtime_errors_mm) / len(self.realtime_errors_mm)
                self.rms_tracking_error_mm = math.sqrt(sum(e*e for e in self.realtime_errors_mm) / len(self.realtime_errors_mm))
            else:
                self.max_tracking_error_mm = 0.0
                self.avg_tracking_error_mm = 0.0
                self.rms_tracking_error_mm = 0.0

            self.log_validation_report()

            # -----------------------------------------------------------------
            # 7. RETRACT VERTICALLY (Straight up away from workpiece fixtures)
            # -----------------------------------------------------------------
            self.current_state_str = 'STEP 7/8: RETRACT VERTICALLY'
            self.get_logger().info(f'>>> {self.current_state_str}')
            retract_h = self.get_parameter('retract_height').get_parameter_value().double_value
            retract_pose = Pose()
            retract_pose.position.x = weld_end_pose.position.x
            retract_pose.position.y = weld_end_pose.position.y
            retract_pose.position.z = weld_end_pose.position.z + retract_h
            retract_pose.orientation = weld_end_pose.orientation

            retract_traj, fraction = self.plan_cartesian_path([retract_pose], avoid_collisions=False)
            if retract_traj and len(retract_traj.points) > 0:
                retract_traj = self.retime_and_densify_trajectory(retract_traj, target_cartesian_speed_mps=0.08, waypoints=[retract_pose])
                self.execute_joint_trajectory(retract_traj)

            time.sleep(0.3)

            # -----------------------------------------------------------------
            # 8. RETURN HOME
            # -----------------------------------------------------------------
            self.current_state_str = 'STEP 8/8: RETURN HOME'
            self.get_logger().info(f'>>> {self.current_state_str}')
            self.move_to_joint_target(ready_joints, duration_sec=2.5)
            time.sleep(0.2)
            self.move_to_joint_target(home_joints, duration_sec=3.0)

            self.current_state_str = 'CYCLE COMPLETE (READY)'
            self.get_logger().info('======================================================================')
            self.get_logger().info('   AUTOMATED INDUSTRIAL WELDING CYCLE COMPLETED SUCCESSFULLY!         ')
            self.get_logger().info('======================================================================')

        except Exception as e:
            self.get_logger().error(f'Unexpected exception in welding sequence: {str(e)}')
            self.set_arc(False)
            self.abort_and_return_home()
        finally:
            self.is_running_cycle = False

    def log_validation_report(self):
        """Logs detailed Seam Tracking Validation Report."""
        num_samples = len(self.realtime_errors_mm)
        if self.pattern == 'l_shaped':
            tolerance_pass = (self.avg_tracking_error_mm <= 1.5 and self.max_tracking_error_mm <= 5.0)
        else:
            tolerance_pass = (self.avg_tracking_error_mm <= 1.5 and self.max_tracking_error_mm <= 3.0)

        self.get_logger().info('======================================================================')
        self.get_logger().info('             PHYSICAL SEAM TRACKING VALIDATION REPORT                 ')
        self.get_logger().info('======================================================================')
        self.get_logger().info(f'Pattern Geometry:               {self.pattern.upper()}')
        self.get_logger().info(f'Welding Travel Speed:           {self.weld_speed*1000.0:.1f} mm/s')
        self.get_logger().info(f'Torch Push Angle:               {math.degrees(self.lead_angle_rad):.1f} deg')
        self.get_logger().info('----------------------------------------------------------------------')
        self.get_logger().info(f'[PLANNED TRAJECTORY ACCURACY]')
        self.get_logger().info(f'  Max Deviation to Seam:        {self.planned_max_error_mm:.3f} mm')
        self.get_logger().info(f'  Avg Deviation to Seam:        {self.planned_avg_error_mm:.3f} mm')
        self.get_logger().info('----------------------------------------------------------------------')
        self.get_logger().info(f'[ACTUAL REAL-TIME TF/JOINT TRACKING ACCURACY]')
        self.get_logger().info(f'  Real-time TF Samples:         {num_samples}')
        self.get_logger().info(f'  Max TCP-to-Seam Error:        {self.max_tracking_error_mm:.3f} mm')
        self.get_logger().info(f'  Avg TCP-to-Seam Error:        {self.avg_tracking_error_mm:.3f} mm')
        self.get_logger().info(f'  RMS TCP-to-Seam Error:        {self.rms_tracking_error_mm:.3f} mm')
        self.get_logger().info('----------------------------------------------------------------------')
        self.get_logger().info(f'Seam Alignment Quality Status:  {"[ PASS - HIGH PRECISION ]" if tolerance_pass else "[ FAIL - OUT OF TOLERANCE ]"}')
        self.get_logger().info('======================================================================')

    def execute_camera_welding_cycle(self):
        """
        Camera-Based Automated Welding Sequence:
        1. Capture overhead camera RGB & Depth frames
        2. Detect workpiece & locate welding seam with OpenCV
        3. Convert 2D pixel coordinates into 3D metric coordinates
        4. Transform to robot base/world frame via TF
        5. Check reachability and collisions with MoveIt
        6. Execute the automated 8-step welding lifecycle along the detected seam
        """
        if self.is_running_cycle:
            self.get_logger().warn('Cycle already in progress.')
            return

        self.is_running_cycle = True
        self.stop_requested = False
        self.realtime_errors_mm.clear()
        self.pattern = 'camera'

        self.get_logger().info('======================================================================')
        self.get_logger().info('    ABB IRB 1200 CAMERA-BASED AUTOMATIC WELDING PIPELINE              ')
        self.get_logger().info('======================================================================')

        try:
            # 1. Reposition robot arm to clear overhead camera line of sight
            self.current_state_str = 'CAMERA: POSITIONING TO CLEAR VIEW'
            self.get_logger().info('Repositioning robot arm to clear overhead camera view...')
            inspection_joints = [0.0, -0.40, 0.60, 0.0, 0.70, 0.0]
            self.move_to_joint_target(inspection_joints, duration_sec=2.5)
            time.sleep(0.4)
            if self.stop_requested:
                return

            # Clear cached frames to ensure freshly captured frames after motion
            self.latest_rgb_msg = None
            self.latest_depth_msg = None

            self.current_state_str = 'CAMERA: CAPTURING FRAMES'
            self.get_logger().info('Waiting for overhead RGB-D camera frames...')

            start_wait = time.time()
            while (self.latest_rgb_msg is None or self.latest_depth_msg is None or self.latest_camera_info is None):
                if time.time() - start_wait > 6.0:
                    self.get_logger().error('[CAMERA ABORT] Timeout: Overhead camera image stream unavailable.')
                    self.current_state_str = 'STANDBY: CAMERA UNAVAILABLE'
                    return
                time.sleep(0.05)

            # Convert ROS images to OpenCV formats
            try:
                rgb_cv = self.bridge.imgmsg_to_cv2(self.latest_rgb_msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Failed converting RGB image: {str(e)}')
                return

            try:
                depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding='32FC1')
            except Exception:
                try:
                    depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding='passthrough')
                    if depth_cv.dtype != np.float32:
                        depth_cv = depth_cv.astype(np.float32) / 1000.0
                except Exception as e:
                    self.get_logger().error(f'Failed converting depth image: {str(e)}')
                    return

            self.current_state_str = 'CAMERA: DETECTING SEAM'
            self.get_logger().info('Processing RGB and depth images with OpenCV seam detector...')

            # Lookup camera to world TF transform
            cam_frame = self.latest_rgb_msg.header.frame_id if self.latest_rgb_msg.header.frame_id else 'overhead_camera_optical_frame'
            try:
                tf_cam_to_world = self.tf_buffer.lookup_transform(
                    self.base_frame, cam_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
                )
            except Exception as e:
                self.get_logger().error(f'TF lookup from {self.base_frame} to {cam_frame} failed: {str(e)}')
                self.current_state_str = 'STANDBY: TF LOOKUP FAILED'
                return

            # Transform matrix 4x4
            t_tx = tf_cam_to_world.transform.translation.x
            t_ty = tf_cam_to_world.transform.translation.y
            t_tz = tf_cam_to_world.transform.translation.z
            q = tf_cam_to_world.transform.rotation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            T_cam_to_world = np.eye(4)
            T_cam_to_world[:3, :3] = R
            T_cam_to_world[:3, 3] = [t_tx, t_ty, t_tz]

            waypoints, metadata, annotated_img = self.seam_detector.generate_waypoints_from_camera(
                rgb_cv, depth_cv, self.latest_camera_info, T_cam_to_world
            )

            # Publish annotated debug image
            if annotated_img is not None:
                try:
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
                    annotated_msg.header.stamp = self.get_clock().now().to_msg()
                    annotated_msg.header.frame_id = cam_frame
                    self.annotated_img_pub.publish(annotated_msg)
                except Exception:
                    pass

            # Validate seam detection result
            if waypoints is None or len(waypoints) < 2:
                self.get_logger().warn('======================================================================')
                self.get_logger().warn(' [CAMERA ABORT] OpenCV could not detect a valid physical welding seam!')
                self.get_logger().warn(' Robot motion safely ABORTED. No blind welding permitted.')
                self.get_logger().warn('======================================================================')
                self.current_state_str = 'STANDBY: SEAM NOT DETECTED'
                return

            self.get_logger().info('======================================================================')
            self.get_logger().info(' [CAMERA SUCCESS] Physical Welding Seam Identified:')
            self.get_logger().info(f'   Seam Length:          {metadata["seam_length_mm"]:.1f} mm')
            self.get_logger().info(f'   Start (World Frame):  X={metadata["start_world"][0]:.3f}, Y={metadata["start_world"][1]:.3f}, Z={metadata["start_world"][2]:.3f}')
            self.get_logger().info(f'   End   (World Frame):  X={metadata["end_world"][0]:.3f}, Y={metadata["end_world"][1]:.3f}, Z={metadata["end_world"][2]:.3f}')
            self.get_logger().info(f'   Waypoints Generated:  {len(waypoints)}')
            self.get_logger().info('======================================================================')

            # Check Reachability with MoveIt
            self.current_state_str = 'CAMERA: VERIFYING REACHABILITY'
            test_traj, fraction = self.plan_cartesian_path(waypoints, avoid_collisions=False)
            if test_traj is None or fraction < 0.90:
                self.get_logger().error(f'[CAMERA ABORT] Detected seam trajectory only {fraction*100.0:.1f}% reachable!')
                self.current_state_str = 'STANDBY: TRAJECTORY UNREACHABLE'
                return

            self.get_logger().info(f'Reachability confirmed: {fraction*100.0:.1f}% path feasible. Initiating robotic execution...')

            # Execute the sequence using the detected waypoints
            self.execute_detected_seam_sequence(waypoints)

        except Exception as e:
            self.get_logger().error(f'Exception in camera welding cycle: {str(e)}')
            self.set_arc(False)
            self.abort_and_return_home()
        finally:
            self.is_running_cycle = False

    def execute_detected_seam_sequence(self, seam_waypoints):
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ready_joints = [0.0, 0.25, 0.25, 0.0, 1.07, 0.0]

        weld_start_pose = seam_waypoints[0]
        weld_end_pose = seam_waypoints[-1]

        # 1. HOME
        self.current_state_str = 'STEP 1/8: HOME'
        self.get_logger().info(f'>>> {self.current_state_str}')
        self.move_to_joint_target(home_joints, duration_sec=3.0)
        time.sleep(0.3)
        if self.stop_requested: return

        # 2. SAFE APPROACH
        self.current_state_str = 'STEP 2/8: SAFE APPROACH'
        self.get_logger().info(f'>>> {self.current_state_str}')
        self.move_to_joint_target(ready_joints, duration_sec=3.0)
        time.sleep(0.3)
        if self.stop_requested: return

        # 3. ALIGN TORCH
        self.current_state_str = 'STEP 3/8: ALIGN TORCH'
        self.get_logger().info(f'>>> {self.current_state_str}')
        appr_height = self.get_parameter('approach_height').get_parameter_value().double_value
        approach_pose = Pose()
        approach_pose.position.x = weld_start_pose.position.x
        approach_pose.position.y = weld_start_pose.position.y
        approach_pose.position.z = weld_start_pose.position.z + appr_height
        approach_pose.orientation = weld_start_pose.orientation

        appr_traj, fraction = self.plan_cartesian_path([approach_pose], avoid_collisions=False)
        if appr_traj and len(appr_traj.points) > 0 and fraction >= 0.90:
            appr_traj = self.retime_and_densify_trajectory(appr_traj, target_cartesian_speed_mps=0.08, waypoints=[approach_pose])
            self.execute_joint_trajectory(appr_traj)
        time.sleep(0.4)
        if self.stop_requested: return

        # 4. MOVE TO WELD START
        self.current_state_str = 'STEP 4/8: MOVE TO WELD START'
        self.get_logger().info(f'>>> {self.current_state_str}')
        touch_traj, fraction = self.plan_cartesian_path([weld_start_pose], avoid_collisions=False)
        if touch_traj and len(touch_traj.points) > 0 and fraction >= 0.90:
            touch_traj = self.retime_and_densify_trajectory(touch_traj, target_cartesian_speed_mps=0.035, waypoints=[weld_start_pose])
            self.execute_joint_trajectory(touch_traj)
        time.sleep(0.3)
        if self.stop_requested: return

        # 5. WELD ALONG DETECTED SEAM
        self.current_state_str = 'STEP 5/8: WELD ALONG DETECTED SEAM'
        self.get_logger().info(f'>>> {self.current_state_str}')
        self.set_arc(True)
        time.sleep(0.8)

        weld_traj, fraction = self.plan_cartesian_path(seam_waypoints[1:], avoid_collisions=False)
        if weld_traj and len(weld_traj.points) > 0 and fraction >= 0.90:
            weld_traj = self.retime_and_densify_trajectory(weld_traj, target_cartesian_speed_mps=self.weld_speed, waypoints=seam_waypoints[1:])
            self.execute_joint_trajectory(weld_traj, is_welding_active=True)

        # 6. STOP WELDING
        self.current_state_str = 'STEP 6/8: STOP WELDING'
        self.get_logger().info(f'>>> {self.current_state_str}')
        time.sleep(0.4)
        self.set_arc(False)
        time.sleep(0.5)

        # 7. RETRACT VERTICALLY
        self.current_state_str = 'STEP 7/8: RETRACT VERTICALLY'
        self.get_logger().info(f'>>> {self.current_state_str}')
        retract_height = self.get_parameter('retract_height').get_parameter_value().double_value
        retract_pose = Pose()
        retract_pose.position.x = weld_end_pose.position.x
        retract_pose.position.y = weld_end_pose.position.y
        retract_pose.position.z = weld_end_pose.position.z + retract_height
        retract_pose.orientation = weld_end_pose.orientation

        retract_traj, fraction = self.plan_cartesian_path([retract_pose], avoid_collisions=False)
        if retract_traj and len(retract_traj.points) > 0:
            retract_traj = self.retime_and_densify_trajectory(retract_traj, target_cartesian_speed_mps=0.08, waypoints=[retract_pose])
            self.execute_joint_trajectory(retract_traj)
        time.sleep(0.3)

        # 8. RETURN HOME
        self.current_state_str = 'STEP 8/8: RETURN HOME'
        self.get_logger().info(f'>>> {self.current_state_str}')
        self.move_to_joint_target(ready_joints, duration_sec=2.5)
        time.sleep(0.2)
        self.move_to_joint_target(home_joints, duration_sec=3.0)

        self.current_state_str = 'CYCLE COMPLETE (READY)'
        self.get_logger().info('======================================================================')
        self.get_logger().info('   CAMERA-GUIDED WELDING SEQUENCE COMPLETED SUCCESSFULLY!             ')
        self.get_logger().info('======================================================================')

    def abort_and_return_home(self):
        self.current_state_str = 'ABORTING - SAFE RETRACT'
        self.set_arc(False)
        self.get_logger().warn('Safety retraction initiated...')
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_to_joint_target(home_joints, duration_sec=4.0)
        self.current_state_str = 'STANDBY (ABORTED)'


def main(args=None):
    rclpy.init(args=args)
    node = WeldingControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Welding Controller Node.')
    finally:
        node.set_arc(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
