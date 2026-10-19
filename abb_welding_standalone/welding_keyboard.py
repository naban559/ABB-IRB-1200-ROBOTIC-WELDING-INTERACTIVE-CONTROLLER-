#!/usr/bin/env python3
"""
Interactive Terminal Keyboard Controller for ABB IRB 1200 Welding Simulation.
Enables real-time operator selection of welding patterns, safe return-home,
instant arc extinction, and live telemetry status display.

Controls:
  S = Straight-line welding
  L = L-shaped welding
  M = Multi-waypoint welding
  C = Camera-based automatic welding (RGB-D vision guided)
  H = Return home safely (only when not actively welding)
  X = Stop current operation / Extinguish arc instantly
  Q = Quit keyboard controller
"""

import os
import sys
import time
import json
import select
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class WeldingKeyboardController(Node):
    def __init__(self):
        super().__init__("welding_keyboard_controller")

        self.cb_group = ReentrantCallbackGroup()

        # Telemetry State
        self.connected = False
        self.last_status_time = 0.0
        self.state_str = "STANDBY / CONNECTING..."
        self.active_pattern = "NONE"
        self.arc_active = False
        self.progress_percent = 0.0
        self.curr_error_mm = 0.0
        self.max_error_mm = 0.0
        self.avg_error_mm = 0.0
        self.is_running_operation = False
        self.last_message = "Ready. Press a key to trigger an industrial welding operation."

        # Subscriber for live status from welding_controller
        self.status_sub = self.create_subscription(
            String,
            "/welding/status",
            self.status_callback,
            10,
            callback_group=self.cb_group,
        )

        # Service Clients
        self.srv_straight = self.create_client(Trigger, "/welding/start_straight", callback_group=self.cb_group)
        self.srv_l_shaped = self.create_client(Trigger, "/welding/start_l_shaped", callback_group=self.cb_group)
        self.srv_multi = self.create_client(Trigger, "/welding/start_multi_waypoint", callback_group=self.cb_group)
        self.srv_camera = self.create_client(Trigger, "/welding/start_camera", callback_group=self.cb_group)
        self.srv_home = self.create_client(Trigger, "/welding/return_home", callback_group=self.cb_group)
        self.srv_stop = self.create_client(Trigger, "/welding/stop_cycle", callback_group=self.cb_group)

        # Command topic publisher as secondary broadcast channel
        self.cmd_pub = self.create_publisher(String, "/welding/command", 10)

        self.running = True

    def status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.connected = True
            self.last_status_time = time.time()
            self.state_str = data.get("state", "UNKNOWN")
            self.active_pattern = data.get("pattern", "UNKNOWN").upper()
            self.arc_active = data.get("arc_active", False)
            self.progress_percent = float(data.get("progress_percent", 0.0))
            self.curr_error_mm = float(data.get("current_tracking_error_mm", 0.0))
            self.max_error_mm = float(data.get("max_tracking_error_mm", 0.0))
            self.avg_error_mm = float(data.get("avg_tracking_error_mm", 0.0))

            # Determine whether an operation is active
            active_keywords = ["STEP", "WELD", "APPROACH", "ALIGN", "RETRACT", "RETURNING"]
            self.is_running_operation = any(k in self.state_str for k in active_keywords)
        except Exception:
            pass

    def call_service_async(self, client, name, op_desc):
        if not client.service_is_ready():
            # Check availability with short timeout
            if not client.wait_for_service(timeout_sec=1.0):
                self.last_message = f"[ERROR] Service {name} not available. Is welding_controller running?"
                return

        req = Trigger.Request()
        future = client.call_async(req)

        def done_cb(fut):
            try:
                res = fut.result()
                if res.success:
                    self.last_message = f"[SUCCESS] {res.message}"
                else:
                    self.last_message = f"[REJECTED] {res.message}"
            except Exception as e:
                self.last_message = f"[ERROR] Failed calling {op_desc}: {str(e)}"

        future.add_done_callback(done_cb)

    def trigger_straight(self):
        if self.is_running_operation:
            self.last_message = f"[BLOCKED] Robot is busy: {self.state_str}. Stop (X) first before new command."
            return
        self.last_message = "Dispatching Straight-Line Welding command..."
        self.call_service_async(self.srv_straight, "/welding/start_straight", "Straight Welding")

    def trigger_l_shaped(self):
        if self.is_running_operation:
            self.last_message = f"[BLOCKED] Robot is busy: {self.state_str}. Stop (X) first before new command."
            return
        self.last_message = "Dispatching L-Shaped Fillet Welding command..."
        self.call_service_async(self.srv_l_shaped, "/welding/start_l_shaped", "L-Shaped Welding")

    def trigger_multi_waypoint(self):
        if self.is_running_operation:
            self.last_message = f"[BLOCKED] Robot is busy: {self.state_str}. Stop (X) first before new command."
            return
        self.last_message = "Dispatching Multi-Waypoint Seam Welding command..."
        self.call_service_async(self.srv_multi, "/welding/start_multi_waypoint", "Multi-Waypoint Welding")

    def trigger_camera(self):
        if self.is_running_operation:
            self.last_message = f"[BLOCKED] Robot is busy: {self.state_str}. Stop (X) first before new command."
            return
        self.last_message = "Dispatching Camera-Based Seam Detection and Welding command..."
        self.call_service_async(self.srv_camera, "/welding/start_camera", "Camera Welding")

    def trigger_return_home(self):
        if self.is_running_operation and "WELD" in self.state_str:
            self.last_message = "[SAFETY REJECTION] 'H' is not an emergency stop! Press 'X' first to extinguish arc and stop motion."
            return
        self.last_message = "Executing safe Return Home..."
        self.call_service_async(self.srv_home, "/welding/return_home", "Return Home")

    def trigger_stop(self):
        self.last_message = "[STOPPING] Emergency stop signal sent. Extinguishing arc immediately..."
        self.call_service_async(self.srv_stop, "/welding/stop_cycle", "Stop Operation")

    def format_progress_bar(self, percent, width=28):
        filled = int(width * percent / 100.0)
        bar = "█" * filled + "░" * (width - filled)
        return f"[{bar}] {percent:5.1f}%"

    def render_dashboard(self):
        # Check connection staleness
        is_alive = self.connected and (time.time() - self.last_status_time < 3.0)
        conn_str = "\033[92mCONNECTED [Gazebo Sim + MoveIt 2 Online]\033[0m" if is_alive else "\033[91mWAITING FOR WELDING SYSTEM...\033[0m"

        arc_str = "\033[93;1m[ ON - ELECTRIC ARC STRUCK ]\033[0m" if self.arc_active else "\033[90m[ OFF - EXTINCTION ]\033[0m"
        state_color = "\033[96;1m" if self.is_running_operation else "\033[92m"

        # ANSI clear screen and home cursor
        output = [
            "\033[2J\033[H",
            "\033[1;37;44m ====================================================================== \033[0m",
            "\033[1;37;44m        ABB IRB 1200 ROBOTIC WELDING INTERACTIVE CONTROLLER             \033[0m",
            "\033[1;37;44m ====================================================================== \033[0m",
            f"  System Link:        {conn_str}",
            f"  System State:       {state_color}{self.state_str}\033[0m",
            f"  Active Pattern:     \033[1m{self.active_pattern}\033[0m",
            f"  Welding Arc:        {arc_str}",
            f"  Process Progress:   {self.format_progress_bar(self.progress_percent)}",
            f"  Seam Accuracy:      Current: {self.curr_error_mm:.2f} mm | Max: {self.max_error_mm:.2f} mm | Avg: {self.avg_error_mm:.2f} mm",
            " ---------------------------------------------------------------------- ",
            f"  Last Response:      \033[1;33m{self.last_message}\033[0m",
            " ====================================================================== ",
            "  KEYBOARD CONTROLS:                                                   ",
            "    \033[1m[S]\033[0m  Straight-Line Seam Welding                                   ",
            "    \033[1m[L]\033[0m  L-Shaped Fillet Welding                                      ",
            "    \033[1m[M]\033[0m  Multi-Waypoint Contoured Seam Welding                        ",
            "    \033[1m[C]\033[0m  Camera-Based Automatic Welding (Vision-Guided RGB-D)         ",
            "    \033[1m[H]\033[0m  Return Home Safely (executed only when idle / safe)          ",
            "    \033[1;31m[X]\033[0m  Stop Active Operation / Extinguish Arc Immediately           ",
            "    \033[1m[Q]\033[0m  Quit Keyboard Controller                                     ",
            " ====================================================================== ",
            "  Press key [S / L / M / C / H / X / Q]: ",
        ]
        sys.stdout.write("\n".join(output))
        sys.stdout.flush()


def run_keyboard_loop(node: WeldingKeyboardController):
    # Set stdin to raw/cbreak mode
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last_render = 0.0

        while node.running and rclpy.ok():
            now = time.time()
            if now - last_render >= 0.15:  # ~7 Hz terminal UI refresh
                node.render_dashboard()
                last_render = now

            # Poll stdin for keypress (timeout 0.05s)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                ch = sys.stdin.read(1)
                if not ch:
                    continue

                ch_lower = ch.lower()
                if ch_lower == "s":
                    node.trigger_straight()
                elif ch_lower == "l":
                    node.trigger_l_shaped()
                elif ch_lower == "m":
                    node.trigger_multi_waypoint()
                elif ch_lower == "c":
                    node.trigger_camera()
                elif ch_lower == "h":
                    node.trigger_return_home()
                elif ch_lower == "x":
                    node.trigger_stop()
                elif ch_lower == "q" or ord(ch) == 3:  # 'q' or Ctrl+C
                    node.last_message = "Exiting Keyboard Controller..."
                    node.running = False
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nKeyboard controller closed gracefully.")


def main(args=None):
    rclpy.init(args=args)
    node = WeldingKeyboardController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        run_keyboard_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

