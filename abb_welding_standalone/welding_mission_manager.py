#!/usr/bin/env python3
"""
Welding Mission Manager CLI.
Allows user to trigger or configure welding operations:
Usage:
  ros2 run abb_welding_standalone welding_mission_manager --pattern straight
  ros2 run abb_welding_standalone welding_mission_manager --pattern l_shaped
  ros2 run abb_welding_standalone welding_mission_manager --pattern multi_waypoint
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


class WeldingMissionManager(Node):
    def __init__(self, pattern=None):
        super().__init__('welding_mission_manager')
        self.trigger_client = self.create_client(Trigger, '/welding/start_cycle')
        self.param_client = self.create_client(SetParameters, '/welding_controller/set_parameters')

    def set_pattern_and_trigger(self, pattern):
        self.get_logger().info(f'Configuring welding pattern: {pattern}')
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Parameter service not ready, continuing...')
        else:
            param = Parameter()
            param.name = 'default_pattern'
            param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=pattern)
            req = SetParameters.Request()
            req.parameters = [param]
            future = self.param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('Sending Trigger to /welding/start_cycle...')
        if not self.trigger_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/welding/start_cycle service is not available.')
            return False

        req = Trigger.Request()
        future = self.trigger_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.success:
            self.get_logger().info(f'Success: {res.message}')
            return True
        else:
            msg = res.message if res else 'Unknown failure'
            self.get_logger().error(f'Failed to start cycle: {msg}')
            return False


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='ABB IRB 1200 Welding Mission Manager')
    parser.add_argument('--pattern', choices=['straight', 'l_shaped', 'multi_waypoint'], default='straight',
                        help='Welding seam geometry pattern')
    parsed_args, _ = parser.parse_known_args()

    node = WeldingMissionManager()
    node.set_pattern_and_trigger(parsed_args.pattern)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

