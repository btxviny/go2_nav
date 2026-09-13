#!/usr/bin/env python3
"""Send a single NavigateToPose goal to Nav2 from the command line.

`ros2 action` isn't installed in this project's setup (see README -- same as
`ros2 run`/`ros2 lifecycle`), so `ros2 action send_goal` doesn't work here.
This is the reusable replacement: a plain rclpy action client, same pattern
verified working during the KISS-ICP/Nav2 integration testing (drove the
robot to within 2cm of a goal).

Usage:
    python3 send_nav_goal.py --x 1.0 --y 0.0 [--yaw 0.0]

Goal is in the `map` frame. Prints feedback (distance remaining, recoveries)
as it drives, then the final result.
"""
import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True, help='Goal x position (map frame)')
    parser.add_argument('--y', type=float, required=True, help='Goal y position (map frame)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw in radians (default 0)')
    parser.add_argument('--timeout', type=float, default=120.0,
                         help='Seconds to wait for a result before giving up (default 120)')
    args = parser.parse_args()

    rclpy.init()
    node = Node('send_nav_goal', namespace='/robot1')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    node.get_logger().info('Waiting for navigate_to_pose action server...')
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error('Action server not available -- is nav_stack.launch.py up?')
        sys.exit(1)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

    def feedback_cb(fb):
        f = fb.feedback
        node.get_logger().info(
            f'distance_remaining={f.distance_remaining:.2f}m '
            f'nav_time={f.navigation_time.sec}s recoveries={f.number_of_recoveries}')

    node.get_logger().info(f'Sending goal: x={args.x} y={args.y} yaw={args.yaw}')
    send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error('Goal rejected or send timed out')
        sys.exit(1)

    node.get_logger().info('Goal accepted, waiting for result...')
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
    result = result_future.result()
    if result is None:
        node.get_logger().error(f'Timed out waiting for result after {args.timeout}s')
        sys.exit(1)

    # status values: nav2_msgs GoalStatus -- 4 == STATUS_SUCCEEDED
    status_names = {2: 'EXECUTING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
    status = status_names.get(result.status, str(result.status))
    node.get_logger().info(f'Result: {status}')
    rclpy.shutdown()
    sys.exit(0 if result.status == 4 else 1)


if __name__ == '__main__':
    main()
