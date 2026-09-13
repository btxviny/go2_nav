#!/usr/bin/env python3
"""WASD + arrow-key teleop for the Go2.

  W/S         strafe forward/back     (linear.x)
  A/D         strafe left/right       (linear.y)
  Left/Right  yaw left/right          (angular.z)
  Up/Down     stand taller / crouch   (linear.z -> stance height)
  Space       stop
  q / Ctrl-C  quit

Up/Down deliberately do NOT command body pitch (angular.y), even though the
RobotVelocity message has a field for it. In the upstream IK trot controller
(quadropted_controller/RobotController/{TrotGaitController,RobotController}.py)
angular.x/y are wired only for small automatic IMU-stabilization feedback, not
user input -- TrotStanceController.position_delta() integrates them straight
into stance foot position every tick with no spring-back to zero, so a held
pitch command drives continuous, non-recovering foot drift that looks exactly
like forward/backward walking (this was the actual cause of a "up/down move
like w/s" report). linear.z was otherwise dead downstream, so it's been
repurposed there as a real, working stance-height adjust instead.

Must run in an interactive terminal with keyboard focus -- reads raw keys off
stdin, so it can't be driven through a non-interactive tool call.

Publishes to /robot1/cmd_vel, matching what cmd_vel_pub.py (quadropted_controller,
upstream go2_ros2_sim_py) subscribes to. Values are normalized to [-1, 1]; the
downstream node applies its own per-axis scaling (see cmd_vel_pub.py's
multiply_and_limit calls) so these should stay in that range like any other
Twist-based teleop, not already-scaled physical units.

Auto-stops if no key is pressed for WATCHDOG_S seconds, since raw terminal
input has no key-release event to publish a real zero on release.
"""
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

WATCHDOG_S = 0.4
RATE_HZ = 20

HELP = """
Go2 keyboard teleop
  W/S         strafe forward / back
  A/D         strafe left / right
  Left/Right  yaw left / right
  Up/Down     stand taller / crouch
  Space       stop
  q           quit
"""

# (linear.x, linear.y, linear.z, angular.z) deltas per key
KEYMAP = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0),
    'd': (0.0, -1.0, 0.0, 0.0),
    'UP': (0.0, 0.0, 1.0, 0.0),
    'DOWN': (0.0, 0.0, -1.0, 0.0),
    'LEFT': (0.0, 0.0, 0.0, 1.0),
    'RIGHT': (0.0, 0.0, 0.0, -1.0),
}


def read_key(timeout):
    """Return a single keypress (arrow keys resolved to 'UP'/'DOWN'/'LEFT'/'RIGHT'),
    or None if nothing arrives within `timeout` seconds."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        # arrow keys arrive as ESC [ A/B/C/D; the two follow-up bytes are
        # already buffered by the terminal, so a short timeout is enough
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not ready:
            return 'ESC'
        seq = sys.stdin.read(2)
        return {'[A': 'UP', '[B': 'DOWN', '[C': 'RIGHT', '[D': 'LEFT'}.get(seq)
    return ch


def main():
    rclpy.init()
    node = Node('keyboard_teleop')
    pub = node.create_publisher(Twist, '/robot1/cmd_vel', 10)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setraw(fd)
        last_key_time = time.time()
        twist = Twist()
        period = 1.0 / RATE_HZ
        while rclpy.ok():
            key = read_key(period)
            now = time.time()

            if key in ('q', '\x03'):  # q or Ctrl-C
                break
            elif key == ' ':
                twist = Twist()
                last_key_time = now
            elif key in KEYMAP:
                dx, dy, dz, dyaw = KEYMAP[key]
                twist = Twist()
                twist.linear.x = dx
                twist.linear.y = dy
                twist.linear.z = dz
                twist.angular.z = dyaw
                last_key_time = now
            elif key is not None:
                pass  # unmapped key, ignore

            if now - last_key_time > WATCHDOG_S:
                twist = Twist()

            pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        pub.publish(Twist())  # always leave it stopped
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        print("\nstopped.")


if __name__ == '__main__':
    main()
