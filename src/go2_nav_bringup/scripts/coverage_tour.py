#!/usr/bin/env python3
"""Hardcoded circular tour around podC's desk cluster for semantic-map data
collection, driven through Nav2. Independent of run_stack.sh/the main stack --
this is launched on its own, after the base stack (office_sim + odometry +
nav_stack) is already up, and it owns its own bag recording (see below): it is
not something run_stack.sh starts for you.

Unlike frontier_explorer.py (which explores until the lidar-based occupancy grid is
fully known -- a good proxy for lidar coverage, but not for the forward-facing RGBD
camera, which only sees whatever it happens to walk past), and unlike an earlier
room-to-room version of this script (reverted -- long cross-building legs through
doorways turned out to badly stress this stack's locomotion/timing margins under
live testing), this walks a short, tight loop around podC -- the 2x2 desk cluster
nearest the building's geometric centre (podA and podB are two further, separate
pods off to the west; see blender/scene_objects.json's "objects" list for
podA/podB/podC_workstation*), a few metres from the robot's own spawn point
(0.0, 1.1) -- see office_sim.launch.py. At each stop the robot faces *outward*, away
from the desks, so the forward-facing camera sweeps across the surrounding open_plan
area (where most of the scene's small objects -- cones, cubes -- are scattered) from
a different angle each stop, rather than staring at the same four desks throughout.
Total loop length is small (~16m, a circle of radius 2.5m around podC's centre) -- a
short, bounded run, not an open-ended search.

Sequencing: this first drives to the circle's starting point with NO recording
running (so the bag doesn't capture the walk over from wherever the robot happened
to be), then starts a bag recording (shelling out to record_bag.sh -- same topic
list, one source of truth), then drives the rest of the circle, ending back at the
start to close the loop, then stops the recording.

This is a ros2-launch-only node, not a standalone script -- coverage_tour.launch.py
owns namespacing and /tf, /tf_static remapping (same division of responsibility as
frontier_explorer.py/frontier_explorer.launch.py; see that pair's docstrings), so
running this directly via `python3 <path>` will come up unnamespaced against the
root /tf tree instead.

Run after office_sim.launch.py + kiss_icp.launch.py + nav_stack.launch.py are up:

    ros2 launch go2_nav_bringup coverage_tour.launch.py [bag_output:=/path/to/bag]
"""
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# podC's 4 workstations (blender/scene_objects.json) span x=[-1.85,-0.15],
# y=[-2.85,0.45] -- centre (-1.0,-1.2), half-diagonal ~1.86m. Radius 2.5m clears the
# whole cluster (plus the robot's own footprint/inflation radius) with margin.
_PODC_CENTER = (-1.0, -1.2)
_RADIUS_M = 2.5
_N_WAYPOINTS = 8

# (x, y, yaw_deg, label) -- yaw = the point's own angle around the circle, i.e.
# radially outward from podC's centre (see module docstring for why: so the
# forward camera looks out across the room, not at the desks). Ordered starting
# from the point nearest the robot's spawn (0.0, 1.1) and proceeding around the
# circle in one direction, so the whole loop is one continuous sweep with no
# backtracking. WAYPOINTS[0] is the starting position (driven to before recording
# starts); the recorded tour then visits WAYPOINTS[1:] and finally WAYPOINTS[0]
# again, to close the loop.
WAYPOINTS = []
for _i in range(_N_WAYPOINTS):
    _theta_deg = 90 + _i * (360 / _N_WAYPOINTS)  # start at the 90 deg point (nearest spawn)
    _theta = math.radians(_theta_deg)
    _x = _PODC_CENTER[0] + _RADIUS_M * math.cos(_theta)
    _y = _PODC_CENTER[1] + _RADIUS_M * math.sin(_theta)
    WAYPOINTS.append((_x, _y, _theta_deg % 360, f'podC circle @ {round(_theta_deg % 360)} deg'))

GOAL_TIMEOUT_S = 90  # per waypoint, matches frontier_explorer.py's GOAL_TIMEOUT_S
# Confirmed live: a goal can fail (or even get aborted near-instantly, e.g.
# "Timed out while waiting for action server to acknowledge goal request for
# compute_path_to_pose" -- nav2_params.yaml's bt_navigator.default_server_timeout
# is 20ms, a tight window that a momentary scheduling hiccup right after Nav2's
# lifecycle nodes activate can miss) on a genuinely reachable waypoint, purely from
# transient timing -- same class of hiccup frontier_explorer.py's own
# MAX_RETRIES_PER_GOAL/RETRY_BACKOFF_S exists to absorb. Retrying a couple of times
# before giving up on a waypoint avoids one bad tick silently emptying the tour.
MAX_RETRIES_PER_WAYPOINT = 2
RETRY_BACKOFF_S = 2.0
# Give rosbag2 a moment to discover and subscribe to every topic (confirmed live:
# subscribing to 6 topics after process start takes a bit over a second) before the
# tour starts moving -- otherwise the first waypoint's early frames/tf can be missed.
RECORDER_STARTUP_WAIT_S = 3.0
RECORDER_SHUTDOWN_TIMEOUT_S = 15.0  # mcap finalization needs a clean SIGINT, not a kill


def wait_for_task(nav: BasicNavigator, timeout_s: float):
    """Spin until a BasicNavigator task finishes or times out (cancels on timeout)."""
    deadline = time.time() + timeout_s
    while not nav.isTaskComplete() and time.time() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.5)
    if time.time() >= deadline:
        nav.cancelTask()


def navigate_to(nav: BasicNavigator, x: float, y: float, yaw_deg: float, label: str) -> bool:
    """goToPose with retries (see MAX_RETRIES_PER_WAYPOINT's comment). Returns success."""
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x, goal.pose.position.y = x, y
    yaw = math.radians(yaw_deg)
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)

    for attempt in range(MAX_RETRIES_PER_WAYPOINT + 1):
        goal.header.stamp = nav.get_clock().now().to_msg()
        accepted = nav.goToPose(goal)
        if not accepted:
            nav.get_logger().warn(
                f'Goal to {label} was rejected (attempt {attempt + 1}/'
                f'{MAX_RETRIES_PER_WAYPOINT + 1}).')
        else:
            wait_for_task(nav, GOAL_TIMEOUT_S)
            if nav.getResult() == TaskResult.SUCCEEDED:
                nav.get_logger().info(f'Reached {label}.')
                return True
            nav.get_logger().warn(
                f'Did not reach {label} (attempt {attempt + 1}/'
                f'{MAX_RETRIES_PER_WAYPOINT + 1}).')
        if attempt < MAX_RETRIES_PER_WAYPOINT:
            time.sleep(RETRY_BACKOFF_S)

    # One bad waypoint shouldn't stall the whole tour -- log and move on, same
    # "keep going" philosophy as frontier_explorer.py's retry/blacklist logic
    # (minus the blacklist -- this list is short and fixed, not an open-ended
    # search, so there's nothing to permanently avoid).
    nav.get_logger().warn(f'Giving up on {label} after {MAX_RETRIES_PER_WAYPOINT + 1} '
                           f'attempts -- continuing.')
    return False


def start_recording(nav: BasicNavigator, bag_output: str):
    """Shell out to record_bag.sh (same topic list, one source of truth) rather than
    reimplementing the `ros2 bag record` invocation here."""
    record_bag_sh = Path(__file__).resolve().parent / 'record_bag.sh'
    args = [str(record_bag_sh)] + ([bag_output] if bag_output else [])
    nav.get_logger().info(f'Starting bag recording: {" ".join(args)}')
    proc = subprocess.Popen(args)
    time.sleep(RECORDER_STARTUP_WAIT_S)
    if proc.poll() is not None:
        nav.get_logger().error(
            f'record_bag.sh exited immediately (code {proc.returncode}) -- aborting tour '
            f'without recording anything.')
        sys.exit(1)
    return proc


def stop_recording(nav: BasicNavigator, proc: subprocess.Popen):
    """SIGINT (not kill) so rosbag2 finalizes the mcap file/index cleanly -- same as a
    user Ctrl+C-ing record_bag.sh in a terminal."""
    nav.get_logger().info('Stopping bag recording...')
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=RECORDER_SHUTDOWN_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        nav.get_logger().warn(
            f'record_bag.sh did not exit within {RECORDER_SHUTDOWN_TIMEOUT_S}s -- killing it '
            f'(bag may be incomplete).')
        proc.kill()
        proc.wait()


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.declare_parameter('bag_output', '')
    bag_output = nav.get_parameter('bag_output').value

    # See frontier_explorer.py's identical call for why 'robot_localization' is used
    # as the sentinel that skips BasicNavigator's built-in AMCL wait (this stack has
    # no amcl node -- slam_toolbox runs continuously instead).
    nav.waitUntilNav2Active(localizer='robot_localization')

    start_x, start_y, start_yaw, start_label = WAYPOINTS[0]
    nav.get_logger().info(f'Navigating to the tour\'s starting position: {start_label}...')
    if not navigate_to(nav, start_x, start_y, start_yaw, start_label):
        nav.get_logger().error('Could not reach the starting position -- aborting '
                                'without ever starting the recording.')
        nav.lifecycleShutdown()
        rclpy.shutdown()
        sys.exit(1)

    recorder = start_recording(nav, bag_output)

    remaining = WAYPOINTS[1:] + [WAYPOINTS[0]]  # ...then back to the start, closing the loop
    nav.get_logger().info(f'Starting recorded coverage tour: {len(remaining)} waypoints.')
    for i, (x, y, yaw_deg, label) in enumerate(remaining):
        nav.get_logger().info(f'[{i + 1}/{len(remaining)}] Heading to {label} at ({x:.2f}, {y:.2f})')
        navigate_to(nav, x, y, yaw_deg, label)

    nav.get_logger().info('Coverage tour complete.')
    stop_recording(nav, recorder)
    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
