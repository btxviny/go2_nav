"""The entire stack (office_sim + rviz + kiss_icp + nav_stack) via one `ros2 launch`,
instead of running scripts/run_stack.sh or four separate terminal commands.

Staggers the four includes with the same timing run_stack.sh uses (office_sim at
t=0, rviz at t=1s, kiss_icp at t=6s, nav_stack at t=11s) so each stage gets a
chance to settle before the next depends on it -- see run_stack.sh's own
comments for why those specific gaps (RViz needs Gazebo's GL context grabbed
first; kiss_icp/nav_stack both need the robot settled and odom->base_link
already flowing).

**Not a full replacement for run_stack.sh** -- this file intentionally does
NOT replicate two things that script does in bash:
  - Killing leftover project processes before starting (see README's
    "Starting over cleanly" for the manual pkill one-liner if you suspect
    stale processes are still around).
  - Aborting the whole sequence if an earlier stage's process has already
    died before a later stage's delay elapses (run_stack.sh's
    `require_alive`) -- ros2 launch has no simple built-in equivalent to that
    mid-sequence liveness check, so if e.g. Gazebo crashes a few seconds in,
    kiss_icp/nav_stack still launch against a broken base rather than
    stopping, same as any plain `ros2 launch` would.
It DOES get one real advantage over run_stack.sh: a single Ctrl+C is handled
by launch's own SIGINT propagation, which tears down every included node
(including Nav2's ~10 lifecycle nodes) as part of one managed process tree,
rather than run_stack.sh's own manual PID-tracking/cleanup trap.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    office_share = get_package_share_directory('go2_nav_bringup')

    def include(launch_file_name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(office_share, 'launch', launch_file_name)
            )
        )

    office_sim = include('office_sim.launch.py')
    rviz = TimerAction(period=1.0, actions=[include('rviz.launch.py')])
    kiss_icp = TimerAction(period=6.0, actions=[include('kiss_icp.launch.py')])
    nav_stack = TimerAction(period=11.0, actions=[include('nav_stack.launch.py')])

    return LaunchDescription([office_sim, rviz, kiss_icp, nav_stack])
