#!/usr/bin/env bash
# Kills any leftover project processes, then launches the full stack in order:
# office_sim -> (1s) -> rviz -> (5s) -> kiss_icp -> (5s) -> nav_stack.
#
# The wait before kiss_icp used to be 15s, a stopgap for the robot's
# gait-stability tip-over bug (KISS-ICP's very first scan registration bakes
# in whatever orientation the robot has *at that moment* as its permanent
# odom-frame reference, so starting it too early misaligned the whole map).
# That bug is now actually fixed, not just delayed around: the real cause was
# every leg joint spawning at 0 while the controller immediately commanded a
# very different ~0.86-1.89 rad stance target, so position control snapped
# the whole body at once while gravity was already acting, and this idle
# stance has no active balance feedback to recover from a bad initial
# disturbance. Fixed at the source (go2_description/xacro/gazebo.xacro's
# per-joint initial_value, paired with office_sim.launch.py's z_pose) so
# there's no snap and no disturbance to begin with -- confirmed live settling
# to an essentially perfectly level pose within about 1-2s. 5s here is just a
# safety margin over that, not a tip-over workaround anymore.
#
# Usage: ~/go2_nav/src/go2_nav_bringup/scripts/run_stack.sh [--odom kiss_icp|fast_lio] [--explore]
# --odom picks the lidar odometry backend (default: kiss_icp). fast_lio is
# spark_fast_lio (FAST-LIO2, tightly-coupled LiDAR-IMU ESKF) -- see
# launch/fast_lio.launch.py for why it's an alternative rather than the default.
# --explore additionally launches frontier_explorer.launch.py after nav_stack comes up,
# for unattended autonomous exploration instead of manual teleop. Off by default -- same
# reasoning as everything else here defaulting to manual control: driving the robot
# unattended is an opt-in choice, not the default.
# Ctrl+C stops everything this script started.

set -u

ODOM_BACKEND="kiss_icp"
EXPLORE=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --odom)
            ODOM_BACKEND="$2"
            shift 2
            ;;
        --odom=*)
            ODOM_BACKEND="${1#--odom=}"
            shift
            ;;
        --explore)
            EXPLORE=1
            shift
            ;;
        *)
            echo "Unknown argument: $1" >&2
            echo "Usage: $0 [--odom kiss_icp|fast_lio] [--explore]" >&2
            exit 1
            ;;
    esac
done
case "$ODOM_BACKEND" in
    kiss_icp|fast_lio) ;;
    *)
        echo "Invalid --odom value '$ODOM_BACKEND' (expected kiss_icp or fast_lio)" >&2
        exit 1
        ;;
esac

# office_fastlio.rviz swaps which backend's accumulated-map display is
# enabled (FastLioGlobalMap vs. KissIcpLocalMap) -- each backend only publishes its
# own map topic, so the "other" display would just sit empty otherwise.
GO2_NAV_BRINGUP_SHARE="$(ros2 pkg prefix go2_nav_bringup)/share/go2_nav_bringup"
if [ "$ODOM_BACKEND" = "fast_lio" ]; then
    RVIZ_CONFIG="$GO2_NAV_BRINGUP_SHARE/config/office_fastlio.rviz"
else
    RVIZ_CONFIG="$GO2_NAV_BRINGUP_SHARE/config/office.rviz"
fi

# "ros2 launch go2_nav_bringup", not bare "go2_nav_bringup": this script's own
# path (.../scripts/run_stack.sh) contains the substring "go2_nav_bringup" too
# -- pkill/pgrep only ever auto-exclude their own PID, not an ancestor script
# that invoked them, so a bare "go2_nav_bringup" term matched and killed this
# very script the moment it ran (confirmed live: printed "Killed" and exited
# right after the cleanup step, before launching anything).
PROJECT_PROCS="gz sim|ros_gz_bridge|quadruped_controller/lib|ros2 launch go2_nav_bringup|kiss_icp_node|spark_lio_mapping|rviz2|slam_toolbox|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|opennav_docking|route_server|smoother_server|pointcloud_to_laserscan_node|lifecycle_manager|frontier_explorer"

# Belt-and-suspenders on top of the narrowed pattern above: explicitly never
# kill this script's own PID, in case a future pattern term accidentally
# matches this file's path again.
kill_project_procs() {
    pgrep -f "$PROJECT_PROCS" 2>/dev/null | grep -v -x "$$" | xargs -r kill -9 2>/dev/null
}

echo "==> Killing any existing project processes..."
kill_project_procs
# 5s, not 2s: a SIGKILL'd Gazebo/RViz GUI doesn't get a chance to cleanly
# release its GL context, and this machine's EGL/driver setup already looks
# fragile (recurring "libEGL warning: pci id for fd ...: driver (null)" in
# normal logs) -- relaunching a new GUI process too soon after killing the
# old one is a plausible reason gazebo/rviz would silently fail to come up on
# a rerun specifically, matching what was observed live.
sleep 5

PIDS=()

cleanup() {
    echo
    echo "==> Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    # 5s, not 2s: nav_stack.launch.py alone tears down ~10 Nav2 lifecycle
    # nodes on SIGINT, which can genuinely take longer than 2s -- the old,
    # shorter grace period is why launches were getting caught by the SIGKILL
    # sweep below instead of exiting cleanly (printed as bash's own "Killed"
    # job-control message, alarming but harmless -- the process just didn't
    # finish its graceful shutdown in time).
    sleep 5
    kill_project_procs
}
trap cleanup EXIT INT TERM

# require_alive: aborts the whole script (via cleanup, since it triggers exit)
# if any named PID has died. Checked at the END of each settle window, not
# right after launching -- a single immediate check missed exactly the
# failure mode observed live: gazebo/rviz surviving their first second or two
# then dying a bit later (e.g. mid GL-context/world-load), which the old
# check didn't catch, so the script barreled ahead into kiss_icp/nav_stack
# against a dead base instead of stopping (confirmed live: kiss_icp_node and
# the whole nav2 stack were still running with no gazebo or rviz at all).
require_alive() {
    local ok=1
    while [ "$#" -gt 0 ]; do
        local name="$1" pid="$2"
        shift 2
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "!!! $name (pid $pid) is not running -- check its terminal output above."
            ok=0
        fi
    done
    if [ "$ok" -eq 0 ]; then
        echo "!!! Aborting: the stack can't come up correctly like this. Fix the issue above and rerun."
        exit 1
    fi
}

echo "==> Launching office_sim.launch.py..."
ros2 launch go2_nav_bringup office_sim.launch.py &
PIDS+=("$!")
GAZEBO_PID="$!"

# 1s, not simultaneous: gives Gazebo's own GUI a moment to grab its GL context
# before RViz tries to grab its own -- concurrent GL context creation at
# startup was an earlier confirmed cause of RViz dying silently.
sleep 1

echo "==> Launching rviz.launch.py..."
ros2 launch go2_nav_bringup rviz.launch.py rviz_config:="$RVIZ_CONFIG" &
PIDS+=("$!")
RVIZ_PID="$!"

echo "==> Waiting 5s for the robot to settle and Gazebo/RViz to finish starting..."
sleep 5
require_alive "office_sim.launch.py" "$GAZEBO_PID" "rviz.launch.py" "$RVIZ_PID"

if [ "$ODOM_BACKEND" = "fast_lio" ]; then
    echo "==> Launching fast_lio.launch.py..."
    ros2 launch go2_nav_bringup fast_lio.launch.py &
else
    echo "==> Launching kiss_icp.launch.py..."
    ros2 launch go2_nav_bringup kiss_icp.launch.py &
fi
PIDS+=("$!")
ODOM_PID="$!"
sleep 5
require_alive "odometry ($ODOM_BACKEND)" "$ODOM_PID"

echo "==> Launching nav_stack.launch.py..."
ros2 launch go2_nav_bringup nav_stack.launch.py &
PIDS+=("$!")
NAV_STACK_PID="$!"
sleep 2
require_alive "nav_stack.launch.py" "$NAV_STACK_PID"

if [ "$EXPLORE" -eq 1 ]; then
    # 10s, not 2s like the others above: frontier_explorer.py's own
    # waitUntilNav2Active() already blocks until Nav2's lifecycle nodes report
    # active, but the map->base_link TF chain (kiss_icp + slam_toolbox) also
    # needs its first few scans registered before there's anything to explore
    # from -- giving nav_stack a real head start here avoids it just sitting
    # in frontier_explorer's own TF-wait retry loop instead.
    echo "==> Waiting 10s for Nav2/SLAM to settle before starting exploration..."
    sleep 10
    echo "==> Launching frontier_explorer.launch.py..."
    ros2 launch go2_nav_bringup frontier_explorer.launch.py &
    PIDS+=("$!")
    EXPLORE_PID="$!"
    sleep 2
    require_alive "frontier_explorer.launch.py" "$EXPLORE_PID"
fi

echo "==> All launched. Ctrl+C to stop everything."
wait
