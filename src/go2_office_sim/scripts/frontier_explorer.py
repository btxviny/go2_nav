#!/usr/bin/env python3
"""Autonomous frontier exploration for the Go2, driven entirely through Nav2.

No teleop, no manually-placed goals: reads /robot1/map (slam_toolbox's live occupancy
grid), finds the boundary between known-free and unknown space, and repeatedly sends
Nav2 the nearest reachable frontier as a NavigateToPose goal until the map stops
growing (no frontiers left) or a time/attempt budget runs out. Full design writeup in
docs/autonomous_exploration_plan.adoc and the project plan this was built from.

Run alongside office_sim.launch.py + kiss_icp.launch.py + nav_stack.launch.py (KISS-ICP
must be up first -- it's the real odom source, see README's KISS-ICP section):

    python3 ~/go2_nav/src/go2_office_sim/scripts/frontier_explorer.py
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# slam_toolbox publishes /robot1/map latched (transient_local) -- a default/volatile
# subscription would never receive the retained message.
MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

UNKNOWN = -1
FREE_MAX = 49  # nav_msgs/OccupancyGrid: 0-100 occupied probability, -1 unknown
MIN_FRONTIER_CELLS = 6  # drop noise-sized clusters
CANDIDATE_K = 5  # how many nearest clusters to refine with a real Nav2 path length
BLACKLIST_RADIUS_M = 0.5

# Confirmed live: a frontier cluster centroid can legitimately fall within
# nav2_params.yaml's xy_goal_tolerance (0.25m) of the robot's own current
# position -- e.g. a small unresolvable pocket right next to the robot from
# self-hit lidar noise on the walking gait (see README's "invisible
# obstacles" note). Nav2 then correctly reports SUCCEEDED instantly, without
# the robot moving at all, since it was already "there" by the goal
# tolerance's own definition. The map/frontier set doesn't change (no new
# real motion, no new lidar viewpoint), so the exact same cluster gets
# rechosen next loop -- an infinite, ~100ms-per-cycle no-op loop that looks
# like "stuck in a small area" and burns through MAX_ATTEMPTS in well under
# three minutes (confirmed live: 1000 attempts in 146s), which then
# legitimately tore down the whole Nav2 stack via this script's own
# MAX_ATTEMPTS-exhausted nav.lifecycleShutdown() call -- not a crash, but
# indistinguishable from one without checking the log. Requiring every
# candidate to be meaningfully farther than goal tolerance forces the robot
# to actually walk somewhere before Nav2 can call it done.
MIN_GOAL_DIST_M = 0.6
# Defense in depth alongside MIN_GOAL_DIST_M: even a goal that clears that
# filter could still resolve as a Nav2 "success" without real movement (e.g.
# a very short real path that a generous lookahead distance completes in one
# controller tick). A genuine walk to a >=0.6m frontier at this stack's
# tuned ~0.4 m/s cruise speed cannot finish in under a second even counting
# acceleration ramp-up, so treat a suspiciously-instant "success" the same
# as a real failure -- blacklist it rather than let it loop forever.
INSTANT_SUCCESS_S = 1.0

# "gets stuck in small areas" per direct ask: MIN_GOAL_DIST_M above only
# guarantees each individual goal is far enough from the robot's *current*
# position to force real movement -- nothing stopped consecutive goals from
# bouncing between two nearby frontier clusters in the same small pocket
# (e.g. a doorway sliver and a bit of self-hit noise a meter apart), each one
# individually satisfying MIN_GOAL_DIST_M without the robot ever making net
# progress into a new area. This is a separate check against the *previous*
# goal actually sent, not the robot's position.
MIN_CONSECUTIVE_GOAL_DIST_M = 1.0

# The office is a ~20x20m building centered on the origin (see
# blender/scene_objects.json -- every room's bounds fall within [-10, 10] on
# both axes). Lidar can see through/around the glass entrance door (confirmed
# live: the robot spawns near it and immediately picked up points outside the
# building), which without this filter creates "frontiers" leading it to try
# to explore the outdoors -- not a sensor bug to fix, just not a valid
# exploration target. Margin is small (building edge, not room edge) since
# real frontiers can legitimately sit right up against an exterior wall.
BUILDING_BOUNDS_X = (-10.5, 10.5)
BUILDING_BOUNDS_Y = (-10.5, 10.5)

# "No frontiers left" (below) is the real completion signal -- by definition,
# once every reachable free cell has no unknown neighbor, the reachable space
# is fully mapped. MAX_ATTEMPTS/MAX_RUNTIME_S are only a safety net against a
# genuine bug (e.g. an infinite retry loop) ever running forever unattended;
# they're intentionally generous so they don't fire during a normal run.
MAX_ATTEMPTS = 1000
MAX_RUNTIME_S = 60 * 60
GOAL_TIMEOUT_S = 90
MAX_RETRIES_PER_GOAL = 2  # retries before a repeatedly-failing location gets blacklisted
RETRY_BACKOFF_S = 1.5  # pause before retrying, to let a transient TF hiccup clear

# If this many goals in a row fail (not just the same one repeatedly -- ANY
# goal), the robot itself is very likely physically stuck against something
# (wedged, oscillating) rather than just having picked a bad frontier --
# trigger an explicit unstuck maneuver instead of keep trying to plan through
# whatever it's wedged against.
STUCK_AFTER_CONSECUTIVE_FAILURES = 3
UNSTUCK_BACKUP_DIST_M = 0.3
UNSTUCK_SPIN_DIST_RAD = math.pi  # full 180 -- a fresh look at the surroundings


class MapTF(Node):
    """Holds the latest occupancy grid and a map->base_link TF lookup.

    A separate small node rather than reusing BasicNavigator's own node: this script
    runs via `python3 <path>` (project convention -- ros2 run isn't installed here),
    so there's no launch file to remap /tf/-> /robot1/tf the way rviz.launch.py and
    every other launch file in this project already have to. tf2_ros.TransformListener
    always listens on the literal absolute /tf and /tf_static topics regardless of
    node namespace, so the equivalent fix here is cli_args.
    """

    def __init__(self):
        super().__init__(
            'frontier_explorer_tf',
            namespace='/robot1',
            cli_args=['--ros-args', '-r', '/tf:=/robot1/tf', '-r', '/tf_static:=/robot1/tf_static'],
        )
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None
        self.create_subscription(OccupancyGrid, 'map', self._on_map, MAP_QOS)

    def _on_map(self, msg):
        self.latest_map = msg

    def robot_xy(self):
        t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y


def find_frontiers(grid: OccupancyGrid):
    """Free cells 8-connected to an unknown cell, clustered, as (x, y, yaw) goals.

    yaw points from each cluster's centroid toward its own unknown side (the
    mean direction, over every frontier cell in the cluster, to the unknown
    neighbor cells it borders) -- i.e. "into" the unexplored region, not just
    an arbitrary/identity orientation. This matters because the head-mounted
    RGBD camera is forward-facing (base_link +X) while the lidar is a 360
    degree sensor mounted on the back -- lidar coverage barely depends on
    which way the robot is pointed, but arriving at a frontier facing the
    wrong way means the camera looks at already-known space or a wall while
    the one direction it needed to see (the unknown side that made this a
    frontier in the first place) is behind it. Facing the unknown-ward
    direction maximizes what the camera actually captures on arrival.
    """
    w, h = grid.info.width, grid.info.height
    data = np.array(grid.data, dtype=np.int8).reshape(h, w)
    free = (data >= 0) & (data <= FREE_MAX)
    unknown = data == UNKNOWN

    frontier = np.zeros_like(free)
    # accumulate, per free cell, the sum of (dx, dy) offsets to each unknown
    # neighbor it borders -- reused below to get each cluster's unknown-ward
    # direction without a second full pass over the grid.
    dir_x = np.zeros((h, w), dtype=np.float64)
    dir_y = np.zeros((h, w), dtype=np.float64)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            shifted_unknown = np.roll(np.roll(unknown, dy, axis=0), dx, axis=1)
            hit = free & shifted_unknown
            frontier |= hit
            # np.roll(arr, shift)[i] == arr[i - shift], so a hit here means
            # the real unknown neighbor sits at offset (-dx, -dy) from this
            # free cell, not (dx, dy) -- confirmed by a unit test that first
            # caught this backwards (goals faced the known side, not the
            # unknown one -- exactly the "wall in front of the camera" bug).
            dir_x[hit] += -dx
            dir_y[hit] += -dy
    # np.roll wraps around at the edges -- zero those out so we don't treat a
    # wrap-around artifact as a real frontier cell.
    frontier[0, :] = False
    frontier[-1, :] = False
    frontier[:, 0] = False
    frontier[:, -1] = False

    visited = np.zeros_like(frontier)
    clusters = []
    for y, x in zip(*np.where(frontier)):
        if visited[y, x]:
            continue
        stack = [(y, x)]
        visited[y, x] = True
        cells = []
        while stack:
            cy, cx = stack.pop()
            cells.append((cy, cx))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < h and 0 <= nx < w and frontier[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
        if len(cells) < MIN_FRONTIER_CELLS:
            continue
        ys, xs = zip(*cells)
        cy, cx = sum(ys) / len(ys), sum(xs) / len(xs)
        wx = grid.info.origin.position.x + (cx + 0.5) * grid.info.resolution
        wy = grid.info.origin.position.y + (cy + 0.5) * grid.info.resolution
        if not (BUILDING_BOUNDS_X[0] <= wx <= BUILDING_BOUNDS_X[1]
                and BUILDING_BOUNDS_Y[0] <= wy <= BUILDING_BOUNDS_Y[1]):
            continue  # outside the building -- see BUILDING_BOUNDS_* comment
        # mean unknown-ward direction over the whole cluster; grid rows are
        # y, columns are x, and world x/y share the grid's own axis sense
        # (only the resolution/origin differ), so summing dir_x/dir_y over
        # the cluster's cells already gives a usable world-frame direction.
        sum_dx = sum(dir_x[cy_, cx_] for cy_, cx_ in cells)
        sum_dy = sum(dir_y[cy_, cx_] for cy_, cx_ in cells)
        yaw = math.atan2(sum_dy, sum_dx) if (sum_dx or sum_dy) else 0.0
        clusters.append((wx, wy, yaw))
    return clusters


def choose_goal(nav: BasicNavigator, clusters, robot_xy, blacklist, last_goal_xy=None):
    """Nearest-frontier, refined by real Nav2 path length over the top-K candidates.

    Pure Euclidean-nearest can pick a frontier that's close as the crow flies but
    actually behind a wall; refining with getPath() (only for a handful of
    candidates, so it stays cheap) avoids that without the backtracking a pure
    largest-cluster heuristic tends to cause in a multi-room office.

    Returns (x, y, yaw) or None -- yaw is each cluster's own unknown-ward
    facing direction from find_frontiers(), carried through unchanged (it
    doesn't affect path length/reachability scoring, only orientation).
    """
    clusters = [
        c for c in clusters
        if math.hypot(c[0] - robot_xy[0], c[1] - robot_xy[1]) > MIN_GOAL_DIST_M
        and all(math.hypot(c[0] - bx, c[1] - by) > BLACKLIST_RADIUS_M for bx, by in blacklist)
    ]
    if not clusters:
        return None

    if last_goal_xy is not None:
        far_from_last = [
            c for c in clusters
            if math.hypot(c[0] - last_goal_xy[0], c[1] - last_goal_xy[1]) > MIN_CONSECUTIVE_GOAL_DIST_M
        ]
        if far_from_last:
            clusters = far_from_last
        # else: every remaining candidate genuinely IS near the last goal
        # (e.g. finishing off a small room) -- fall back to the unfiltered
        # set rather than permanently blocking real, legitimate progress.

    clusters.sort(key=lambda c: math.hypot(c[0] - robot_xy[0], c[1] - robot_xy[1]))

    start = PoseStamped()
    start.header.frame_id = 'map'
    start.pose.position.x, start.pose.position.y = robot_xy
    start.pose.orientation.w = 1.0

    best, best_len = None, math.inf
    for wx, wy, yaw in clusters[:CANDIDATE_K]:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = wx, wy
        goal.pose.orientation.w = 1.0
        path = nav.getPath(start, goal, use_start=True)
        if path is None or not path.poses:
            continue
        length = sum(
            math.hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)
            for a, b in zip(path.poses, path.poses[1:])
        )
        if length < best_len:
            best, best_len = (wx, wy, yaw), length
    return best


def wait_for_task(nav: BasicNavigator, tf_node: MapTF, timeout_s: float):
    """Spin until a BasicNavigator task (goToPose/backup/spin/...) finishes or times out."""
    deadline = time.time() + timeout_s
    while not nav.isTaskComplete() and time.time() < deadline:
        rclpy.spin_once(tf_node, timeout_sec=0.5)
    if time.time() >= deadline:
        nav.cancelTask()


def unstick(nav: BasicNavigator, tf_node: MapTF):
    """Back up and spin in place -- for when the robot looks physically stuck.

    clearAllCostmaps() first: several stuck episodes this session traced back
    to stale "occupied" cost lingering near the robot (self-hit noise from the
    walking gait, or slow-to-clear voxels under load -- see nav2_params.yaml's
    own comments), which can make the controller see a wall that isn't really
    there anymore. Backing up then turning gives both the costmap and the
    frontier search a genuinely fresh look before the main loop tries again.
    """
    nav.get_logger().warn(
        f'{STUCK_AFTER_CONSECUTIVE_FAILURES} goals in a row failed -- looks physically '
        f'stuck, not just a bad frontier. Backing up and turning around.')
    nav.clearAllCostmaps()
    nav.backup(backup_dist=UNSTUCK_BACKUP_DIST_M, backup_speed=0.05, time_allowance=10)
    wait_for_task(nav, tf_node, timeout_s=15)
    nav.spin(spin_dist=UNSTUCK_SPIN_DIST_RAD, time_allowance=10)
    wait_for_task(nav, tf_node, timeout_s=15)


def main():
    rclpy.init()
    tf_node = MapTF()

    nav = BasicNavigator(namespace='/robot1')
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    # 'robot_localization' is the sentinel value that skips BasicNavigator's built-in
    # AMCL wait -- this stack has no amcl node at all (slam_toolbox runs continuously
    # instead), so the default would block here forever.
    nav.waitUntilNav2Active(localizer='robot_localization')

    blacklist = []
    retry_counts = {}
    attempts = 0
    consecutive_failures = 0
    t_start = time.time()
    tf_wait_since = None
    tf_wait_last_logged = None
    last_goal_xy = None

    while rclpy.ok():
        rclpy.spin_once(tf_node, timeout_sec=0.5)

        if time.time() - t_start > MAX_RUNTIME_S:
            nav.get_logger().info(f'Exploration budget exhausted ({MAX_RUNTIME_S}s elapsed).')
            break
        if attempts >= MAX_ATTEMPTS:
            nav.get_logger().info(f'Exploration budget exhausted ({MAX_ATTEMPTS} goal attempts).')
            break

        if tf_node.latest_map is None:
            continue

        clusters = find_frontiers(tf_node.latest_map)
        if not clusters:
            nav.get_logger().info('No frontiers left -- map fully explored.')
            break

        try:
            robot_xy = tf_node.robot_xy()
            tf_wait_since = None
            tf_wait_last_logged = None
        except Exception as exc:  # noqa: BLE001 -- TF lookups raise several distinct types
            # A short sleep here matters: without it this branch spins as fast
            # as the TF buffer keeps rejecting lookups, which during the
            # startup warm-up (or any transient TF hiccup) printed hundreds of
            # near-identical warnings within under two seconds -- confirmed
            # live. This just paces the retry instead of masking anything.
            #
            # This *always* logs at least once per run: kiss_icp/slam_toolbox
            # haven't published their first map->odom/odom->base_link
            # transform yet the moment this script starts, so the very first
            # attempt(s) always land here -- normal, self-resolving startup
            # warm-up, not a bug (confirmed live: it clears within a few
            # seconds). Logging every single 0.5s retry at WARN level during
            # that window was pure noise though; log once when the wait
            # starts, then only every 5s it's still not resolved, so a
            # genuinely stuck TF tree still gets surfaced without spamming
            # the normal case.
            now = time.time()
            if tf_wait_since is None:
                tf_wait_since = now
                tf_wait_last_logged = now
                nav.get_logger().info(
                    f'Waiting for the map->base_link TF chain (kiss_icp/slam_toolbox likely '
                    f'still starting up): {exc}')
            elif now - tf_wait_last_logged >= 5.0:
                tf_wait_last_logged = now
                nav.get_logger().warn(
                    f'Still waiting for map->base_link TF after '
                    f'{now - tf_wait_since:.0f}s: {exc}')
            time.sleep(0.5)
            continue

        chosen = choose_goal(nav, clusters, robot_xy, blacklist, last_goal_xy)
        if chosen is None:
            # Counts toward stuck-detection too, not just a failed goToPose --
            # every nearby candidate's getPath() getting rejected (e.g. the
            # robot's own current position is inside costmap-lethal space)
            # is, if anything, a *stronger* stuck signal than one bad goal,
            # but wasn't being tracked at all before -- exploration could spin
            # on this warning forever in a small area without ever triggering
            # the unstick maneuver.
            consecutive_failures += 1
            nav.get_logger().warn(
                f'All nearby frontier candidates unreachable ({consecutive_failures}/'
                f'{STUCK_AFTER_CONSECUTIVE_FAILURES} before unstick attempt); retrying next loop.')
            if consecutive_failures >= STUCK_AFTER_CONSECUTIVE_FAILURES:
                unstick(nav, tf_node)
                consecutive_failures = 0
            else:
                time.sleep(RETRY_BACKOFF_S)
            continue
        gx, gy, gyaw = chosen
        goal_xy = (gx, gy)  # blacklist/retry bookkeeping below is position-only
        last_goal_xy = goal_xy

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        # Face into the frontier's own unknown side (see find_frontiers'
        # docstring) instead of a fixed identity orientation -- the fixed-yaw
        # version routinely arrived facing already-known space or a wall,
        # leaving the forward-facing camera with nothing new to see even
        # though lidar-based coverage (360 degree, back-mounted) was fine.
        goal.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.orientation.w = math.cos(gyaw / 2.0)

        nav.get_logger().info(
            f'Heading to frontier at ({gx:.2f}, {gy:.2f}), facing {math.degrees(gyaw):.0f} deg')
        goal_sent_at = time.time()
        accepted = nav.goToPose(goal)
        attempts += 1

        # goToPose() returning False (goal rejected, e.g. the start/goal pose
        # is inside costmap-lethal space) is NOT the same as a goal that was
        # accepted and later failed -- confirmed live in nav2_simple_commander's
        # own source: on rejection it returns immediately WITHOUT setting
        # self.result_future to a fresh value, leaving it pointing at
        # whatever the *previous* goToPose/getPath call left behind. Calling
        # wait_for_task()/getResult() anyway (the original bug here) reads
        # that stale leftover state instead of the real rejection -- this is
        # what caused the observed rapid-fire re-attempts with no real
        # backoff ever kicking in (RETRY_BACKOFF_S never actually applied,
        # since the stale future was often already "done").
        if accepted:
            wait_for_task(nav, tf_node, timeout_s=GOAL_TIMEOUT_S)
            succeeded = nav.getResult() == TaskResult.SUCCEEDED
        else:
            nav.get_logger().warn(f'Goal to ({gx:.2f}, {gy:.2f}) was rejected outright.')
            succeeded = False

        if succeeded and (time.time() - goal_sent_at) < INSTANT_SUCCESS_S:
            # See INSTANT_SUCCESS_S's comment -- this is MIN_GOAL_DIST_M's
            # backstop, not the primary fix, so it's rare in practice. Falls
            # through to the existing failure-handling branch below (retry
            # count -> eventual blacklist) instead of duplicating that logic.
            nav.get_logger().warn(
                f'Goal at {goal_xy} "succeeded" in under {INSTANT_SUCCESS_S}s -- too fast for '
                f'a real walk, likely already within goal tolerance; treating as a failure.')
            succeeded = False

        if succeeded:
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            # Goals that fail almost instantly are overwhelmingly a transient
            # TF/timing hiccup (confirmed live this session: "Transform data
            # too old when converting from map to odom" under real CPU
            # contention), not a genuinely unreachable frontier -- blacklisting
            # on the very first failure was burning through good frontiers
            # over a single momentary TF stall. Retry the *same* location a
            # couple of times, with a short backoff so the hiccup has a
            # chance to clear, before finally giving up on it.
            key = (round(goal_xy[0], 2), round(goal_xy[1], 2))
            retry_counts[key] = retry_counts.get(key, 0) + 1
            elapsed = time.time() - goal_sent_at
            if retry_counts[key] > MAX_RETRIES_PER_GOAL:
                nav.get_logger().warn(
                    f'Goal at {goal_xy} failed {retry_counts[key]} times -- blacklisting it.')
                blacklist.append(goal_xy)
            else:
                nav.get_logger().warn(
                    f'Goal at {goal_xy} failed after {elapsed:.1f}s '
                    f'(attempt {retry_counts[key]}/{MAX_RETRIES_PER_GOAL}) -- retrying shortly.')
                time.sleep(RETRY_BACKOFF_S)

            if consecutive_failures >= STUCK_AFTER_CONSECUTIVE_FAILURES:
                unstick(nav, tf_node)
                consecutive_failures = 0

    nav.lifecycleShutdown()
    tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
