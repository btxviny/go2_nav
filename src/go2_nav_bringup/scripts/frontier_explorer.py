#!/usr/bin/env python3
"""Autonomous frontier exploration for the Go2, driven entirely through Nav2.

No teleop, no manually-placed goals: reads /robot1/map (slam_toolbox's live occupancy
grid), finds the boundary between known-free and unknown space via Wavefront Frontier
Detection (see find_frontiers()'s docstring), scores each frontier cluster by a
information-gain-vs-path-length utility (see choose_goal()'s docstring), and repeatedly
sends Nav2 the best-scoring reachable frontier as a NavigateToPose goal until the map
stops growing (no frontiers left) or a time/attempt budget runs out. Full design writeup
in docs/autonomous_exploration_plan.adoc and the project plan this was built from.

All tunables below are ROS2 parameters (see load_config()), backed by
config/frontier_explorer.yaml -- override them there or via the usual
`ros2 launch ... frontier_explorer.launch.py` parameter mechanisms rather than editing
this file.

This is a ros2-launch-only node, not a standalone script -- frontier_explorer.launch.py
owns namespacing and /tf, /tf_static remapping (same division of responsibility as
slam_toolbox/kiss_icp_node; see that launch file's docstring), so running this directly
via `python3 <path>` will come up unnamespaced against the root /tf tree instead.

Run alongside office_sim.launch.py + kiss_icp.launch.py + nav_stack.launch.py (KISS-ICP
must be up first -- it's the real odom source, see README's KISS-ICP section):

    ros2 launch go2_nav_bringup frontier_explorer.launch.py
"""
import math
import time
from collections import deque
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.node import Node
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


def load_config(node: Node) -> SimpleNamespace:
    """Declare and read this node's tunables as ROS2 parameters.

    Every value here previously lived as a hardcoded module-level constant; the defaults
    below match that version exactly, and config/frontier_explorer.yaml documents the
    reasoning behind each one (the comments here were moved there, not deleted) -- so
    behavior is identical until that file (or a launch-time override) actually changes
    something.
    """
    defaults = {
        'min_frontier_cells': 25,
        'seed_search_radius_cells': 15,
        'candidate_k': 5,
        'info_gain_radius_m': 1.0,
        'info_gain_weight': 1.0,
        'min_goal_dist_m': 0.6,
        'instant_success_s': 1.0,
        'min_consecutive_goal_dist_m': 2.5,
        'blacklist_radius_m': 1.5,
        'building_bounds_x_min': -10.5,
        'building_bounds_x_max': 10.5,
        'building_bounds_y_min': -10.5,
        'building_bounds_y_max': 10.5,
        'max_attempts': 1000,
        'max_runtime_s': 3600.0,
        'goal_timeout_s': 90.0,
        'max_retries_per_goal': 1,
        'retry_backoff_s': 1.5,
        'stuck_after_consecutive_failures': 2,
        'unstuck_backup_dist_m': 1.0,
        'unstuck_spin_dist_rad': math.pi,
    }
    for name, default in defaults.items():
        node.declare_parameter(name, default)
    cfg = SimpleNamespace(**{name: node.get_parameter(name).value for name in defaults})
    cfg.building_bounds_x = (cfg.building_bounds_x_min, cfg.building_bounds_x_max)
    cfg.building_bounds_y = (cfg.building_bounds_y_min, cfg.building_bounds_y_max)
    return cfg


class MapTF(Node):
    """Holds the latest occupancy grid and a map->base_link TF lookup.

    A separate small node rather than reusing BasicNavigator's own node, purely so this
    file has one obvious place to hold latest_map + the TF buffer. Namespacing and the
    /tf, /tf_static remap (tf2_ros.TransformListener always listens on the literal
    absolute /tf/tf_static topics regardless of node namespace) are both handled by
    frontier_explorer.launch.py's GroupAction/PushRosNamespace/SetRemap wrapper now,
    not here -- see this module's docstring.
    """

    def __init__(self):
        super().__init__('frontier_explorer_tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None
        self.create_subscription(OccupancyGrid, 'map', self._on_map, MAP_QOS)

    def _on_map(self, msg):
        self.latest_map = msg

    def robot_xy(self):
        t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y


def world_to_grid(grid: OccupancyGrid, x, y):
    """Map-frame (x, y) -> (row, col) grid indices, per OccupancyGrid.info."""
    col = int((x - grid.info.origin.position.x) / grid.info.resolution)
    row = int((y - grid.info.origin.position.y) / grid.info.resolution)
    return row, col


def _nearest_free_cell(free, row, col, seed_search_radius_cells):
    """(row, col) if free, else the nearest free cell within seed_search_radius_cells.

    WFD needs a free cell to seed its outer BFS from. The robot's own map cell can
    legitimately not be `free` at the instant this runs -- self-hit lidar noise from the
    walking gait can leave a small halo of occupied/unknown cells right around the robot
    even while it's plainly standing on real free space -- so search outward in
    expanding rings rather than assuming the exact cell is usable.
    """
    h, w = free.shape
    if 0 <= row < h and 0 <= col < w and free[row, col]:
        return row, col
    for r in range(1, seed_search_radius_cells + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dy), abs(dx)) != r:
                    continue  # only the newly-added ring at this radius
                ny, nx = row + dy, col + dx
                if 0 <= ny < h and 0 <= nx < w and free[ny, nx]:
                    return ny, nx
    return None


def find_frontiers(grid: OccupancyGrid, robot_rc, cfg: SimpleNamespace):
    """Wavefront Frontier Detection (WFD), seeded at the robot's own map cell.

    Two-pass BFS per Topiwala/Inani/Kathpal, "Frontier Based Exploration for Autonomous
    Robot" (arXiv:1806.03581) -- the same paper nav2_wfd
    (github.com/SeanReg/nav2_wavefront_frontier_exploration) implements. An outer BFS
    ("Map-Open/Close-List") floods known-free space reachable from the robot; whenever it
    reaches a free cell touching unknown space, an inner BFS ("Frontier-Open/Close-List")
    floods the whole contiguous frontier region from there, in one pass, before the outer
    BFS continues.

    This only ever proposes frontiers in the free-space component the robot can *actually
    reach right now*, per the current map -- a disconnected "free" blob from sensor noise
    across a wall (or behind a still-unopened door) can never surface as a candidate here.

    Each returned cluster is (wx, wy, yaw, gain_m2):
      - yaw points from each cluster's centroid toward its own unknown side (the mean
        direction, over every frontier cell in the cluster, to the unknown neighbor cells
        it borders) -- "into" the unexplored region. This matters because the head-mounted
        RGBD camera is forward-facing (base_link +X) while the lidar is a 360 degree
        sensor mounted on the back -- arriving facing the wrong way leaves the camera
        looking at already-known space while the one direction it needed to see is behind
        it. Facing the unknown-ward direction maximizes what the camera actually captures
        on arrival.
      - gain_m2 is the estimated unknown-cell area (m^2) within cfg.info_gain_radius_m of
        the cluster's centroid -- an approximate "how much new map this frontier is likely
        to reveal", used by choose_goal()'s utility scoring. Computed here rather than as
        a separate pass, since the unknown-space array and each cluster's cell list are
        already in scope from the BFS above.
    """
    w, h = grid.info.width, grid.info.height
    data = np.array(grid.data, dtype=np.int8).reshape(h, w)
    free = (data >= 0) & (data <= FREE_MAX)
    unknown = data == UNKNOWN
    resolution = grid.info.resolution
    gain_radius_cells = max(1, round(cfg.info_gain_radius_m / resolution))

    def neighbors8(y, x):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w:
                    yield ny, nx

    def is_frontier_point(y, x):
        return bool(free[y, x]) and any(unknown[ny, nx] for ny, nx in neighbors8(y, x))

    def unknown_gain_m2(cy, cx):
        """Unknown-cell area (m^2) within gain_radius_cells of grid cell (cy, cx)."""
        y0, y1 = max(0, cy - gain_radius_cells), min(h, cy + gain_radius_cells + 1)
        x0, x1 = max(0, cx - gain_radius_cells), min(w, cx + gain_radius_cells + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        disk = (yy - cy) ** 2 + (xx - cx) ** 2 <= gain_radius_cells ** 2
        return float(np.count_nonzero(unknown[y0:y1, x0:x1] & disk)) * resolution * resolution

    seed = _nearest_free_cell(free, *robot_rc, cfg.seed_search_radius_cells)
    if seed is None:
        return []

    map_open = np.zeros((h, w), dtype=bool)
    map_close = np.zeros((h, w), dtype=bool)
    frontier_open = np.zeros((h, w), dtype=bool)
    frontier_close = np.zeros((h, w), dtype=bool)

    clusters = []
    outer_queue = deque([seed])
    map_open[seed] = True
    while outer_queue:
        y, x = outer_queue.popleft()
        if map_close[y, x]:
            continue
        map_close[y, x] = True

        if is_frontier_point(y, x) and not frontier_open[y, x] and not frontier_close[y, x]:
            # Inner BFS: flood the whole contiguous frontier region touching (y, x),
            # accumulating each cell's own unknown-neighbor offsets as we go.
            cells = []
            dir_x_sum = dir_y_sum = 0.0
            inner_queue = deque([(y, x)])
            frontier_open[y, x] = True
            while inner_queue:
                fy, fx = inner_queue.popleft()
                if frontier_close[fy, fx]:
                    continue
                frontier_close[fy, fx] = True
                cells.append((fy, fx))
                for ny, nx in neighbors8(fy, fx):
                    if unknown[ny, nx]:
                        dir_x_sum += nx - fx
                        dir_y_sum += ny - fy
                    elif (is_frontier_point(ny, nx) and not frontier_open[ny, nx]
                            and not frontier_close[ny, nx]):
                        frontier_open[ny, nx] = True
                        inner_queue.append((ny, nx))

            if len(cells) >= cfg.min_frontier_cells:
                ys, xs = zip(*cells)
                cy, cx = sum(ys) / len(ys), sum(xs) / len(xs)
                wx = grid.info.origin.position.x + (cx + 0.5) * resolution
                wy = grid.info.origin.position.y + (cy + 0.5) * resolution
                if (cfg.building_bounds_x[0] <= wx <= cfg.building_bounds_x[1]
                        and cfg.building_bounds_y[0] <= wy <= cfg.building_bounds_y[1]):
                    yaw = math.atan2(dir_y_sum, dir_x_sum) if (dir_x_sum or dir_y_sum) else 0.0
                    gain_m2 = unknown_gain_m2(int(round(cy)), int(round(cx)))
                    clusters.append((wx, wy, yaw, gain_m2))
                # else: outside the building -- see config/frontier_explorer.yaml's
                # building_bounds_* comment

        # Outer BFS only ever expands across free cells -- this is what bounds the whole
        # search to the robot's currently-reachable free-space component.
        for ny, nx in neighbors8(y, x):
            if free[ny, nx] and not map_open[ny, nx] and not map_close[ny, nx]:
                map_open[ny, nx] = True
                outer_queue.append((ny, nx))

    return clusters


def choose_goal(nav: BasicNavigator, clusters, robot_xy, blacklist, last_goal_xy,
                 cfg: SimpleNamespace):
    """Information-gain-aware goal selection, refined by real Nav2 path length.

    Each candidate's utility is gain_m2 - cfg.info_gain_weight * path_length_m (see
    config/frontier_explorer.yaml's info_gain_weight comment) -- a frontier that reveals
    a lot of new space can win over a nearer, low-value one, while info_gain_weight still
    bounds how far the robot will detour to chase gain. Getting an exact utility for every
    candidate would need a real Nav2 path for each one, which doesn't scale; instead this
    uses gain minus straight-line distance as a cheap proxy to pick the top
    cfg.candidate_k clusters, then only refines those with a real getPath() call -- same
    cost bound as the pure nearest-K heuristic this replaces, but ranked by utility
    instead of raw distance so gain gets a say in which candidates are even considered.

    Returns (x, y, yaw) or None -- yaw is each cluster's own unknown-ward facing direction
    from find_frontiers(), carried through unchanged (it doesn't affect path
    length/reachability scoring, only orientation).
    """
    clusters = [
        c for c in clusters
        if math.hypot(c[0] - robot_xy[0], c[1] - robot_xy[1]) > cfg.min_goal_dist_m
        and all(math.hypot(c[0] - bx, c[1] - by) > cfg.blacklist_radius_m for bx, by in blacklist)
    ]
    if not clusters:
        return None

    if last_goal_xy is not None:
        far_from_last = [
            c for c in clusters
            if math.hypot(c[0] - last_goal_xy[0], c[1] - last_goal_xy[1])
            > cfg.min_consecutive_goal_dist_m
        ]
        if far_from_last:
            clusters = far_from_last
        # else: every remaining candidate genuinely IS near the last goal
        # (e.g. finishing off a small room) -- fall back to the unfiltered
        # set rather than permanently blocking real, legitimate progress.

    def utility_proxy(c):
        wx, wy, _yaw, gain_m2 = c
        dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
        return gain_m2 - cfg.info_gain_weight * dist

    clusters.sort(key=utility_proxy, reverse=True)

    start = PoseStamped()
    start.header.frame_id = 'map'
    start.pose.position.x, start.pose.position.y = robot_xy
    start.pose.orientation.w = 1.0

    best, best_utility = None, -math.inf
    for wx, wy, yaw, gain_m2 in clusters[:cfg.candidate_k]:
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
        utility = gain_m2 - cfg.info_gain_weight * length
        if utility > best_utility:
            best, best_utility = (wx, wy, yaw), utility
    return best


def wait_for_task(nav: BasicNavigator, tf_node: MapTF, timeout_s: float):
    """Spin until a BasicNavigator task (goToPose/backup/spin/...) finishes or times out."""
    deadline = time.time() + timeout_s
    while not nav.isTaskComplete() and time.time() < deadline:
        rclpy.spin_once(tf_node, timeout_sec=0.5)
    if time.time() >= deadline:
        nav.cancelTask()


def unstick(nav: BasicNavigator, tf_node: MapTF, cfg: SimpleNamespace):
    """Back up and spin in place -- for when the robot looks physically stuck.

    clearAllCostmaps() first: several stuck episodes this session traced back
    to stale "occupied" cost lingering near the robot (self-hit noise from the
    walking gait, or slow-to-clear voxels under load -- see nav2_params.yaml's
    own comments), which can make the controller see a wall that isn't really
    there anymore. Backing up then turning gives both the costmap and the
    frontier search a genuinely fresh look before the main loop tries again.
    """
    nav.get_logger().warn(
        f'{cfg.stuck_after_consecutive_failures} goals in a row failed -- looks physically '
        f'stuck, not just a bad frontier. Backing up and turning around.')
    nav.clearAllCostmaps()
    nav.backup(backup_dist=cfg.unstuck_backup_dist_m, backup_speed=0.05, time_allowance=10)
    wait_for_task(nav, tf_node, timeout_s=15)
    nav.spin(spin_dist=cfg.unstuck_spin_dist_rad, time_allowance=10)
    wait_for_task(nav, tf_node, timeout_s=15)


def main():
    rclpy.init()
    tf_node = MapTF()
    cfg = load_config(tf_node)

    nav = BasicNavigator()
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

        if time.time() - t_start > cfg.max_runtime_s:
            nav.get_logger().info(f'Exploration budget exhausted ({cfg.max_runtime_s}s elapsed).')
            break
        if attempts >= cfg.max_attempts:
            nav.get_logger().info(f'Exploration budget exhausted ({cfg.max_attempts} goal attempts).')
            break

        if tf_node.latest_map is None:
            continue

        # robot_xy is needed before find_frontiers() now (it seeds the wavefront BFS at
        # the robot's own map cell), not just for choose_goal() afterward -- so this TF
        # lookup moved ahead of frontier detection.
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

        robot_rc = world_to_grid(tf_node.latest_map, robot_xy[0], robot_xy[1])
        clusters = find_frontiers(tf_node.latest_map, robot_rc, cfg)
        if not clusters:
            nav.get_logger().info('No frontiers left -- map fully explored.')
            break

        chosen = choose_goal(nav, clusters, robot_xy, blacklist, last_goal_xy, cfg)
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
                f'{cfg.stuck_after_consecutive_failures} before unstick attempt); retrying next loop.')
            if consecutive_failures >= cfg.stuck_after_consecutive_failures:
                unstick(nav, tf_node, cfg)
                consecutive_failures = 0
            else:
                time.sleep(cfg.retry_backoff_s)
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
        # backoff ever kicking in (cfg.retry_backoff_s never actually applied,
        # since the stale future was often already "done").
        if accepted:
            wait_for_task(nav, tf_node, timeout_s=cfg.goal_timeout_s)
            succeeded = nav.getResult() == TaskResult.SUCCEEDED
        else:
            nav.get_logger().warn(f'Goal to ({gx:.2f}, {gy:.2f}) was rejected outright.')
            succeeded = False

        if succeeded and (time.time() - goal_sent_at) < cfg.instant_success_s:
            # See config/frontier_explorer.yaml's instant_success_s comment --
            # this is min_goal_dist_m's backstop, not the primary fix, so it's
            # rare in practice. Falls through to the existing failure-handling
            # branch below (retry count -> eventual blacklist) instead of
            # duplicating that logic.
            nav.get_logger().warn(
                f'Goal at {goal_xy} "succeeded" in under {cfg.instant_success_s}s -- too fast for '
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
            if retry_counts[key] > cfg.max_retries_per_goal:
                nav.get_logger().warn(
                    f'Goal at {goal_xy} failed {retry_counts[key]} times -- blacklisting it.')
                blacklist.append(goal_xy)
            else:
                nav.get_logger().warn(
                    f'Goal at {goal_xy} failed after {elapsed:.1f}s '
                    f'(attempt {retry_counts[key]}/{cfg.max_retries_per_goal}) -- retrying shortly.')
                time.sleep(cfg.retry_backoff_s)

            if consecutive_failures >= cfg.stuck_after_consecutive_failures:
                unstick(nav, tf_node, cfg)
                consecutive_failures = 0

    nav.lifecycleShutdown()
    tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
