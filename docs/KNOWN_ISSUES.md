# Known issues

Open bugs, plus a couple of already-fixed ones kept here with their root causes since the
debugging story is often more useful than the one-line fix. See the
[README](../README.md) for the overall project.

- ~~Robot sometimes tips over within a few seconds of spawning~~ **fixed.** Root cause
  found by watching actual joint commands, not just the body's pose: every leg joint spawns
  at 0 (fully extended) by default, but the IK gait controller commands its real
  ~0.86-1.89 rad standing-stance target from the very first control tick regardless — so
  position control snaps all 12 joints from 0 toward that very different target at once,
  while gravity is already acting on the body. Confirmed live via `joint_states` sampling
  that the *commanded* stance was already correct and static within about 1s of spawn, yet
  the *body* kept visibly tipping for several more seconds afterward — this idle stance has
  no active balance feedback at all (legs just hold a fixed angle relative to the trunk), so
  once that initial snap perturbed the body even slightly, nothing corrected it and physics
  alone finished the fall. Fixed by giving every leg joint an `initial_value` in
  `go2_description/xacro/gazebo.xacro` matching that exact stance (so there's no snap to
  begin with), paired with `office_sim.launch.py`'s `z_pose` raised to 0.30 (empirically
  confirmed, not simply `RobotController.py`'s documented 0.25m — that undershot the real
  geometric clearance for these joint angles and caused instant ground interpenetration).
  Confirmed live with both fixes together: settles to an essentially perfectly level pose
  within 1-2s and stays rock solid over 20+s of testing.
- **Nav2 bringup can stall on a loaded machine.** A lifecycle node's `change_state` service
  response can get lost at the DDS layer under CPU contention (confirmed live: a node
  finished configuring internally but `lifecycle_manager` never got the acknowledgment,
  logged as `failed to send response ... at rmw_response.cpp:153`), leaving bringup stuck
  on that one node forever — no timeout, no retry. Gazebo's GUI alone can pin ~2 cores, and
  the worst spike is right at cold-start when Gazebo + KISS-ICP + `slam_toolbox` + all of
  Nav2's ~10 nodes initialize simultaneously. If bringup stalls with a node stuck
  "Configuring" and no further log output, kill and retry `nav_stack.launch.py` after
  letting `office_sim.launch.py`/`kiss_icp.launch.py` settle for a few seconds first — see
  the [Quick start](../README.md#quick-start) note.
- **`slam_toolbox` keeps its whole map/pose-graph in memory only, no disk persistence** —
  if it falls behind real-time for long enough (its own message-filter queue log-spams
  `discarding message because the queue is full`) it can die and silently restart from a
  blank map, which looks exactly like the map "resetting" rather than a process crash.
  Mitigated (not eliminated) via `slam.yaml`'s `throttle_scans: 2` (halves its per-scan
  compute cost) and a larger `scan_queue_size` (bigger shock absorber for transient stalls
  like loop-closure searches) — worth revisiting if it recurs. Saving the map to disk
  periodically (`map_saver_cli`, not yet wired into a script) would make this recoverable
  instead of just less likely; see `autonomous_exploration_plan.adoc`'s Step 5.
- **`frontier_explorer.py` and `record_bag.sh` are unimplemented placeholders**, not
  oversights — full design for both (frontier detection + goal selection loop,
  `nav2_simple_commander`-based; a straightforward `ros2 bag record` one-liner) is written
  up in `autonomous_exploration_plan.adoc`, just not built yet. Manual Nav2 goals and
  manual-teleop SLAM both work fully today in the meantime.
- **The 3D lidar's scan pattern isn't physically representative of a real Livox Mid-360** —
  `gpu_lidar` gives a uniform raster, not Mid-360's non-repetitive rosette pattern. Point
  *density* looks different from a real unit; the geometry (FOV, range) is right and it's
  fine for SLAM.
- **No headless rendering.** `eglInitialize` fails against `/dev/dri/card1` on this machine
  (driver/headless EGL mismatch), so both the RGBD camera and lidar — being *rendering*
  sensors — require a real X display attached (`DISPLAY=:1` here is the machine's actual
  local screen, confirmed via `Xorg` on `seat0`/`tty2` — not a remote or virtual session).
  `gz sim -s` alone will not produce sensor data on this box.
- **Depth camera far clip is 25 m**, not a "real" RealSense spec value — it was originally
  10 m and got extended after depth rendering went blank specifically in the office's larger
  rooms. Root cause: rays exceeding the far clip return `+Inf`, and RViz's `Image` display
  auto-normalizes min/max across the whole frame — one `Inf` in that computation poisons the
  normalization constant for *every* pixel, not just the out-of-range ones. 25 m covers the
  room's full diagonal with margin.
