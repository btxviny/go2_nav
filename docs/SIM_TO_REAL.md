# Sim-to-Real: Sensor Fidelity and Networking Gaps

This repo is currently **simulation-only** — Gazebo Harmonic, single process, single
machine, no real-robot code path exists yet. This doc is a pre-flight reference for when
that changes: where the simulated sensors diverge from the real Go2 EDU's hardware, and
what the real deployment's networking actually looks like, so the nav/SLAM stack isn't
seeing its first packet loss or its first multi-machine DDS hop on the physical robot.

The real-hardware details below are condensed from a separate, private setup repo
(`btxviny/go2_setup`) that documents the actual Go2 EDU dock, sensors, and PC connectivity
used there — treat that repo as the source of truth if anything here goes stale.

> **Note:** the main `README.md` links to `docs/ARCHITECTURE.md` and `docs/PATCHES.md` in
> several places; neither currently exists in this repo. Don't treat citations to those
> files (here or elsewhere) as verifiable until they're written.

---

## 1. Real robot: sensor and network architecture (reference)

**Sensors**, run as native ROS2 Humble nodes inside a Docker container on the Go2's
expansion dock (`network_mode: host`, `ROS_DOMAIN_ID=0` — same domain as the robot's own
built-in nodes):

- **RealSense D435i** — RGB + depth. Color is streamed/recorded **compressed** (JPEG,
  ~30Hz); depth stays **raw** (~24-28Hz) because its compressed transport
  (`compressedDepth`) collapses to ~0.7Hz on this hardware — PNG-encoding 16-bit depth is
  too CPU-expensive on the dock's Jetson to keep up. Topics live under a doubled
  `/camera/camera/...` namespace (a `realsense-ros` default).
- **Hesai PandarXT-16** — 3D lidar, `/rslidar_points` (`sensor_msgs/PointCloud2`).
- **Unitree built-in L1 lidar** — always-on, `/utlidar/cloud` (~15Hz) — not used by this
  sensor pipeline, but present on the robot regardless.

**Topology:** a controller PC connects to the dock over a dedicated subnet
(`192.168.123.0/24`) via Ethernet — static IPs on both ends, the robot's own main computer
reserved at a fixed address. The dock is *also* dual-homed onto WiFi, but **only** for
SSH / `docker` control / file transfer, never for live sensor streaming — see §3.

**DDS domain-bridging pattern:** sensors publish on domain 0 (the robot's own domain). The
controller PC's own tools (RViz, `ros2` CLI) run isolated on a separate domain instead of
joining domain 0 directly, so they don't pollute the robot's own discovery traffic. A
`domain_bridge` process on the PC opens both domains at once and relays only an explicit
topic whitelist from one to the other, over the Ethernet link.

**QoS:** every bridged sensor topic is forced to **`best_effort`**. Reliable QoS across the
real network hop was confirmed, live, to stall an entire stream on a single dropped UDP
fragment (CycloneDDS retries that exact sample before delivering anything newer) — this hit
raw depth (~1.8MB/frame), the point cloud, and even JPEG-compressed color frames.

**Known real-hardware failure modes** worth designing around:
- A firewall (UFW) with a default-deny policy silently drops all DDS UDP traffic before it
  reaches a socket — `ping`/`tcpdump` look completely healthy, DDS just sees nothing.
- USB bandwidth contention on the RealSense: the depth stream fails to start if the IR
  streams are also enabled; fixed by disabling infra1/infra2 and forcing a hardware reset
  at launch (`initial_reset:=true`).
- Intermittent depth-stream USB hardware errors, still unresolved on that hardware.

---

## 2. Sim sensor fidelity vs. real hardware

| Sensor | Sim today | Real hardware | Gap |
|---|---|---|---|
| Lidar | Livox Mid-360-style FOV/pattern, 7Hz, range capped at 15m (tied to Nav2's costmap, not sensor spec), 2cm Gaussian range noise | Hesai PandarXT-16, ~9-10Hz as delivered to the controller PC | Sim isn't modeling the actual lidar's FOV or scan pattern at all — it's standing in a different sensor |
| RGB camera | D435-style HFOV, 848×480, 15Hz, no compression modeled | RealSense D435i, ~30Hz, **streamed compressed** over the real link | Rate mismatch, and sim has no bandwidth/compression constraint to test against |
| Depth camera | The sim sensor's `<clip>` tag is confirmed **non-functional** on this Gazebo build — no real range limiting or noise, and depth isn't consumed by SLAM/Nav2 at all today | ~24-28Hz raw; compressed transport unusable (~0.7Hz) | Sim depth is currently fidelity-free and functionally decorative |
| IMU | 100Hz, generic Gazebo-tutorial noise/bias values | Real IMU inside the RealSense/dock, uncharacterized here | Noise model has never been validated against real hardware |

---

## 3. Networking: what sim has none of

Sim runs everything in a single process on one machine — no `ROS_DOMAIN_ID` separation, no
DDS bridging, no RMW/QoS configuration anywhere, default (Reliable) QoS throughout. None of
this is visible in sim today because there's no network hop for it to matter on. Concrete
things the real deployment will require that the current stack has never been exercised
against:

- **Ethernet only for sensor streaming.** This isn't a preference — the real dock's WiFi
  link was confirmed, live, unable to carry DDS discovery at all (0 participants found in
  10s of listening over WiFi vs. instant over Ethernet), because the WiFi router's
  AP/client isolation blocks the multicast traffic DDS discovery depends on by default.
  SSH/`docker`/file-transfer still work fine over WiFi since they don't need multicast —
  only live topic data doesn't survive that link. Plan the controller PC's connection to
  the robot as Ethernet, full stop; WiFi is not a fallback for sensor topics.
- **Reliable QoS will silently stall large messages over a real link.** Point clouds and
  raw images are exactly the message sizes that hit this. If/when this stack talks to real
  sensors over an actual network hop (rather than in-process), lidar/camera topics should
  be `best_effort`, not the current default.
- **Static IP + firewall checklist**, before the first real connection attempt:
  - Persistent (not transient) static IP on the controller PC's Ethernet interface, on
    whatever subnet the robot/dock use.
  - Confirm no firewall (e.g. UFW) is silently dropping inbound UDP on that interface —
    `ping` succeeding does not rule this out.
  - Never assign the robot's own reserved IP to anything else on the subnet.
- **Domain bridging / containerized-driver pattern.** If real sensors end up running in a
  container on the robot's own compute (as they do in the reference setup) while this
  stack's nav/SLAM runs on a separate controller PC, these launch files will need either to
  join whatever domain the sensors publish on, or run behind an equivalent domain-bridge
  step — the current launch files all assume a single machine/process and don't yet have
  this concept.
- **Untested latency/jitter tolerance.** This repo's KISS-ICP tuning was chosen specifically
  to avoid a TF-buffer-staleness failure mode seen in sim under upstream's default (looser)
  settings — that class of bug is exactly what an intermittent real network hop tends to
  provoke more of, and it's never been tested under real jitter/packet loss here.

---

## 4. Recommended next steps, in priority order

1. **Model the bandwidth-relevant constraints in sim now**, even single-machine: bridge the
   camera topic via compressed transport where the real pipeline would use it, and set
   `best_effort` QoS on the lidar/camera topics in `bridge.yaml`/launch files, so the
   nav stack is exercised under the QoS it'll actually get on hardware.
2. **Fix or replace the non-functional depth `<clip>`** — either patch the sim sensor
   properly or add a downstream range-filtering node — since depth currently has no real
   fidelity and isn't consumed by anything.
3. **Re-check lidar/camera rate and model choices against the actual hardware specs**
   (Hesai PandarXT-16's native rate, RealSense D435i's real color/depth rates) instead of
   the current Livox/generic-D435 placeholders. Where a mismatch is intentional (e.g. the
   lidar range cap tied to Nav2's costmap rather than sensor spec), document it as such.
4. **Stretch goal: a two-process sim mode** — sensor-publishing nodes in one
   `ROS_DOMAIN_ID`/process, nav/SLAM in another, bridged the same way the real setup does
   it. This is the only way to actually exercise domain-bridging, QoS, and latency behavior
   before touching hardware.
5. **Real-robot networking pre-flight checklist**, ready to follow once the physical robot
   is available: static IP setup, firewall check, Ethernet-only for sensors — adapt from
   the reference setup repo's own network-setup script rather than re-deriving it.
