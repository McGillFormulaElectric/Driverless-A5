# MFE Driverless — Assignment 5: Path planning + pure-pursuit control

This is the capstone of the onboarding series. You've done YOLO (A1), ROS 2
pub/sub + IIR filtering (A2), TF2 (A3), and EKF (A4). Now you get to actually
**drive** a car around a track. Neil runs a lightweight 2D bicycle
sim on the class Tailscale network, publishes a cone map, and grades every
student's laps live.

Like the earlier assignments this is split [Advent-of-Code style](https://adventofcode.com/):

- **A5.1** — subscribe to the cone map, compute the ordered centerline, and
  publish it as a `nav_msgs/Path`.
- **A5.2** — implement a **pure-pursuit** lateral controller and a simple
  speed schedule so your car completes at least one clean lap of the track
  in under 60 s.

Everything runs inside a Docker container so your local OS / Python version
don't matter.

---

## 1. Getting started

### 1.1 GitHub
1. Go to this repo on GitHub.
2. Create a new branch named after you, e.g. `NeilJoeGeorge`.
3. Clone and check out your branch:

```bash
git clone <repo-url>
cd Driverless-A5
git checkout <FirstNameLastName>
```

### 1.2 Tailscale (class VPN)
Neil runs the sim + grader on the class Tailscale network. Every
student joins the same tailnet so DDS discovery works between machines.

1. Install Tailscale: <https://tailscale.com/download>.
2. `sudo tailscale up` and sign in with the invite Neil sent.
3. Verify: `tailscale ping neil` (hostname will be shared in class).
4. Note your own Tailscale hostname/IP — you'll set it via env var if
   auto-detection fails.

### 1.3 Docker
Linux host with Docker + Docker Compose is the supported path (host
networking + Tailscale interface work cleanly).

```bash
cd docker
export GITHUB_USER=<your-github-handle>              # required
export A5_NEIL_HOST=<professor tailnet host>    # e.g. neil.tail1234.ts.net
docker compose build
docker compose run --rm student
```

Inside the container you'll have `/workspace` mounted to `ros2_ws/`. Build
and source:

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

> **macOS/Windows caveat:** Docker Desktop's `network_mode: host` is limited.
> If you're not on Linux, run the container with `--network host` on a Linux
> VM, or use Tailscale's [userspace networking mode](https://tailscale.com/kb/1112/userspace-networking)
> inside the container.

---

## 2. Topic contract

| Topic                   | Type                             | Owner    | QoS                           | Purpose                                |
| ----------------------- | -------------------------------- | -------- | ----------------------------- | -------------------------------------- |
| `/neil/cone_map`   | `geometry_msgs/PoseArray`        | Neil | RELIABLE + TRANSIENT_LOCAL    | Latched cone map (blue + yellow)       |
| `/neil/sim_stats`  | `std_msgs/String` (JSON)         | Neil | RELIABLE + TRANSIENT_LOCAL    | Per-user lap / cone-hit stats for grader |
| `/neil/feedback`   | `std_msgs/String`                | Neil | RELIABLE                      | Per-student grading verdict            |
| `/<user>/state`         | `nav_msgs/Odometry`              | Neil | RELIABLE                      | Sim state of *this* student's car      |
| `/<user>/centerline`    | `nav_msgs/Path`                  | Student  | RELIABLE                      | A5.1 output                            |
| `/<user>/cmd`           | `ackermann_msgs/AckermannDrive`  | Student  | RELIABLE                      | A5.2 output (drive commands, 50 Hz)    |

All pubs/subs use the class-standard QoS
`QoSProfile(reliability=RELIABLE, history=KEEP_LAST, depth=10)`. The two
latched topics additionally set `durability=TRANSIENT_LOCAL` — your
subscriber must match that durability or DDS will silently drop the
connection. See `a5_solution/planner_node.py` for a working example.

### 2.1 Cone-color encoding
Neil's sim packs cone color into a `geometry_msgs/Pose` because
`PoseArray` is a stock message type:

```
pose.position.x, pose.position.y  ->  cone position in the map frame
pose.orientation.z == 0.0         ->  blue  (left boundary)
pose.orientation.z == 1.0         ->  yellow (right boundary)
pose.orientation.w == 1.0         ->  constant filler
```

**This is not a valid quaternion.** Do not try to interpret it as a
rotation. It's a compact per-cone label; parse it as such (see
`planner_node.py::_on_cones`).

### 2.2 Vehicle / sim parameters (matches MFE-Driverless-V1)
- Wheelbase `L = 1.56 m`
- `|steering_angle| <= 0.5 rad` (clamped by the sim)
- `0 <= speed <= 15 m/s` (clamped by the sim)
- Sim integrator: standard bicycle kinematics at 50 Hz
  (`x += v cos θ dt; y += v sin θ dt; θ += v/L tan δ dt`)
- Cone hit radius: `0.4 m`

---

## 3. A5.1 — Centerline planning

**Goal:** subscribe to `/neil/cone_map`, compute the ordered centerline
of the track, and publish it on `/${GITHUB_USER}/centerline`
(`nav_msgs/Path`, `map` frame). Grader threshold: **every point within
1.0 m of the true centerline, and at least 30 points.**

Template in `ros2_ws/src/a5_solution/a5_solution/planner_node.py`. The
`TODO(student)` block inside `_compute_centerline` is where you fill in the
plan. A minimal approach:

1. For each blue cone, find its nearest yellow cone.
2. Midpoint of that pair = raw centerline point.
3. Order the midpoints (greedy nearest-neighbour is fine on this track).
4. Publish as `Path`.

If you're feeling ambitious, use `scipy.spatial.Delaunay` on the combined
cone set and keep only the edges that connect a blue to a yellow cone.
Either passes.

### Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a5_solution planner.launch.py github_user:=$GITHUB_USER
```

Watch for the verdict:
```bash
ros2 topic echo /neil/feedback
```

You should see:
```
[A5.1] Congrats <your-github-user>, the answer is correct
```

---

## 4. A5.2 — Pure-pursuit driving

**Goal:** subscribe to your own `/${GITHUB_USER}/centerline` and
`/${GITHUB_USER}/state`, and publish drive commands on
`/${GITHUB_USER}/cmd` at 50 Hz using a **pure-pursuit** lateral controller
plus a simple curvature-scheduled speed profile.

Grader threshold: **>= 1 complete lap in <= 60 s with zero cone hits.**

Template in `ros2_ws/src/a5_solution/a5_solution/controller_node.py`. The
`TODO(student)` block inside `_pure_pursuit` walks you through the algorithm:

1. Find the closest waypoint.
2. Walk forward along the path until you find the first waypoint
   `>= LOOKAHEAD` away (loop around the end). That's the goal.
3. Transform the goal into the vehicle body frame.
4. Steering: `delta = atan2(2 * L * local_y, LOOKAHEAD**2)`.
5. Speed: back off `MAX_SPEED` as the local curvature grows.

Sensible starting values: `LOOKAHEAD = 4 m`, `MIN_SPEED = 3 m/s`,
`MAX_SPEED = 10 m/s`. Tuning is fair game.

### References
- Wikipedia — [Pure pursuit](https://en.wikipedia.org/wiki/Pure_pursuit).
- Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path Tracking
  Algorithm* (CMU-RI-TR-92-01).

### Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a5_solution controller.launch.py github_user:=$GITHUB_USER
```

This launch file brings up both the planner and the controller. Watch:
```bash
ros2 topic echo /neil/feedback
```

Expected verdict:
```
[A5.2] Congrats <your-github-user>, the answer is correct
```

---

## 5. Visualizing with Foxglove Studio (strongly recommended)

Watching your car crash into cones is the fastest way to debug a bad
pure-pursuit gain.

### 5.1 Bridge (inside the container)
```bash
ros2 run foxglove_bridge foxglove_bridge port:=8765
```
Leave that terminal running.

### 5.2 Studio (on your host)
Download from <https://foxglove.dev/download>.

### 5.3 Connect
1. Open Foxglove Studio -> **Open connection...** -> **Foxglove WebSocket**.
2. URL: `ws://localhost:8765` (or `ws://<your-tailscale-host>:8765`).
3. Add a **3D** panel. Recommended layers:
   - `/neil/cone_map` (PoseArray). Color the arrows by
     `orientation.z` if you want blue/yellow to render correctly.
   - `/<GITHUB_USER>/state` (Odometry) — your car.
   - `/<GITHUB_USER>/centerline` (Path) — your planned line.
4. Add a **Raw Messages** panel on `/neil/feedback` for verdicts.

> Tip: save your layout to `submissions/a5_layout.json`
> (**Layout -> Export**) so future teammates can reuse it.

---

## 6. Layout

```
Driverless-A5/
├── docker/                          # Dockerfile, compose, CycloneDDS config, entrypoint
├── ros2_ws/
│   └── src/
│       ├── a5_solution/              # your template — this is where you write code
│       │   ├── a5_solution/planner_node.py     (A5.1)
│       │   ├── a5_solution/controller_node.py  (A5.2)
│       │   └── launch/{planner,controller}.launch.py
│       └── a5_neil/            # for reference; not run by students
│           ├── a5_neil/sim_node.py       (bicycle sim + cone map)
│           ├── a5_neil/grader.py         (verdicts on /neil/feedback)
│           ├── config/track.yaml              (~40 cones, closed loop)
│           └── launch/neil.launch.py
└── README.md
```

## 7. Troubleshooting

- **`ros2 topic list` doesn't show `/neil/cone_map`.** DDS discovery
  isn't reaching Neil's node. Confirm `tailscale ping <professor-host>`
  works, `A5_NEIL_HOST` is set, and `ROS_DOMAIN_ID` matches (`42`).
- **`ros2 topic echo /neil/cone_map` prints nothing forever.** Your
  subscription's QoS is likely mismatched. `/neil/cone_map` is
  latched with `TRANSIENT_LOCAL` — use `--qos-durability transient_local`
  on the CLI, and in code copy the `LATCHED_QOS` profile from
  `planner_node.py`.
- **Sim state topic `/<user>/state` never appears.** The professor spawns
  a sim for you only after it discovers your `/<user>/cmd` topic. Start
  publishing (even zeros) from the controller and wait ~2 s.
- **You complete a lap but the grader still says incorrect.** Check
  `/neil/sim_stats` — it publishes JSON per user. If `cone_hits > 0`
  the lap doesn't count.

---

## 8. Submission
1. Commit your changes to your `FirstNameLastName` branch.
2. Include both feedback screenshots in `submissions/`:
   - `submissions/a5_1_feedback.png`
   - `submissions/a5_2_feedback.png`
3. Open a pull request against `main`.
