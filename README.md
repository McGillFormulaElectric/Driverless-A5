# MFE Driverless — Assignment 5: Path Planning & Control

This assignment focuses on trajectory planning and vehicle control. You'll implement two complementary algorithms for autonomous navigation:

- **A5.1** — **Centerline Planning**: compute the optimal path centerline from cone-detected boundaries.
- **A5.2** — **Pure Pursuit Controller**: track the planned centerline using a geometric control law.

Both run alongside a grader that validates your path and control outputs via `/grader/feedback`.

> **Grading Setup** — The grader runs as a local Docker service alongside your student code. Both use host networking and ROS domain ID 42 for automatic DDS discovery.

---

## 1. Getting started

### 1.1 GitHub
1. Go to this repo on GitHub.
2. Create a new branch named after you, e.g. `NeilJoeGeorge`.
3. Clone and check out your branch:

```bash
git clone <repo-url>
cd Driverless-AA5
git checkout <FirstNameLastName>
```

### 1.2 Docker & Local Grading
The grader runs as a local service inside Docker alongside your student code. Both use host networking and ROS domain ID 42 for automatic DDS discovery.

```bash
cd docker
docker compose -f docker-compose-local.yml build
docker compose -f docker-compose-local.yml up -d
```

This starts two services:
1. **student** — your code (subscriber + publisher)
2. **grader** — reference implementation (signal publisher + grader)

Both services share the same network and ROS domain, so topics auto-discover via DDS.

Inside either container, the workspace is mounted at `/workspace` (your `ros2_ws`). Build and source:

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

View logs from either service:
```bash
docker compose -f docker-compose-local.yml logs student -f  # tail student logs
docker compose -f docker-compose-local.yml logs grader -f   # tail grader logs
docker compose -f docker-compose-local.yml logs             # both services
```

Stop everything:
```bash
docker compose -f docker-compose-local.yml down
```

---

## 2. A5.1 — Centerline Planning

**Goal:** compute the track centerline from cone positions and publish it as a list of waypoints.

The grader publishes cone positions on `/grader/cones` (custom message with blue and yellow cone arrays). Your job is to:
1. Subscribe to cone positions.
2. Compute the centerline by averaging blue and yellow cones (pairing them spatially).
3. Publish ordered waypoints on `/${GITHUB_USER}/centerline` (geometry_msgs/PoseArray).

### Algorithm outline
For each blue cone, find the nearest yellow cone; the midpoint is a centerline point. Order the waypoints front-to-back relative to the vehicle. Theory refs:
- Cone pairing (nearest-neighbour matching)
- Greedy path ordering to avoid loops
- Waypoint smoothing (optional but recommended)

### Implementation
Open `ros2_ws/src/a5_new_member/a5_new_member/planner_node.py`. There's a `TODO` block where you implement:
1. **`_compute_centerline()`** — pair cones, compute midpoints, order waypoints.
2. Publish to `/${GITHUB_USER}/centerline` as a PoseArray.
3. Handle edge cases (no cones, unpaired cones, etc.).

### Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a5_new_member planner.launch.py github_user:=$GITHUB_USER
```

Watch the grader feedback:
```bash
ros2 topic echo /grader/feedback
```

The grader checks that your centerline is smooth, ordered, and well-spaced.

Grader feedback will confirm your centerline is correct.

---

## 3. A5.2 — Pure Pursuit Path Tracking Controller

Implement a geometric control law to steer the vehicle along the planned centerline.

### Algorithm
**Pure Pursuit** is a cross-track error minimization algorithm:
1. Identify the "lookahead point" (L steps ahead on the centerline).
2. Compute steering angle to drive the vehicle toward that point.
3. Publish steering command.

**Steering law:**
```
δ = atan2(2 * L * sin(θ_error), v * k_a)
```
where:
- `L` — lookahead distance (tuning parameter)
- `v` — vehicle speed
- `θ_error` — angular error to lookahead point
- `k_a` — speed-dependent damping constant

### Implementation
Open `ros2_ws/src/a5_new_member/a5_new_member/controller_node.py`. There's a `TODO` block where you implement:
1. Subscribe to `/${GITHUB_USER}/centerline` (from A5.1).
2. Subscribe to odometry (vehicle state).
3. Compute lookahead point and steering angle.
4. Publish steering command on `/${GITHUB_USER}/cmd` (ackermann_msgs/AckermannDriveStamped).

### Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a5_new_member controller.launch.py github_user:=$GITHUB_USER
```

Watch the grader feedback:
```bash
ros2 topic echo /grader/feedback
```

The grader evaluates your cross-track error and heading accuracy against the centerline.

Grader feedback will show cross-track error and control metrics.

---

## 4. Topic contract (summary)

| Topic              | Type              | Owner   | Purpose                            |
| ------------------ | ----------------- | ------- | ---------------------------------- |
| `/grader/cones`     | Custom (ConeArray) | Neil    | Blue/yellow cone positions for A5.1 |
| `/grader/odom`      | `nav_msgs/Odometry` | Neil   | Vehicle state for controller input |
| `/grader/feedback`  | `std_msgs/String` | Neil    | Per-student grading verdict        |
| `/<user>/centerline` | `geometry_msgs/PoseArray` | Student | A5.1 centerline waypoints        |
| `/<user>/cmd`       | `ackermann_msgs/AckermannDriveStamped` | Student | A5.2 steering command             |

---

## 5. Layout

```
Driverless-A5/
├── docker/
│   ├── docker-compose-local.yml  # Single container: grader + student
│   ├── Dockerfile                # ROS2 Humble + dependencies
│   └── (entrypoint script)
├── ros2_ws/
│   └── src/
│       ├── a5_new_member/        # your template — write code here
│       │   ├── a5_new_member/
│       │   │   ├── planner_node.py     # A5.1
│       │   │   └── controller_node.py  # A5.2
│       │   ├── launch/
│       │   │   ├── planner.launch.py
│       │   │   └── controller.launch.py
│       │   ├── config/params.yaml
│       │   ├── setup.py
│       │   └── package.xml
│       └── a5_grader/            # grader (runs in same container)
│           ├── a5_grader/grader.py
│           ├── config/params.yaml
│           ├── launch/grader.launch.py
│           ├── setup.py
│           └── package.xml
└── README.md
```

## 6. Troubleshooting

- **`ros2 topic list` doesn't show `/grader/cones`.** Ensure both grader and student are running in the container. Check `docker compose logs`.
- **Topics visible but grader shows no feedback.** Verify your centerline and command are publishing to correct topics (`/<user>/centerline` and `/<user>/cmd`).
- **Centerline is jagged or backwards.** Check your cone-pairing logic and waypoint ordering. Visualize with Foxglove to debug.
- **Controller steers wildly.** Check lookahead distance and speed scaling. Start with conservative tuning parameters.

---

## 7. Submission Checklist
- [ ] `planner_node.py` implements centerline computation from cones.
- [ ] `controller_node.py` implements pure pursuit steering law.
- [ ] Both nodes publish to correct topics with correct types.
- [ ] Grader feedback is positive for both A5.1 and A5.2.
- [ ] Code is committed to your branch.

## 8. Parameters

All constants are exposed as ROS parameters in YAML files:

- **Grader** [`ros2_ws/src/a5_grader/config/params.yaml`](ros2_ws/src/a5_grader/config/params.yaml):
  - Cone positions and noise
  - Track geometry
  - Grading tolerances
  - Feedback periods

- **Student** [`ros2_ws/src/a5_new_member/config/params.yaml`](ros2_ws/src/a5_new_member/config/params.yaml):
  - Lookahead distance `L` for pure pursuit
  - Speed damping constant `k_a`
  - Cone pairing tolerance
  - Waypoint filtering (smoothing parameters)

You should not edit grader parameters. For student parameters, start with suggested values and tune if needed. The launch files load these automatically.

---

## 🔴 NEIL REFERENCE

**To view complete solutions for this assignment:**

```bash
# View the reference implementation on the solution branch
git checkout solution/a5-student-code

# Or clone directly from the solution branch for testing
git clone -b solution/a5-student-code <repo-url>
```

**Solution Branch Reference:** [`solution/a5-student-code`](https://github.com/McGillFormulaElectric/Driverless-A5/tree/solution/a5-student-code)

**Pull Request:** [PR #2 - A5 Solution](https://github.com/McGillFormulaElectric/Driverless-A5/pull/2)

