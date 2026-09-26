# MFE Driverless — Assignment 5: ROS 2 pub / sub with Local Grading

This assignment introduces ROS 2 publishers, subscribers, namespaces, and DDS discovery. It is split into two parts: Part 1 is a warm-up, Part 2 is the real challenge.

- **A5.1** — publish `Hello World!` on your own namespaced topic.
- **A5.2** — subscribe to a noisy signal, filter it with a first-order IIR low-pass filter, and publish your filtered output. The grader runs locally alongside your code and auto-discovers your topic, reporting back on `/neil/feedback` whether you got it right.

Everything runs inside a Docker container using `docker-compose-local.yml`.

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

## 2. A5.1 — Hello World! (warm-up)

**Goal:** publish the string `Hello World!` on `/${GITHUB_USER}/hello` at 1 Hz.

Open `ros2_ws/src/a5_new_member/a5_new_member/hello_publisher.py`. The node, publisher, and timer are already wired up — there's a `TODO` block inside `_tick` where you build and publish a `std_msgs/String`. If you're new to ROS 2 publishers, see the [ROS2 Industrial Workshop — Simple Publisher/Subscriber](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/basics/ROS2-Simple-Publisher-Subscriber.html). Then launch it with your GitHub username as the ROS namespace:

```bash
ros2 launch a5_new_member hello.launch.py github_user:=$GITHUB_USER
```

Neil's grader is watching for any topic matching `/<user>/hello` (type `std_msgs/String`). When it sees `Hello World!` from your namespace, it will publish on `/neil/feedback`:

```
Hello <your-github-user>
```

Watch the feedback live from another terminal (inside the container). `/neil/feedback` is shared by every student on the tailnet, so once the class is connected simultaneously you'll want to filter to just your own handle:
```bash
ros2 topic echo /neil/feedback | grep --line-buffered "$GITHUB_USER"
```
(Drop the `grep` to see everyone's verdicts — useful for confirming the grader is alive at all.)

**Deliverable for A5.1:** a screenshot of `/neil/feedback` congratulating your GitHub handle, committed to your branch under `submissions/a5_1_feedback.png`.

---

## 3. A5.2 — Low-pass filter Neil's signal

Neil publishes a deterministic-but-noisy waveform on `/neil/signal` (`std_msgs/Float32`):

```
x(t) = 1.0 * sin(2π * 0.5 * t) + 0.6 * sin(2π * 5.0 * t) + N(0, 0.3²)
```

Your job is to **subscribe** to it, apply a **first-order IIR low-pass filter**, and **publish** the filtered value on `/${GITHUB_USER}/answer` (`std_msgs/Float32`).

### The filter (exact spec — do not change α)

```
y[n] = α · x[n] + (1 − α) · y[n − 1]
y[0] = x[0]
α    = 0.1
```

This is the same "exponential moving average" you'll see in most sensor pipelines. Theory refs:
- Wikipedia — [Infinite impulse response](https://en.wikipedia.org/wiki/Infinite_impulse_response) and [Exponential smoothing](https://en.wikipedia.org/wiki/Exponential_smoothing).
- Smith, *The Scientist and Engineer's Guide to DSP*, [Ch. 19 — Recursive Filters](https://www.dspguide.com/ch19.htm) (free online).

### Where to put your code
Open `ros2_ws/src/a5_new_member/a5_new_member/lpf_node.py`. There's a `TODO` block inside `_on_signal`. Replace the stub with the IIR recurrence above.

### Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a5_new_member lpf.launch.py github_user:=$GITHUB_USER
```

Watch your verdict the same way as A5.1 (see §2) — `ros2 topic echo /neil/feedback | grep --line-buffered "$GITHUB_USER"` in another terminal.

### How grading works
Neil's grader:
1. Subscribes to `/neil/signal` and runs **the same** LPF (α = 0.1, y[0] = x[0]) to build a reference sequence.
2. Discovers any `/<user>/answer` topic on the network and buffers the last 200 samples per student.
3. Matches student samples to the reference by nearest receive-time and computes MSE.
4. If MSE < 0.02, publishes on `/neil/feedback`:
   ```
   Congratulations <your-github-user> you got the correct LPF value
   ```
   Otherwise:
   ```
   Sorry <your-github-user>, the answer is incorrect (MSE=0.4127)
   ```

Feedback is republished on every grading tick (~every 2s) while your `/answer` topic is live, so it always reflects your current state — fix your filter and you'll see it flip to "correct" without needing to restart anything.

**Deliverable for A5.2:** screenshot of `/neil/feedback` congratulating your handle (MSE value visible), committed as `submissions/a5_2_feedback.png`, plus your finished `lpf_node.py`.

---

## 4. Visualizing with Foxglove Studio (optional but strongly recommended)

Seeing the raw signal and your filtered signal on the same plot makes it obvious what your filter is doing wrong.

### 4.1 Install the Foxglove bridge inside the container
Already available in the image:
```bash
apt-get install -y ros-humble-foxglove-bridge   # only if you rebuild the image
ros2 run foxglove_bridge foxglove_bridge port:=8765
```
Leave that terminal running.

### 4.2 Install Foxglove Studio on your host
Download from <https://foxglove.dev/download> (free, works on Linux/macOS/Windows).

### 4.3 Connect
1. Open Foxglove Studio → **Open connection…** → **Foxglove WebSocket**.
2. URL: `ws://localhost:8765` (or `ws://<your-tailscale-host>:8765` from another machine on the tailnet).
3. Add a **Plot** panel.
   - Series 1: topic `/neil/signal`, path `data`, color red.
   - Series 2: topic `/${GITHUB_USER}/answer`, path `data`, color green.
4. Add a **Raw Messages** panel on `/neil/feedback` to see verdicts as they arrive.

You should see the green (filtered) curve tracking the low-frequency component of the red (noisy) signal while attenuating the 5 Hz sinusoid — that's the LPF working.

> Tip: save your Foxglove layout to `submissions/a5_layout.json` (**Layout → Export**) so future assignments can reuse it.

---

## 5. Topic contract (summary)

| Topic              | Type              | Owner   | Purpose                            |
| ------------------ | ----------------- | ------- | ---------------------------------- |
| `/neil/signal`     | `std_msgs/Float32` | Neil    | Noisy input for A5.2               |
| `/neil/feedback`   | `std_msgs/String`  | Neil    | Per-student grading verdict        |
| `/<user>/hello`    | `std_msgs/String`  | Student | A5.1 output                        |
| `/<user>/answer`   | `std_msgs/Float32` | Student | A5.2 output (your filtered signal) |

---

## 6. Layout

```
Driverless-AA5/
├── docker/
│   ├── docker-compose-local.yml  # Two services: student + grader
│   ├── Dockerfile                # ROS2 Humble + dependencies
│   ├── entrypoint.sh             # Startup script (socket buffer config)
│   └── cyclonedds.xml            # DDS discovery config
├── ros2_ws/
│   └── src/
│       ├── a5_new_member/        # your template — write code here
│       └── a5_grader/            # grader (runs as local service in docker-compose)
│           ├── a5_grader/        # Python package
│           ├── config/           # params.yaml
│           ├── launch/           # neil.launch.py
│           ├── setup.py
│           └── package.xml
└── README.md
```

## 7. Troubleshooting

- **`ros2 topic list` doesn't show `/neil/signal`.** DDS discovery isn't reaching Neil. Confirm `tailscale ping <neil-host>` works, that `A2_NEIL_HOST` is set, and that `ROS_DOMAIN_ID` matches (`42`).
- **You see your own topics but no one else's.** Check `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` inside the container (`env | grep RMW`).
- **Grader keeps saying incorrect.** Confirm α = 0.1, that you initialise `y[0] = x[0]` (not zero), and that you're publishing on `/<GITHUB_USER>/answer` (not `~answer` or `/answer`).

---

## 8. Submitting via Pull Request

Committing screenshots to `submissions/` on your branch is only half the workflow. The class repo uses pull requests + review for every landed change, and this assignment is your first practice PR. Follow these steps end-to-end.

1. Push your `FirstNameLastName` branch to GitHub:
   ```bash
   git push -u origin FirstNameLastName
   ```
2. On GitHub, open a PR from `<your-branch>` → `main`.
3. **PR title:** `A2 submission — <Your Name>`.
4. **PR body** must include:
   - Your GitHub handle.
   - The screenshot of `/neil/feedback` congratulating you for A5.1 (drag-and-drop into the PR body, or reference it as `![A5.1](submissions/a5_1_feedback.png)`).
   - The screenshot for A5.2 with the MSE value visible.
   - A one-paragraph reflection: what surprised you about DDS or the filter?
5. Neil (or a designated senior) reviews the PR:
   - Screenshots must show your handle in the feedback string.
   - On approval, they close the PR **without merging**.
6. That's it — the PR is your record of having completed A5. It never lands on `main`: merging would ship your working `hello_publisher.py`/`lpf_node.py` as the template, handing the answer to every student who clones this repo afterward.

> **Why bother with a PR if it doesn't merge?** The PR is how you practice the real MFE workflow — every change to `MFE-Driverless-V1` lands via PR + review, no exceptions. This assignment mimics that process end-to-end (branch, push, PR, review); the merge step is the one part intentionally skipped, so the template stays answer-free for the next student.

### What reviewers look for

- Node runs without exceptions inside the container.
- Screenshots prove the auto-grader accepted your new_member.
- No secrets or personal paths committed.
- Reasonable commit messages.

---

## 9. Submission
1. Commit your changes to your `FirstNameLastName` branch.
2. Include both feedback screenshots in `submissions/`.
3. Open a pull request against `main` when done (see section 8 for the full workflow).

---

## 10. Parameters

The scenario constants (signal frequencies/amplitudes, noise, filter α, grader tolerances, timer periods) are exposed as ROS parameters and loaded from YAML at launch time. You should not need to edit them for the graded assignment, but tweaking them locally is a useful way to build intuition (e.g. crank up `noise_std` and watch your MSE climb).

- Grader side: [`ros2_ws/src/a5_grader/config/params.yaml`](ros2_ws/src/a5_grader/config/params.yaml) — `signal_hz`, `f1`, `a1`, `f2`, `a2`, `noise_std`, `seed` (for `signal_publisher`); `alpha`, `match_window`, `mse_tolerance`, `discovery_period_s`, `grade_period_s` (for `grader`).
- Student side: [`ros2_ws/src/a5_new_member/config/params.yaml`](ros2_ws/src/a5_new_member/config/params.yaml) — `alpha` (fixed at `0.1` for grading; do **not** change for your submission).

The launch files (`neil.launch.py`, `lpf.launch.py`) pass the YAML file into each node via the `parameters=[...]` argument, so `ros2 launch` picks them up automatically.

