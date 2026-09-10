"""A5 Neil sim node.

Responsibilities:
  * Load `config/track.yaml` and publish the cone map on `/neil/cone_map`
    as a `geometry_msgs/PoseArray` (latched via TRANSIENT_LOCAL).
  * Encode cone color in `orientation.z` (0.0 = blue, 1.0 = yellow) with
    `orientation.w = 1.0`. This is NOT a valid quaternion — it is a compact
    per-cone label and is documented in the README.
  * Discover per-student `/<user>/cmd` topics
    (`ackermann_msgs/AckermannDrive`) via `get_topic_names_and_types()` every
    2 s. For each new user, spawn a fresh bicycle-model sim state (start at
    the first straight, v = `sim.start_v`) and start integrating.
  * At `sim.hz` Hz, integrate every active sim using a kinematic bicycle
    model augmented with a **linear lateral-tire limit** (understeer) and a
    longitudinal accel limit. Numbers are pulled from ROS parameters
    (see `a5_neil/config/params.yaml`) — defaults match the MFE25
    `mfe_bringup/config/vehicle.yaml`:
        wheelbase        = 1.56 m
        max_steer_rad    = 0.489 rad (28°)
        max_lat_accel    = 8.0  m/s^2
        max_decel        = 10.0 m/s^2
        max_accel        = 5.0  m/s^2
  * Publish `/<user>/state` (`nav_msgs/Odometry`, map frame) and
    `/<user>/dynamics_debug` (`std_msgs/Float32MultiArray` with
    `[a_lat_req, a_lat_actual, understeer_active]`).
  * Track per-user lap count (crossing start line in correct direction),
    cone hits, lap time, mean lateral error to the analytic centerline.
    Expose these to the grader via `/neil/sim_stats`
    (String, JSON payload, TRANSIENT_LOCAL).
"""
from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDrive
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, String

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# Module-constant defaults so external importers (e.g. grader.py) don't break.
WHEELBASE_L = 1.56          # matches MFE-Driverless-V1 vehicle.yaml
MAX_STEERING = 0.489        # rad (28 deg) — matches vehicle.yaml
MAX_SPEED = 15.0            # m/s (sim cap, not a vehicle physical limit)
CONE_HIT_RADIUS = 0.4       # m
LAP_START_HALF_WIDTH = 2.0  # m from the start pose along the +y axis
COMMAND_TIMEOUT_S = 1.0     # if no cmd for this long, coast to zero

CMD_RE = re.compile(r'^/([^/]+)/cmd$')
RESERVED_USERS = {'neil'}


# ---------------------------------------------------------------------------
# Track / centerline helpers (shared with grader.py via import)
# ---------------------------------------------------------------------------
def load_track(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def analytic_centerline(n: int = 400) -> np.ndarray:
    """Return the true centerline as an ordered (N, 2) array.

    Must stay in sync with the track.yaml generator: same base circle and
    perturbation. Used by the grader for A5.1 and by the sim for lateral
    error computation.
    """
    u = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    r = 25.0 + 4.0 * np.sin(u) + 3.0 * np.sin(2.0 * u + 0.7)
    x = r * np.cos(u)
    y = r * np.sin(u)
    return np.stack([x, y], axis=1)


# ---------------------------------------------------------------------------
# Per-user sim state
# ---------------------------------------------------------------------------
@dataclass
class BicycleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    last_cmd_time: float = 0.0
    cmd_steer: float = 0.0
    cmd_speed: float = 0.0
    # last-step diagnostics (for /<user>/dynamics_debug)
    last_a_lat_req: float = 0.0
    last_a_lat_actual: float = 0.0
    last_understeer_active: bool = False
    # bookkeeping
    laps: int = 0
    cone_hits: int = 0
    lap_start_time: Optional[float] = None
    last_lap_time: Optional[float] = None
    lat_err_sum: float = 0.0
    lat_err_count: int = 0
    prev_signed_x: float = 0.0     # for lap-line crossing detection
    hit_cones: set = field(default_factory=set)


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')

        # -- ROS parameters (declare_parameter; defaults track vehicle.yaml).
        # Keys mirror a5_neil/config/params.yaml — grouped under vehicle.*,
        # sim.*, grading.*.
        self.wheelbase_L = float(
            self.declare_parameter('vehicle.wheelbase', WHEELBASE_L).value
        )
        self.max_steer_rad = float(
            self.declare_parameter('vehicle.max_steer_rad', MAX_STEERING).value
        )
        self.max_lat_accel = float(
            self.declare_parameter('vehicle.max_lat_accel', 8.0).value
        )
        self.max_decel = float(
            self.declare_parameter('vehicle.max_decel', 10.0).value
        )
        self.max_accel = float(
            self.declare_parameter('vehicle.max_accel', 5.0).value
        )
        self.max_speed = float(
            self.declare_parameter('vehicle.max_speed', MAX_SPEED).value
        )
        self.cone_hit_dist_m = float(
            self.declare_parameter('grading.cone_hit_dist_m', CONE_HIT_RADIUS).value
        )
        self.sim_hz = float(self.declare_parameter('sim.hz', 50.0).value)
        self.speed_ctrl_k = float(
            self.declare_parameter('sim.speed_ctrl_k', 3.0).value
        )
        self.start_v = float(self.declare_parameter('sim.start_v', 0.0).value)
        self.discovery_period_s = float(
            self.declare_parameter('sim.discovery_period_s', 2.0).value
        )
        self.stats_period_s = float(
            self.declare_parameter('sim.stats_period_s', 1.0).value
        )
        self.sim_dt = 1.0 / self.sim_hz

        share = get_package_share_directory('a5_neil')
        track_path = os.path.join(share, 'config', 'track.yaml')
        self._track = load_track(track_path)
        self._blue = np.array(self._track['blue_cones'], dtype=float)
        self._yellow = np.array(self._track['yellow_cones'], dtype=float)
        self._start = self._track['start_pose']
        self._centerline = analytic_centerline()

        # latched cone map
        self.cone_pub = self.create_publisher(
            PoseArray, '/neil/cone_map', LATCHED_QOS
        )
        self._publish_cone_map()

        # stats (JSON, latched so late-joining graders see the latest)
        self.stats_pub = self.create_publisher(
            String, '/neil/sim_stats', LATCHED_QOS
        )

        # per-user state
        self._sims: Dict[str, BicycleState] = {}
        self._state_pubs: Dict[str, object] = {}
        self._debug_pubs: Dict[str, object] = {}
        self._cmd_subs: Dict[str, object] = {}

        self.create_timer(self.discovery_period_s, self._discover)
        self.create_timer(self.sim_dt, self._step)
        self.create_timer(self.stats_period_s, self._publish_stats)

        self.get_logger().info(
            f'SimNode: {len(self._blue)} blue + {len(self._yellow)} yellow cones, '
            f'L={self.wheelbase_L} m, dt={self.sim_dt:.3f}s, '
            f'a_lat_max={self.max_lat_accel} m/s^2, '
            f'a_long in [-{self.max_decel}, +{self.max_accel}] m/s^2.'
        )

    # -- cone map ----------------------------------------------------------
    def _publish_cone_map(self) -> None:
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self._blue:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0
            p.orientation.z = 0.0  # blue
            msg.poses.append(p)
        for x, y in self._yellow:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0
            p.orientation.z = 1.0  # yellow
            msg.poses.append(p)
        self.cone_pub.publish(msg)
        self.get_logger().info(
            f'Published cone_map (latched) with {len(msg.poses)} cones.'
        )

    # -- discovery ---------------------------------------------------------
    def _discover(self) -> None:
        for name, types in self.get_topic_names_and_types():
            m = CMD_RE.match(name)
            if not m:
                continue
            user = m.group(1)
            if user in RESERVED_USERS or user in self._cmd_subs:
                continue
            if 'ackermann_msgs/msg/AckermannDrive' not in types:
                continue
            self._spawn_user(user)

    def _spawn_user(self, user: str) -> None:
        s = BicycleState(
            x=float(self._start['x']),
            y=float(self._start['y']),
            yaw=float(self._start['yaw']),
            v=self.start_v,
            last_cmd_time=self._now(),
        )
        s.prev_signed_x = 0.0
        self._sims[user] = s
        self._state_pubs[user] = self.create_publisher(
            Odometry, f'/{user}/state', RELIABLE_QOS
        )
        self._debug_pubs[user] = self.create_publisher(
            Float32MultiArray, f'/{user}/dynamics_debug', RELIABLE_QOS
        )
        self._cmd_subs[user] = self.create_subscription(
            AckermannDrive, f'/{user}/cmd', self._make_cmd_cb(user), RELIABLE_QOS
        )
        self.get_logger().info(f'Spawned sim for user {user}.')

    def _make_cmd_cb(self, user: str):
        def _cb(msg: AckermannDrive) -> None:
            s = self._sims.get(user)
            if s is None:
                return
            steer = max(-self.max_steer_rad,
                        min(self.max_steer_rad, float(msg.steering_angle)))
            speed = max(0.0, min(self.max_speed, float(msg.speed)))
            s.cmd_steer = steer
            s.cmd_speed = speed
            s.last_cmd_time = self._now()
        return _cb

    # -- integration -------------------------------------------------------
    def _step(self) -> None:
        now = self._now()
        L = self.wheelbase_L
        dt = self.sim_dt
        for user, s in self._sims.items():
            # Command timeout -> coast to zero speed.
            if now - s.last_cmd_time > COMMAND_TIMEOUT_S:
                target_speed = 0.0
                steer_cmd = 0.0
            else:
                target_speed = s.cmd_speed
                steer_cmd = s.cmd_steer

            # --- Longitudinal: clamp v_dot in [-max_decel, +max_accel] ---
            v_dot_raw = self.speed_ctrl_k * (target_speed - s.v)
            v_dot = max(-self.max_decel, min(self.max_accel, v_dot_raw))
            s.v += v_dot * dt

            # --- Lateral: linear tire-limit understeer ---
            # a_lat_req = v^2 * tan(delta) / L. If the driver commands a
            # steer that would need more grip than max_lat_accel, the tires
            # saturate and the effective steering angle is clamped so
            # a_lat_actual = max_lat_accel. Sign of the driver's command
            # is preserved.
            if abs(s.v) > 1e-3:
                a_lat_req = s.v * s.v * math.tan(steer_cmd) / L
            else:
                a_lat_req = 0.0

            if abs(a_lat_req) > self.max_lat_accel and abs(s.v) > 1e-3:
                sign = math.copysign(1.0, steer_cmd) if steer_cmd != 0.0 \
                    else math.copysign(1.0, a_lat_req)
                # delta_eff = atan(a_lat_max * L / v^2), sign of driver input
                steer_eff = sign * math.atan(self.max_lat_accel * L / (s.v * s.v))
                # Never exceed the physical steering limit either.
                steer_eff = max(-self.max_steer_rad,
                                min(self.max_steer_rad, steer_eff))
                understeer_active = True
            else:
                steer_eff = steer_cmd
                understeer_active = False

            # a_lat_actual after any clamping (may be < a_lat_max if driver
            # wasn't at the limit).
            a_lat_actual = s.v * s.v * math.tan(steer_eff) / L

            s.last_a_lat_req = float(a_lat_req)
            s.last_a_lat_actual = float(a_lat_actual)
            s.last_understeer_active = bool(understeer_active)

            # --- Bicycle kinematics with the effective steering angle ---
            s.x += s.v * math.cos(s.yaw) * dt
            s.y += s.v * math.sin(s.yaw) * dt
            s.yaw += s.v / L * math.tan(steer_eff) * dt
            s.yaw = math.atan2(math.sin(s.yaw), math.cos(s.yaw))

            self._publish_state(user, s)
            self._publish_dynamics_debug(user, s)
            self._update_metrics(user, s, now)

    def _publish_state(self, user: str, s: BicycleState) -> None:
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = f'{user}/base_link'
        odom.pose.pose.position.x = s.x
        odom.pose.pose.position.y = s.y
        odom.pose.pose.orientation.z = math.sin(s.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(s.yaw / 2.0)
        odom.twist.twist.linear.x = s.v
        self._state_pubs[user].publish(odom)

    def _publish_dynamics_debug(self, user: str, s: BicycleState) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(s.last_a_lat_req),
            float(s.last_a_lat_actual),
            1.0 if s.last_understeer_active else 0.0,
        ]
        self._debug_pubs[user].publish(msg)

    # -- metrics -----------------------------------------------------------
    def _update_metrics(self, user: str, s: BicycleState, now: float) -> None:
        # cone hits (each cone can only count once per lap)
        pos = np.array([s.x, s.y])
        all_cones = np.vstack([self._blue, self._yellow])
        d = np.linalg.norm(all_cones - pos, axis=1)
        for idx in np.where(d < self.cone_hit_dist_m)[0]:
            if idx not in s.hit_cones:
                s.hit_cones.add(int(idx))
                s.cone_hits += 1

        # lateral error to analytic centerline
        cd = np.linalg.norm(self._centerline - pos, axis=1)
        s.lat_err_sum += float(cd.min())
        s.lat_err_count += 1

        # lap detection: cross the start line moving in +yaw direction.
        # Transform position into start-pose local frame; a lap is a sign
        # change in local x (forward axis) while |local y| < LAP_START_HALF_WIDTH.
        sx = float(self._start['x'])
        sy = float(self._start['y'])
        syaw = float(self._start['yaw'])
        dx = s.x - sx
        dy = s.y - sy
        local_x = math.cos(-syaw) * dx - math.sin(-syaw) * dy
        local_y = math.sin(-syaw) * dx + math.cos(-syaw) * dy
        if s.lap_start_time is None:
            s.lap_start_time = now
        crossed = (s.prev_signed_x < 0.0 <= local_x) and abs(local_y) < LAP_START_HALF_WIDTH
        # Require some distance travelled so we don't false-trigger on t=0.
        if crossed and (now - s.lap_start_time) > 5.0:
            s.laps += 1
            s.last_lap_time = now - s.lap_start_time
            s.lap_start_time = now
            s.hit_cones.clear()  # reset per-lap cone tracking
            self.get_logger().info(
                f'[{user}] lap {s.laps} completed in {s.last_lap_time:.2f}s '
                f'(cone_hits so far: {s.cone_hits})'
            )
        s.prev_signed_x = local_x

    def _publish_stats(self) -> None:
        payload = {}
        for user, s in self._sims.items():
            mean_lat = (s.lat_err_sum / s.lat_err_count) if s.lat_err_count else 0.0
            payload[user] = {
                'laps': s.laps,
                'cone_hits': s.cone_hits,
                'last_lap_time': s.last_lap_time,
                'mean_lat_err': mean_lat,
                'elapsed': self._now() - (s.lap_start_time or self._now()),
            }
        msg = String()
        msg.data = json.dumps(payload)
        self.stats_pub.publish(msg)

    # -- utils -------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
