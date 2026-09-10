"""A5 professor sim node.

Responsibilities:
  * Load `config/track.yaml` and publish the cone map on `/professor/cone_map`
    as a `geometry_msgs/PoseArray` (latched via TRANSIENT_LOCAL).
  * Encode cone color in `orientation.z` (0.0 = blue, 1.0 = yellow) with
    `orientation.w = 1.0`. This is NOT a valid quaternion — it is a compact
    per-cone label and is documented in the README.
  * Discover per-student `/<user>/cmd` topics
    (`ackermann_msgs/AckermannDrive`) via `get_topic_names_and_types()` every
    2 s. For each new user, spawn a fresh bicycle-model sim state (start at
    the first straight, v=0) and start integrating.
  * At 50 Hz, integrate every active sim and publish `/<user>/state`
    (`nav_msgs/Odometry`, map frame).
  * Clamp commands: |steering| <= 0.5 rad, 0 <= speed <= 15 m/s.
  * Track per-user lap count (crossing start line in correct direction),
    cone hits (dist < 0.4 m), lap time, mean lateral error to the analytic
    centerline. Expose these to the grader via `/professor/sim_stats`
    (String, JSON payload, TRANSIENT_LOCAL) for simple in-network access.
"""
from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

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
from std_msgs.msg import String

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

SIM_HZ = 50.0
SIM_DT = 1.0 / SIM_HZ
DISCOVERY_PERIOD_S = 2.0
STATS_PERIOD_S = 1.0

WHEELBASE_L = 1.56          # matches MFE-Driverless-V1 vehicle.yaml
MAX_STEERING = 0.5          # rad
MAX_SPEED = 15.0            # m/s
CONE_HIT_RADIUS = 0.4       # m
LAP_START_HALF_WIDTH = 2.0  # m from the start pose along the +y axis of the pose frame
COMMAND_TIMEOUT_S = 1.0     # if no cmd for this long, coast to zero

CMD_RE = re.compile(r'^/([^/]+)/cmd$')
RESERVED_USERS = {'professor'}


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

        share = get_package_share_directory('a5_professor')
        track_path = os.path.join(share, 'config', 'track.yaml')
        self._track = load_track(track_path)
        self._blue = np.array(self._track['blue_cones'], dtype=float)
        self._yellow = np.array(self._track['yellow_cones'], dtype=float)
        self._start = self._track['start_pose']
        self._centerline = analytic_centerline()

        # latched cone map
        self.cone_pub = self.create_publisher(
            PoseArray, '/professor/cone_map', LATCHED_QOS
        )
        self._publish_cone_map()

        # stats (JSON, latched so late-joining graders see the latest)
        self.stats_pub = self.create_publisher(
            String, '/professor/sim_stats', LATCHED_QOS
        )

        # per-user state
        self._sims: Dict[str, BicycleState] = {}
        self._state_pubs: Dict[str, object] = {}
        self._cmd_subs: Dict[str, object] = {}

        self.create_timer(DISCOVERY_PERIOD_S, self._discover)
        self.create_timer(SIM_DT, self._step)
        self.create_timer(STATS_PERIOD_S, self._publish_stats)

        self.get_logger().info(
            f'SimNode: {len(self._blue)} blue + {len(self._yellow)} yellow cones, '
            f'L={WHEELBASE_L} m, dt={SIM_DT:.3f}s'
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
            v=0.0,
            last_cmd_time=self._now(),
        )
        s.prev_signed_x = 0.0
        self._sims[user] = s
        self._state_pubs[user] = self.create_publisher(
            Odometry, f'/{user}/state', RELIABLE_QOS
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
            steer = max(-MAX_STEERING, min(MAX_STEERING, float(msg.steering_angle)))
            speed = max(0.0, min(MAX_SPEED, float(msg.speed)))
            s.cmd_steer = steer
            s.cmd_speed = speed
            s.last_cmd_time = self._now()
        return _cb

    # -- integration -------------------------------------------------------
    def _step(self) -> None:
        now = self._now()
        for user, s in self._sims.items():
            # Command timeout -> coast to zero speed.
            if now - s.last_cmd_time > COMMAND_TIMEOUT_S:
                target_speed = 0.0
                steer = 0.0
            else:
                target_speed = s.cmd_speed
                steer = s.cmd_steer

            # Simple first-order approach to target speed (rate-limited).
            s.v += 3.0 * (target_speed - s.v) * SIM_DT

            # Bicycle kinematics.
            s.x += s.v * math.cos(s.yaw) * SIM_DT
            s.y += s.v * math.sin(s.yaw) * SIM_DT
            s.yaw += s.v / WHEELBASE_L * math.tan(steer) * SIM_DT
            s.yaw = math.atan2(math.sin(s.yaw), math.cos(s.yaw))

            self._publish_state(user, s)
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

    # -- metrics -----------------------------------------------------------
    def _update_metrics(self, user: str, s: BicycleState, now: float) -> None:
        # cone hits (each cone can only count once per lap)
        pos = np.array([s.x, s.y])
        all_cones = np.vstack([self._blue, self._yellow])
        d = np.linalg.norm(all_cones - pos, axis=1)
        for idx in np.where(d < CONE_HIT_RADIUS)[0]:
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
