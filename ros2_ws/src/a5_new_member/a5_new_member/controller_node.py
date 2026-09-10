"""A5.2 — pure-pursuit lateral controller on a bicycle model.

Subscribe to your own centerline (`nav_msgs/Path` on `<ns>/centerline`) and
state (`nav_msgs/Odometry` on `<ns>/state`) and publish drive commands on
`<ns>/cmd` (`ackermann_msgs/AckermannDrive`) at 50 Hz.

Pure pursuit references:
  * Wikipedia — https://en.wikipedia.org/wiki/Pure_pursuit
  * Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path
    Tracking Algorithm* (CMU-RI-TR-92-01).

Tunables are exposed as ROS parameters (see
`a5_new_member/config/params.yaml`): `lookahead_m`, `v_target_min`,
`v_target_max`.

Run with your GitHub username as the ROS namespace:

    ros2 run a5_new_member controller_node --ros-args -r __ns:=/<github-username>
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Constants that match Neil's sim geometry (bicycle model wheelbase and
# physical steering clamp). Not parameterized — they must match the sim.
CONTROL_HZ = 50.0
WHEELBASE_L = 1.56        # m, matches Neil's sim
MAX_STEERING = 0.489      # rad, hard-clamped by the sim too (28 deg)
SPEED_CURVATURE_GAIN = 6.0   # v_target = clamp(v_max - k*|curvature|, v_min, v_max)


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # -- ROS parameters (see a5_new_member/config/params.yaml). --
        self.lookahead_m = float(
            self.declare_parameter('lookahead_m', 4.0).value
        )
        self.v_target_min = float(
            self.declare_parameter('v_target_min', 3.0).value
        )
        self.v_target_max = float(
            self.declare_parameter('v_target_max', 10.0).value
        )

        self._path: Optional[np.ndarray] = None       # (N, 2)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._have_state = False

        self.create_subscription(
            Path, 'centerline', self._on_path, RELIABLE_QOS
        )
        self.create_subscription(
            Odometry, 'state', self._on_state, RELIABLE_QOS
        )
        self.cmd_pub = self.create_publisher(
            AckermannDrive, 'cmd', RELIABLE_QOS
        )

        self.create_timer(1.0 / CONTROL_HZ, self._tick)

        ns = self.get_namespace()
        self.get_logger().info(
            f'ControllerNode: lookahead={self.lookahead_m} m, publishing {ns}/cmd'
        )

    # -- callbacks ---------------------------------------------------------
    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        self._path = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses],
            dtype=float,
        )

    def _on_state(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._yaw = 2.0 * math.atan2(qz, qw)
        self._v = msg.twist.twist.linear.x
        self._have_state = True

    # -- control loop ------------------------------------------------------
    def _tick(self) -> None:
        if not self._have_state or self._path is None or len(self._path) < 2:
            return

        steer, v_target = self._pure_pursuit(self._path)
        cmd = AckermannDrive()
        cmd.steering_angle = float(max(-MAX_STEERING, min(MAX_STEERING, steer)))
        cmd.speed = float(
            max(self.v_target_min, min(self.v_target_max, v_target))
        )
        self.cmd_pub.publish(cmd)

    def _pure_pursuit(self, path: np.ndarray) -> tuple:
        """Classic pure pursuit + a curvature-scheduled target speed.

        Returns (steering_angle_rad, v_target_mps).
        """
        # ------------------------------------------------------------------
        # TODO(student): implement pure pursuit.
        #
        #   1. Find the closest waypoint to the vehicle.
        #   2. Walk forward along the path (with wrap-around) until you
        #      find the first waypoint whose distance from the vehicle is
        #      >= lookahead_m. That is your goal point (gx, gy).
        #   3. Transform the goal into the vehicle body frame:
        #         dx = gx - x
        #         dy = gy - y
        #         local_x =  cos(-yaw)*dx - sin(-yaw)*dy
        #         local_y =  sin(-yaw)*dx + cos(-yaw)*dy
        #      (equivalently local = R(-yaw) @ [dx, dy]).
        #   4. Steering angle (bicycle model):
        #         delta = atan2(2*L*local_y, lookahead_m**2)
        #   5. Pick a target speed. A simple heuristic: use the local
        #      curvature |2*local_y / lookahead_m**2| and slow down when
        #      it's large. E.g.:
        #         curvature = 2*abs(local_y) / (lookahead_m**2)
        #         v_target  = v_target_max - SPEED_CURVATURE_GAIN * curvature
        #
        # A worked reference implementation is intentionally not provided —
        # the pieces above are all you need.
        # ------------------------------------------------------------------
        # Stub: aim straight, at the min speed. Replace this.
        gx, gy = self._pick_lookahead_stub(path)
        dx = gx - self._x
        dy = gy - self._y
        c, s = math.cos(-self._yaw), math.sin(-self._yaw)
        local_x = c * dx - s * dy
        local_y = s * dx + c * dy
        # Placeholder — students should replace with real pure-pursuit math.
        delta = math.atan2(
            2.0 * WHEELBASE_L * local_y, self.lookahead_m * self.lookahead_m
        )
        curvature = 2.0 * abs(local_y) / (self.lookahead_m * self.lookahead_m + 1e-6)
        v_target = self.v_target_max - SPEED_CURVATURE_GAIN * curvature
        return delta, v_target

    def _pick_lookahead_stub(self, path: np.ndarray) -> tuple:
        pos = np.array([self._x, self._y])
        d = np.linalg.norm(path - pos, axis=1)
        i0 = int(np.argmin(d))
        n = len(path)
        for k in range(n):
            i = (i0 + k) % n
            if np.linalg.norm(path[i] - pos) >= self.lookahead_m:
                return float(path[i, 0]), float(path[i, 1])
        return float(path[i0, 0]), float(path[i0, 1])


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
