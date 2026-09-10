"""A5.1 — centerline path planner.

Subscribe to Neil's cone map on `/neil/cone_map`
(`geometry_msgs/PoseArray`, latched via TRANSIENT_LOCAL) and your own state
on `/<namespace>/state` (`nav_msgs/Odometry`). Publish an ordered centerline
on `/<namespace>/centerline` (`nav_msgs/Path`, `map` frame).

Cone-color encoding (see README):
    pose.orientation.z == 0.0  ->  blue  (left boundary)
    pose.orientation.z == 1.0  ->  yellow (right boundary)
    pose.orientation.w == 1.0  (constant filler; not a real quaternion)

Run with your GitHub username as the ROS namespace:

    ros2 run a5_solution planner_node --ros-args -r __ns:=/<github-username>
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Cone map is latched by Neil via TRANSIENT_LOCAL — our subscription
# must match durability or DDS will silently drop us.
LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self._blue: Optional[np.ndarray] = None
        self._yellow: Optional[np.ndarray] = None
        self._have_state = False

        self.create_subscription(
            PoseArray, '/neil/cone_map', self._on_cones, LATCHED_QOS
        )
        self.create_subscription(
            Odometry, 'state', self._on_state, RELIABLE_QOS
        )
        self.path_pub = self.create_publisher(
            Path, 'centerline', RELIABLE_QOS
        )

        # Recompute the plan every second (cones don't move, but this makes
        # late-joining topics easy to reason about).
        self.create_timer(1.0, self._plan_and_publish)

        ns = self.get_namespace()
        self.get_logger().info(
            f'PlannerNode: publishing centerline on {ns}/centerline'
        )

    # -- callbacks ---------------------------------------------------------
    def _on_cones(self, msg: PoseArray) -> None:
        blue: List[Tuple[float, float]] = []
        yellow: List[Tuple[float, float]] = []
        for p in msg.poses:
            xy = (p.position.x, p.position.y)
            if p.orientation.z < 0.5:
                blue.append(xy)
            else:
                yellow.append(xy)
        self._blue = np.array(blue, dtype=float) if blue else None
        self._yellow = np.array(yellow, dtype=float) if yellow else None
        self.get_logger().info(
            f'Cone map: {len(blue)} blue, {len(yellow)} yellow.'
        )

    def _on_state(self, msg: Odometry) -> None:
        self._have_state = True

    # -- planning ----------------------------------------------------------
    def _plan_and_publish(self) -> None:
        if self._blue is None or self._yellow is None:
            return
        if len(self._blue) < 3 or len(self._yellow) < 3:
            return

        centerline = self._compute_centerline(self._blue, self._yellow)
        if centerline is None or len(centerline) < 3:
            return

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in centerline:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _compute_centerline(
        self, blue: np.ndarray, yellow: np.ndarray
    ) -> Optional[np.ndarray]:
        """Compute an ordered centerline from left/right cone boundaries.

        Reference approach (recommended): pair each blue cone with its
        nearest yellow cone, take the midpoints, then order the midpoints
        by walking around the loop from an arbitrary starting point.

        A more principled option is a Delaunay triangulation over the
        combined cone set and keeping only the edges that connect a blue
        to a yellow cone (see `scipy.spatial.Delaunay`). Either is fine
        for grading — the tolerance is 1.0 m.
        """
        # ------------------------------------------------------------------
        # TODO(student): implement centerline construction.
        #
        # A minimal recipe:
        #   1. For each blue cone, find its nearest yellow cone.
        #   2. Compute the midpoint of that pair -> raw centerline point.
        #   3. Order the midpoints so they form a loop: start from any
        #      point, repeatedly pick the nearest unvisited neighbour.
        #   4. Return an (N, 2) array. Aim for N >= 30 (grader threshold).
        #
        # HINT: numpy broadcasting makes step 1 a one-liner:
        #   d = np.linalg.norm(blue[:, None, :] - yellow[None, :, :], axis=2)
        #   nearest = np.argmin(d, axis=1)
        # ------------------------------------------------------------------
        d = np.linalg.norm(blue[:, None, :] - yellow[None, :, :], axis=2)
        nearest = np.argmin(d, axis=1)
        mids = 0.5 * (blue + yellow[nearest])
        # Stub ordering (leaves it up to the student to improve on):
        return self._greedy_loop_order(mids)

    @staticmethod
    def _greedy_loop_order(points: np.ndarray) -> np.ndarray:
        """Greedy nearest-neighbour ordering starting at index 0."""
        n = len(points)
        if n == 0:
            return points
        used = np.zeros(n, dtype=bool)
        order = [0]
        used[0] = True
        for _ in range(n - 1):
            last = points[order[-1]]
            d = np.linalg.norm(points - last, axis=1)
            d[used] = np.inf
            nxt = int(np.argmin(d))
            order.append(nxt)
            used[nxt] = True
        return points[order]


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
