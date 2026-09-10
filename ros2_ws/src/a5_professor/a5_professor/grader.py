"""Auto-discovery grader for MFE A5.

Scans the ROS graph every 2 s and grades two things:
  * A5.1 — the student's `/<user>/centerline` (nav_msgs/Path). Pass if the
    published path stays within 1.0 m of the analytic true centerline for
    its full length and has >= 30 points.
  * A5.2 — the per-user sim results reported by `a5_professor/sim_node`
    on `/professor/sim_stats` (JSON String, latched). Pass if the user
    completes >= 1 lap within 60 s with zero cone hits.

Feedback is published on `/professor/feedback` (std_msgs/String), only when
a student's verdict changes, in the same format as the earlier assignments:
    'Congrats <user>, the answer is correct'
    'Sorry <user>, the answer is incorrect (<metric>)'
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

from a5_professor.sim_node import analytic_centerline

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

DISCOVERY_PERIOD_S = 2.0
GRADE_PERIOD_S = 2.0

CENTERLINE_RE = re.compile(r'^/([^/]+)/centerline$')
RESERVED_USERS = {'professor'}

# A5.1 thresholds
A51_MAX_LATERAL = 1.0   # m
A51_MIN_POINTS = 30

# A5.2 thresholds
A52_MIN_LAPS = 1
A52_DEADLINE_S = 60.0
A52_MAX_CONE_HITS = 0


@dataclass
class UserState:
    a51_last_verdict: Optional[str] = None
    a52_last_verdict: Optional[str] = None
    first_seen: float = field(default_factory=time.time)


class Grader(Node):
    def __init__(self):
        super().__init__('grader')

        self._centerline_ref = analytic_centerline(n=800)

        self.feedback_pub = self.create_publisher(
            String, '/professor/feedback', RELIABLE_QOS
        )

        self._path_subs: Dict[str, object] = {}
        self._latest_paths: Dict[str, Path] = {}
        self._users: Dict[str, UserState] = {}
        self._latest_stats: dict = {}

        self.create_subscription(
            String, '/professor/sim_stats', self._on_stats, LATCHED_QOS
        )

        self.create_timer(DISCOVERY_PERIOD_S, self._discover)
        self.create_timer(GRADE_PERIOD_S, self._grade)

        self.get_logger().info(
            f'Grader running. A5.1 tol={A51_MAX_LATERAL} m / >={A51_MIN_POINTS} pts, '
            f'A5.2 >= {A52_MIN_LAPS} lap in {A52_DEADLINE_S:.0f}s with '
            f'<= {A52_MAX_CONE_HITS} cone hits.'
        )

    # -- discovery ---------------------------------------------------------
    def _discover(self) -> None:
        for name, types in self.get_topic_names_and_types():
            m = CENTERLINE_RE.match(name)
            if not m:
                continue
            user = m.group(1)
            if user in RESERVED_USERS or user.startswith('professor'):
                continue
            if user in self._path_subs:
                continue
            if 'nav_msgs/msg/Path' not in types:
                continue
            self._users.setdefault(user, UserState())
            self._path_subs[user] = self.create_subscription(
                Path, name, self._make_path_cb(user), RELIABLE_QOS
            )
            self.get_logger().info(f'Discovered A5.1 topic: {name}')

    def _make_path_cb(self, user: str):
        def _cb(msg: Path) -> None:
            self._latest_paths[user] = msg
        return _cb

    # -- sim stats ---------------------------------------------------------
    def _on_stats(self, msg: String) -> None:
        try:
            self._latest_stats = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Bad JSON on /professor/sim_stats')
        for user in self._latest_stats:
            if user in RESERVED_USERS or user.startswith('professor'):
                continue
            self._users.setdefault(user, UserState())

    # -- grading -----------------------------------------------------------
    def _grade(self) -> None:
        for user, state in self._users.items():
            self._grade_a51(user, state)
            self._grade_a52(user, state)

    def _grade_a51(self, user: str, state: UserState) -> None:
        path = self._latest_paths.get(user)
        if path is None or len(path.poses) == 0:
            return
        pts = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in path.poses]
        )
        if len(pts) < A51_MIN_POINTS:
            verdict = 'incorrect'
            metric = f'only {len(pts)} points (need >= {A51_MIN_POINTS})'
        else:
            # For each student point, distance to nearest true-centerline point.
            # (Centerline is dense enough that nearest-point ~= perpendicular.)
            diffs = pts[:, None, :] - self._centerline_ref[None, :, :]
            dists = np.linalg.norm(diffs, axis=2)
            max_err = float(dists.min(axis=1).max())
            if max_err <= A51_MAX_LATERAL:
                verdict = 'correct'
                metric = f'max_err={max_err:.3f} m, n={len(pts)}'
            else:
                verdict = 'incorrect'
                metric = f'max_err={max_err:.3f} m > {A51_MAX_LATERAL} m'
        if verdict != state.a51_last_verdict:
            state.a51_last_verdict = verdict
            self._publish_feedback('A5.1', user, verdict, metric)

    def _grade_a52(self, user: str, state: UserState) -> None:
        stats = self._latest_stats.get(user)
        if stats is None:
            return
        laps = int(stats.get('laps', 0))
        cone_hits = int(stats.get('cone_hits', 0))
        last_lap_time = stats.get('last_lap_time')
        if laps >= A52_MIN_LAPS and cone_hits <= A52_MAX_CONE_HITS \
                and last_lap_time is not None and last_lap_time <= A52_DEADLINE_S:
            verdict = 'correct'
            metric = f'lap_time={last_lap_time:.2f}s, cone_hits={cone_hits}'
        else:
            # Only emit an incorrect verdict once the student has had some
            # airtime; otherwise everyone gets spammed with 'incorrect' right
            # after publishing their first cmd.
            if time.time() - state.first_seen < 15.0:
                return
            verdict = 'incorrect'
            metric = f'laps={laps}, cone_hits={cone_hits}, last_lap_time={last_lap_time}'
        if verdict != state.a52_last_verdict:
            state.a52_last_verdict = verdict
            self._publish_feedback('A5.2', user, verdict, metric)

    # -- feedback ----------------------------------------------------------
    def _publish_feedback(self, task: str, user: str, verdict: str, metric: str) -> None:
        if verdict == 'correct':
            text = f'Congrats {user}, the answer is correct'
        else:
            text = f'Sorry {user}, the answer is incorrect ({metric})'
        text = f'[{task}] {text}'
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = Grader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
