"""Auto-discovery grader for MFE A5.

Grades:
  - A5.1: /<user>/centerline (nav_msgs/Path) — path planning
  - A5.2: /<user>/cmd (ackermann_msgs/AckermannDrive) — pure-pursuit controller

Feedback published on /grader/feedback (std_msgs/String).

params: discovery_period_s, grade_period_s
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

CENTERLINE_RE = re.compile(r'^/([^/]+)/centerline$')
CMD_RE = re.compile(r'^/([^/]+)/cmd$')
RESERVED_USERS = {'grader'}


@dataclass
class PathState:
    last_msg: Optional[Path] = None
    message_count: int = 0


@dataclass
class CmdState:
    last_msg: Optional[AckermannDrive] = None
    message_count: int = 0


class Grader(Node):
    def __init__(self):
        super().__init__('grader')

        self.declare_parameter('discovery_period_s', 2.0)
        self.declare_parameter('grade_period_s', 2.0)

        self.discovery_period_s = float(self.get_parameter('discovery_period_s').value)
        self.grade_period_s = float(self.get_parameter('grade_period_s').value)

        self.feedback_pub = self.create_publisher(String, '/grader/feedback', RELIABLE_QOS)

        self._centerline: dict[str, PathState] = {}
        self._centerline_subs: dict[str, object] = {}
        self._cmd: dict[str, CmdState] = {}
        self._cmd_subs: dict[str, object] = {}

        self.create_timer(self.discovery_period_s, self._discover)
        self.create_timer(self.grade_period_s, self._grade)

        self.get_logger().info('A5 Grader running (A5.1 Planner + A5.2 Controller)')

    def _discover(self) -> None:
        for name, types in self.get_topic_names_and_types():
            m = CENTERLINE_RE.match(name)
            if m and 'nav_msgs/msg/Path' in types:
                user = m.group(1)
                if user in RESERVED_USERS or user in self._centerline_subs:
                    continue
                self._centerline[user] = PathState()
                self._centerline_subs[user] = self.create_subscription(
                    Path, name, self._make_centerline_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A5.1 (Planner) topic: {name}')
                continue

            m = CMD_RE.match(name)
            if m and 'ackermann_msgs/msg/AckermannDrive' in types:
                user = m.group(1)
                if user in RESERVED_USERS or user in self._cmd_subs:
                    continue
                self._cmd[user] = CmdState()
                self._cmd_subs[user] = self.create_subscription(
                    AckermannDrive, name, self._make_cmd_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A5.2 (Controller) topic: {name}')

    def _make_centerline_cb(self, user: str):
        def _cb(msg: Path) -> None:
            state = self._centerline[user]
            state.last_msg = msg
            state.message_count += 1
        return _cb

    def _make_cmd_cb(self, user: str):
        def _cb(msg: AckermannDrive) -> None:
            state = self._cmd[user]
            state.last_msg = msg
            state.message_count += 1
        return _cb

    def _grade(self) -> None:
        for user, state in self._centerline.items():
            if state.message_count == 0:
                continue
            num_points = len(state.last_msg.poses)
            self._publish_feedback(
                user,
                'A5.1',
                f'Centerline: {num_points} points',
            )

        for user, state in self._cmd.items():
            if state.message_count == 0:
                continue
            steering = state.last_msg.steering_angle
            speed = state.last_msg.speed
            self._publish_feedback(
                user,
                'A5.2',
                f'Control: steer={steering:.3f} speed={speed:.2f}',
            )

    def _publish_feedback(self, user: str, task: str, detail: str) -> None:
        text = f'{user} {task}: {detail}'
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
