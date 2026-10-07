"""Publish a latched local cone map and continuously updated vehicle state."""
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

RELIABLE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class ScenarioPublisher(Node):
    def __init__(self):
        super().__init__('scenario_publisher')
        self.declare_parameter('cone_pairs', 32)
        self.declare_parameter('track_radius', 12.0)
        self.declare_parameter('track_width', 4.0)
        self.declare_parameter('state_hz', 20.0)
        self.count = int(self.get_parameter('cone_pairs').value)
        self.radius = float(self.get_parameter('track_radius').value)
        self.width = float(self.get_parameter('track_width').value)
        hz = float(self.get_parameter('state_hz').value)
        self.cone_pub = self.create_publisher(PoseArray, '/grader/cone_map', LATCHED)
        self.state_pub = self.create_publisher(Odometry, '/student/state', RELIABLE)
        self._publish_map()
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / hz, self._publish_state)
        self.get_logger().info('Publishing local A5 cone map and /student/state')

    def _publish_map(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.count):
            angle = 2.0 * math.pi * i / self.count
            for radius, color in ((self.radius - self.width / 2.0, 0.0),
                                  (self.radius + self.width / 2.0, 1.0)):
                pose = Pose()
                pose.position.x = radius * math.cos(angle)
                pose.position.y = radius * math.sin(angle)
                pose.orientation.z = color
                pose.orientation.w = 1.0
                msg.poses.append(pose)
        self.cone_pub.publish(msg)

    def _publish_state(self):
        # Hold a valid initial vehicle state at the front of the circular track.
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.radius
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.z = math.sin(math.pi / 4.0)
        msg.pose.pose.orientation.w = math.cos(math.pi / 4.0)
        msg.twist.twist.linear.x = 3.0
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = ScenarioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
