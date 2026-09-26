"""Publishes /neil/signal at signal_hz Hz.

Signal = sum of two sinusoids + zero-mean Gaussian noise, deterministic given
the seed (so grading is reproducible across restarts on Neil's side).

    x(t) = a1*sin(2*pi*f1*t) + a2*sin(2*pi*f2*t) + noise(t)

params: signal_hz, f1, a1, f2, a2, noise_std, seed (see config/params.yaml).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class SignalPublisher(Node):
    def __init__(self):
        super().__init__('signal_publisher')

        # Scenario parameters (see a2_neil/config/params.yaml).
        self.declare_parameter('signal_hz', 20.0)
        self.declare_parameter('f1', 0.5)
        self.declare_parameter('a1', 1.0)
        self.declare_parameter('f2', 5.0)
        self.declare_parameter('a2', 0.6)
        self.declare_parameter('noise_std', 0.3)
        self.declare_parameter('seed', 20260909)

        self.signal_hz = float(self.get_parameter('signal_hz').value)
        self.f1 = float(self.get_parameter('f1').value)
        self.a1 = float(self.get_parameter('a1').value)
        self.f2 = float(self.get_parameter('f2').value)
        self.a2 = float(self.get_parameter('a2').value)
        self.noise_std = float(self.get_parameter('noise_std').value)
        self.seed = int(self.get_parameter('seed').value)

        self.pub = self.create_publisher(Float32, '/neil/signal', RELIABLE_QOS)
        self.timer = self.create_timer(1.0 / self.signal_hz, self._tick)
        self._rng = np.random.default_rng(self.seed)
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'Publishing /neil/signal @ {self.signal_hz:.1f} Hz '
            f'(f1={self.f1} Hz, f2={self.f2} Hz, noise_std={self.noise_std})'
        )

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        clean = (
            self.a1 * math.sin(2 * math.pi * self.f1 * t)
            + self.a2 * math.sin(2 * math.pi * self.f2 * t)
        )
        x = clean + float(self._rng.normal(0.0, self.noise_std))
        msg = Float32()
        msg.data = float(x)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
