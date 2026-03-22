#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class QBotFollower(Node):
    def __init__(self):
        super().__init__('qbot_follower')

        # Required behavior
        self.target_dist = 1.0
        self.detect_max = 2.0

        # Tuned so forward following is stronger, backward still smooth
        self.k_ang = 1.8
        self.max_ang = 0.8

        # Front cone for tracking
        self.front_half_angle = math.radians(25.0)

        # Need enough lidar hits to count as a real object
        self.min_valid_points = 6

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.get_logger().info('QBot follower started')

    def stop(self):
        self.pub.publish(Twist())

    def scan_cb(self, msg: LaserScan):
        points = []

        for i, r in enumerate(msg.ranges):
            if r is None or math.isnan(r) or math.isinf(r):
                continue
            if r < 0.12 or r > self.detect_max:
                continue

            ang = msg.angle_min + i * msg.angle_increment

            while ang > math.pi:
                ang -= 2.0 * math.pi
            while ang < -math.pi:
                ang += 2.0 * math.pi

            if abs(ang) <= self.front_half_angle:
                points.append((r, ang))

        # No object detected in front within 2m
        if len(points) < self.min_valid_points:
            self.stop()
            return

        # Use closest cluster of points
        points.sort(key=lambda x: x[0])
        cluster = points[:min(20, len(points))]

        avg_dist = sum(r for r, _ in cluster) / len(cluster)
        avg_ang = sum(a for _, a in cluster) / len(cluster)

        err_dist = avg_dist - self.target_dist

        # Stronger forward follow, smoother backward
        if err_dist > 0.05:
            lin = clamp(1.8 * err_dist, 0.10, 0.28)
        elif err_dist < -0.05:
            lin = clamp(1.0 * err_dist, -0.18, -0.08)
        else:
            lin = 0.0

        # Turn toward object
        if abs(avg_ang) < 0.05:
            ang_z = 0.0
        else:
            ang_z = clamp(self.k_ang * avg_ang, -self.max_ang, self.max_ang)

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang_z
        self.pub.publish(cmd)

        self.get_logger().info(
            f"pts={len(points)}, dist={avg_dist:.2f}, ang={avg_ang:.2f}, lin={lin:.2f}, angz={ang_z:.2f}"
        )


def main():
    rclpy.init()
    node = QBotFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
