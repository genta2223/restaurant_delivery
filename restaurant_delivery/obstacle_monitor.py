#!/usr/bin/env python3
"""Obstacle Monitor Node for Restaurant Delivery Robot.

Monitors LiDAR LaserScan data to detect sudden intrusions (e.g., running children,
unexpected dynamic obstacles) in front of the robot.
Publishes emergency alerts and obstacle status to stop/warn the delivery controller.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_monitor')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stop_distance', 0.45)      # Immediate emergency stop threshold (meters)
        self.declare_parameter('warning_distance', 1.00)   # Early warning threshold for diagonal motion (meters)
        self.declare_parameter('front_angle_deg', 50.0)    # Wide frontal cone (+- 50 deg, ~100 deg field of view)
        self.declare_parameter('clear_consecutive_count', 4)  # Required safe scans before clearing alert
        self.declare_parameter('startup_grace_sec', 1.5)   # Ignore startup transients for first 2 seconds

        self.scan_topic = self.get_parameter('scan_topic').value
        self.stop_dist = self.get_parameter('stop_distance').value
        self.warn_dist = self.get_parameter('warning_distance').value
        self.front_angle = math.radians(self.get_parameter('front_angle_deg').value)
        self.required_clear_count = self.get_parameter('clear_consecutive_count').value
        self.startup_grace = self.get_parameter('startup_grace_sec').value

        self.start_time = self.get_clock().now()
        self.safe_count = 0
        self.current_state = "SAFE"  # "SAFE", "WARNING", "EMERGENCY"
        self.robot_x = 0.0
        self.is_turning = False

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # /emergency_stop: True when obstacle is dangerously close
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        # /obstacle_alert: Human readable warning / voice synthesis / UI message
        self.alert_pub = self.create_publisher(String, '/obstacle_alert', 10)

        self.get_logger().info('ObstacleMonitor node started. Listening on %s...' % self.scan_topic)

    def cmd_callback(self, msg: Twist):
        # If robot is executing an intentional in-place turn, set is_turning flag
        self.is_turning = abs(msg.angular.z) > 0.25 and abs(msg.linear.x) < 0.1

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def scan_callback(self, msg: LaserScan):
        # Startup grace period check
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_grace:
            return

        min_front_distance = float('inf')

        effective_angle = self.front_angle * 0.6 if self.is_turning else self.front_angle
        effective_stop_dist = 0.28 if self.is_turning else self.stop_dist

        angle = msg.angle_min
        for r in msg.ranges:
            # Check validity of range reading (must be > 0.15m to avoid self-reflections)
            valid_min = max(msg.range_min, 0.15)
            if valid_min <= r <= msg.range_max:
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                if abs(norm_angle) <= effective_angle:
                    if r < min_front_distance:
                        min_front_distance = r
            angle += msg.angle_increment

        # Evaluate distance and trigger appropriate states (purely based on LiDAR distance)
        if min_front_distance < effective_stop_dist:
            self.safe_count = 0
            new_state = "EMERGENCY"
            alert_msg = f"[緊急停止] 前方に急な飛び出し・障害物を検知！距離: {min_front_distance:.2f}m"
            emergency_active = True
        elif min_front_distance < self.warn_dist:
            self.safe_count = 0
            new_state = "WARNING"
            alert_msg = f"[注意喚起] 前方に障害物接近中。減速待機します。距離: {min_front_distance:.2f}m"
            emergency_active = False
        else:
            self.safe_count += 1
            if self.safe_count >= self.required_clear_count:
                new_state = "SAFE"
                alert_msg = "前方安全確認完了。運行を継続・再開します。"
            else:
                new_state = self.current_state
                alert_msg = f"安全確認待機中... ({self.safe_count}/{self.required_clear_count})"
            emergency_active = False

        # Publish emergency stop boolean
        stop_msg = Bool()
        stop_msg.data = emergency_active
        self.emergency_pub.publish(stop_msg)

        # Publish alert text when state changes or during emergency
        if new_state != self.current_state or new_state == "EMERGENCY":
            str_msg = String()
            str_msg.data = f"[{new_state}] {alert_msg}"
            self.alert_pub.publish(str_msg)

            if new_state == "EMERGENCY":
                self.get_logger().warn(alert_msg)
            elif new_state == "WARNING":
                self.get_logger().info(alert_msg)
            elif new_state == "SAFE" and self.current_state != "SAFE":
                self.get_logger().info(alert_msg)

        self.current_state = new_state


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
