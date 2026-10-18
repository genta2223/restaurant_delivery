#!/usr/bin/env python3
"""Delivery Controller Node for Restaurant Delivery Robot.

Controls autonomous delivery task:
1. Leaves kitchen counter (X=0.0, Y=0.0) carrying Omurice.
2. Approaches rolling child at X=1.2m ~ 1.3m (distance to obstacle ~ 0.7m).
3. Evaluates 2 seconds of blockage on direct Route A -> Triggers Autonomous Detour!
4. Detour execution:
   - Backs up slightly to X=0.9m to gain clearance.
   - Turns 90 degrees left toward Upper Aisle (Y=1.6m).
   - Travels along Upper Aisle past the blocked table zone (up to X=3.5m).
   - Turns right back into Main Aisle toward Target Customer Table (X=4.0m, Y=0.0m).
5. Completes Omurice delivery task cleanly.
"""

import math
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class RobotState(Enum):
    DELIVERING_ROUTE_A = 1
    BLOCKED_WAITING = 2
    DETOUR_BACKUP = 3
    DETOUR_TURN_LEFT = 4
    DETOUR_TRANSIT_Y = 5
    DETOUR_TURN_FORWARD = 6
    DETOUR_PASS_AISLE = 7
    DETOUR_RETURN_LINE = 8
    DELIVERY_COMPLETED = 9


class DeliveryController(Node):
    def __init__(self):
        super().__init__('delivery_controller')

        # Parameters
        self.declare_parameter('target_x', 4.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('goal_tolerance', 0.35)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

        self.state = RobotState.DELIVERING_ROUTE_A
        self.blocked_timer = 0
        self.seq_timer = 0
        self.min_front_scan = float('inf')
        self.emergency_stop_active = False

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.alert_pub = self.create_publisher(String, '/obstacle_alert', 10)

        # 10 Hz Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('DeliveryController initialized. Starting Omurice delivery on Route A.')

    def odom_callback(self, msg: Odometry):
        self.has_odom = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def emergency_callback(self, msg: Bool):
        self.emergency_stop_active = msg.data

    def scan_callback(self, msg: LaserScan):
        min_front = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.15 <= r <= 10.0:
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                if abs(norm_angle) <= math.radians(20.0):
                    if r < min_front:
                        min_front = r
            angle += msg.angle_increment
        self.min_front_scan = min_front

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.alert_pub.publish(msg)
        self.get_logger().info(f'[STATUS_DISPLAY] {text}')

    def control_loop(self):
        cmd = Twist()

        # Goal Completion Check
        dx_goal = self.target_x - self.current_x
        dy_goal = self.target_y - self.current_y
        dist_goal = math.hypot(dx_goal, dy_goal)

        if self.state == RobotState.DELIVERY_COMPLETED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        if self.current_x >= 3.8 and abs(self.current_y) <= 0.5 and dist_goal < self.goal_tolerance:
            self.state = RobotState.DELIVERY_COMPLETED
            self.publish_status('【配膳完了】オムライスをお届けしました！ごゆっくりお召し上がりください。')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        # ---------------- State Machine ----------------
        # 1. Cruise on Route A until girl blocks at X >= 1.3m
        if self.state == RobotState.DELIVERING_ROUTE_A:
            # Check for blocking child ahead near X=2.8 (Act 2)
            # Detected when robot reaches X >= 1.30m and obstacle is detected ahead (< 1.3m) or robot stops
            if self.current_x >= 1.30 and (self.min_front_scan < 1.30 or self.emergency_stop_active):
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.blocked_timer += 1

                if self.blocked_timer >= 15:  # 1.5 seconds stoppage in front of girl
                    self.publish_status('[REROUTING] 最短通路の閉塞を検知！上側迂回ルート(Y=1.8)へ切り替えます。')
                    self.state = RobotState.DETOUR_BACKUP
                    self.seq_timer = 0
                    self.get_logger().info('迂回判断完了。ステップ1: 後退クリアランス(2.0秒)を開始します。')
                else:
                    self.get_logger().warn(f'前方に通路を塞ぐ子どもを検知。一時停止して安全確認中 ({self.blocked_timer}/15)...')
                return

            # Boy crossing during early stage (X < 1.30)
            if self.emergency_stop_active:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.blocked_timer = 0
                self.get_logger().warn('【第1幕】男の子の飛び出し検知により緊急停止中！')
                return

            self.blocked_timer = 0

            # Keep straight on Y=0.0 line
            y_err = 0.0 - self.current_y
            yaw_err = 0.0 - self.current_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            cmd.linear.x = self.cruise_speed
            cmd.angular.z = max(min(2.0 * y_err + 1.2 * yaw_err, 0.35), -0.35)
            self.cmd_vel_pub.publish(cmd)

        # 2. Step 1: 後退クリアランス (linear.x = -0.10, angular.z = 0.0 を 2.0秒間)
        elif self.state == RobotState.DETOUR_BACKUP:
            self.seq_timer += 1
            cmd.linear.x = -0.10
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            if self.seq_timer >= 20:  # 2.0s (20 ticks at 10Hz)
                self.state = RobotState.DETOUR_TURN_LEFT
                self.seq_timer = 0
                self.get_logger().info('ステップ2: 左90度旋回(5.0秒)を開始します。')

        # 3. Step 2: 左90度旋回 (linear.x = 0.0, angular.z = 0.35 を 5.0秒間、真北を向く)
        elif self.state == RobotState.DETOUR_TURN_LEFT:
            self.seq_timer += 1
            cmd.linear.x = 0.0
            cmd.angular.z = 0.35
            self.cmd_vel_pub.publish(cmd)

            if self.seq_timer >= 50:  # 5.0s (50 ticks)
                self.state = RobotState.DETOUR_TRANSIT_Y
                self.seq_timer = 0
                self.get_logger().info('ステップ3: 上側通路へ直進(11.5秒)を開始します。')

        # 4. Step 3: 上側通路へ直進 (linear.x = 0.15, angular.z = 0.0 を 11.5秒間、Y=1.8mへ進入)
        elif self.state == RobotState.DETOUR_TRANSIT_Y:
            self.seq_timer += 1
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            if self.seq_timer >= 115:  # 11.5s (115 ticks at 10Hz, reaches Y ~ 1.8m)
                self.state = RobotState.DETOUR_TURN_FORWARD
                self.seq_timer = 0
                self.get_logger().info('ステップ4: 東向き・正面への右90度旋回(5.0秒)を開始します。')

        # 5. Step 4: 右90度旋回 (linear.x = 0.0, angular.z = -0.35 を 5.0秒間、東向きを向く)
        elif self.state == RobotState.DETOUR_TURN_FORWARD:
            self.seq_timer += 1
            cmd.linear.x = 0.0
            cmd.angular.z = -0.35
            self.cmd_vel_pub.publish(cmd)

            if self.seq_timer >= 50:  # 5.0s (50 ticks)
                self.state = RobotState.DETOUR_PASS_AISLE
                self.seq_timer = 0
                self.publish_status('[REROUTING] 上側迂回ルート(Y=1.8)を通過中。配膳先へ向かいます。')
                self.get_logger().info('ステップ5: 迂回通路前進(14.0秒)を開始します。')

        # 6. Step 5: 迂回通路前進 (linear.x = 0.18, angular.z = 0.0 を 14.0秒間、Y=1.8mの広々としたラインを前進)
        elif self.state == RobotState.DETOUR_PASS_AISLE:
            self.seq_timer += 1
            cmd.linear.x = 0.18
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            if self.seq_timer >= 140:  # 14.0s (140 ticks)
                self.state = RobotState.DETOUR_RETURN_LINE
                self.seq_timer = 0
                self.get_logger().info('ステップ6: 客席アプローチ・配膳(8.0秒)へ向かいます。')

        # 7. Step 6: 客席アプローチ＆配膳完了 (8.0秒間 目標テーブル正面へ斜めにアプローチ)
        elif self.state == RobotState.DETOUR_RETURN_LINE:
            self.seq_timer += 1
            if self.seq_timer < 80:  # 8.0s (80 ticks)
                cmd.linear.x = 0.12
                cmd.angular.z = -0.25
                self.cmd_vel_pub.publish(cmd)
            else:
                self.state = RobotState.DELIVERY_COMPLETED
                self.seq_timer = 0
                self.publish_status('【配膳完了】オムライスをお届けしました！ごゆっくりお召し上がりください。')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)

        # 8. Step 7: 到着後の余韻停止 (8.0秒間 停止保持)
        elif self.state == RobotState.DELIVERY_COMPLETED:
            self.seq_timer += 1
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
