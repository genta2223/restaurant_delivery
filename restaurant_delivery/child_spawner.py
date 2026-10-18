#!/usr/bin/env python3
"""Two Children Sequential Animator for Restaurant Simulation.

Defines a 2-Act Storyboard:
- **Act 1: Running Boy Dash (T = 2.0s ~ 5.0s)**:
  - 'running_boy' (Blue clothes) at X=1.0m dashes across robot's path from Y=2.0m down to Y=-1.8m.
  - Robot detects sudden intrusion, triggers EMERGENCY STOP, confirms clearance, and resumes.
  - Boy comes to a complete rest at safe zone (X=1.0, Y=-1.8) and never interferes with any aisle again.

- **Act 2: Rolling Girl Aisle Blockage (T >= 8.0s)**:
  - 'rolling_girl' (Pink clothes) at X=2.0m, Y=0.0m begins rolling continuously across Route A.
  - Robot approaches at X ~ 1.3m, detects prolonged blockage, and initiates autonomous detour to Route B.
"""

import math
import subprocess
import rclpy
from rclpy.node import Node


class StoryboardChildrenSpawner(Node):
    def __init__(self):
        super().__init__('child_spawner')

        # Boy Parameters (Act 1: cross over at toddler walking speed 0.22m/s from X=1.1, Y=1.2 to Y=-2.0)
        self.boy_name = 'running_boy'
        self.boy_x = 1.1
        self.boy_start_y = 1.2
        self.boy_target_y = -2.0
        self.boy_current_y = self.boy_start_y
        self.boy_speed = 0.22  # ~6.0s crossing in robot view, clears to Y=-2.0m
        self.boy_running = False
        self.boy_done = False

        # Girl Parameters (Act 2: initially waiting at seat X=3.0, Y=-1.2. At T>=8.5s enters aisle to X=2.8, Y=0.0 and rolls)
        self.girl_name = 'rolling_girl'
        self.girl_start_x = 3.0
        self.girl_target_x = 2.8
        self.girl_current_x = self.girl_start_x
        self.girl_seat_y = -1.2
        self.girl_target_y = 0.0
        self.girl_current_y = self.girl_seat_y
        self.girl_speed = 0.50
        self.girl_state = 'SEATED'  # 'SEATED', 'APPROACHING_AISLE', 'ROLLING'
        self.girl_roll = 0.0

        # Timer at 10 Hz (0.1s step)
        self.timer = self.create_timer(0.1, self.step_animation)
        self.init_time = self.get_clock().now()

        self.get_logger().info('StoryboardChildrenSpawner initialized: Act 1 (Boy Walk at 2.0s) -> Act 2 (Girl Enters & Rolls at 8.5s).')

    def send_gz_pose(self, model_name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        try:
            cmd = [
                'gz', 'model',
                '-m', model_name,
                '-x', f'{x:.3f}',
                '-y', f'{y:.3f}',
                '-z', f'{z:.3f}',
                '-R', f'{roll:.3f}',
                '-P', f'{pitch:.3f}',
                '-Y', f'{yaw:.3f}'
            ]
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def step_animation(self):
        elapsed = (self.get_clock().now() - self.init_time).nanoseconds / 1e9

        # --- Act 1: Running Boy Toddler Walk (T = 2.0s ~ ) ---
        if not self.boy_done:
            if not self.boy_running:
                if elapsed >= 2.0:
                    self.boy_running = True
                    self.get_logger().warn('【第1幕：男の子横断】男の子(running_boy)がロボット目前(X=1.1)をトコトコゆっくり横断開始！')
            else:
                dt = 0.1
                self.boy_current_y -= self.boy_speed * dt
                if self.boy_current_y <= self.boy_target_y:
                    self.boy_current_y = self.boy_target_y
                    self.boy_done = True
                    self.get_logger().info('【第1幕完了】男の子が下側通路外(Y=-2.0)へ退避完了。')

                self.send_gz_pose(self.boy_name, self.boy_x, self.boy_current_y, 0.0, yaw=-1.57)

        # --- Act 2: Girl Emerges from Seat and Rolls across Route A (Begins at T >= 8.5s) ---
        if elapsed >= 8.5:
            dt = 0.1
            if self.girl_state == 'SEATED':
                self.girl_state = 'APPROACHING_AISLE'
                self.get_logger().warn('【第2幕：女の子入場】女の子(rolling_girl)が客席(X=3.0, Y=-1.2)から最短通路(X=2.8, Y=0.0)へ降りてきます！')

            if self.girl_state == 'APPROACHING_AISLE':
                self.girl_current_y += self.girl_speed * dt
                self.girl_current_x -= 0.10 * dt
                if self.girl_current_x <= self.girl_target_x:
                    self.girl_current_x = self.girl_target_x

                if self.girl_current_y >= self.girl_target_y:
                    self.girl_current_y = self.girl_target_y
                    self.girl_current_x = self.girl_target_x
                    self.girl_state = 'ROLLING'
                    self.get_logger().warn('【第2幕：通路閉塞】女の子が中央通路(X=2.8, Y=0.0)でゴロゴロと横たわり、通路を塞ぎました！')

                # Standing/walking pose toward aisle
                self.send_gz_pose(
                    self.girl_name,
                    self.girl_current_x,
                    self.girl_current_y,
                    0.0,
                    roll=0.0,
                    pitch=0.0,
                    yaw=2.35
                )
            elif self.girl_state == 'ROLLING':
                self.girl_roll += 0.35
                offset_y = 0.10 * math.sin(self.girl_roll * 0.5)
                # Lying down and rolling horizontally across aisle (pitch=1.57, z=0.12)
                self.send_gz_pose(
                    self.girl_name,
                    self.girl_target_x,
                    self.girl_current_y + offset_y,
                    0.12,
                    roll=self.girl_roll,
                    pitch=1.57,
                    yaw=0.0
                )


def main(args=None):
    rclpy.init(args=args)
    node = StoryboardChildrenSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
