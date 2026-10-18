#!/usr/bin/env python3
"""Debug Recorder Node for Restaurant Delivery Simulation.

Subscribes to:
- /debug_camera/image_raw (sensor_msgs/Image)
- /emergency_stop (std_msgs/Bool)
- /obstacle_alert (std_msgs/String)
- /scan (sensor_msgs/LaserScan)

Records frames at 1 Hz, overlays real-time telemetry, saves JPG frames,
writes formatted execution logs, and on shutdown compiles GIF and contact sheet.
Includes a fail-safe fallback directory mechanism to guarantee output files.
"""

import atexit
import math
import os
import shutil
import signal
import sys
import numpy as np
from PIL import Image as PILImage
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String


class DebugRecorder(Node):
    def __init__(self):
        super().__init__('debug_recorder')

        # Parameters
        self.declare_parameter('output_dir', '')
        self.declare_parameter('record_fps', 4.0)
        self.declare_parameter('gif_duration_ms', 250)

        out_param = self.get_parameter('output_dir').value
        if out_param:
            target_dir = out_param
        else:
            target_dir = '/home/ubuntu/pai_ws/src/restaurant_delivery/debug_output'

        # Verify directory writability, fallback to /tmp/debug_output if needed
        self.output_dir = target_dir
        self.fallback_dir = '/tmp/debug_output'
        self.using_fallback = False

        self._ensure_writable_directory()
        self._cleanup_old_frames()

        self.record_interval = 1.0 / self.get_parameter('record_fps').value
        self.gif_duration_ms = self.get_parameter('gif_duration_ms').value

        # Initialize execution_log.txt
        self._init_execution_log()

        self.latest_cv_image = None
        self.latest_emergency = False
        self.latest_alert = "NORMAL RUNNING"
        self.min_obstacle_dist = float('inf')
        self.frame_paths = []
        self.frame_count = 0
        self.has_finished_export = False

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/debug_camera/image_raw',
            self.image_callback,
            10
        )
        self.stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.stop_callback,
            10
        )
        self.alert_sub = self.create_subscription(
            String,
            '/obstacle_alert',
            self.alert_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 1 Hz Timer to capture and write frame
        self.timer = self.create_timer(self.record_interval, self.record_frame_callback)

        self.get_logger().info(
            f'DebugRecorder started. Saving frames to: {self.output_dir} '
            f'(Fallback: {self.fallback_dir}) (Rate: {1.0/self.record_interval:.1f} Hz)'
        )

    def _ensure_writable_directory(self):
        """Ensure self.output_dir is writable; otherwise fallback to self.fallback_dir."""
        for d in [self.output_dir, self.fallback_dir]:
            try:
                os.makedirs(d, exist_ok=True)
                try:
                    os.chmod(d, 0o777)
                except Exception:
                    pass
                test_file = os.path.join(d, '.test_write')
                with open(test_file, 'w') as f:
                    f.write('ok')
                os.remove(test_file)
                self.output_dir = d
                if d == self.fallback_dir:
                    self.using_fallback = True
                return
            except Exception as e:
                self.get_logger().warn(f'Directory {d} is not writable: {e}')

        self.output_dir = '/tmp'

    def _cleanup_old_frames(self):
        """Remove any leftover frame_*.jpg to keep output compact and prevent Windows freeze."""
        try:
            if os.path.isdir(self.output_dir):
                count = 0
                for fname in os.listdir(self.output_dir):
                    if fname.startswith('frame_') and fname.endswith('.jpg'):
                        try:
                            os.remove(os.path.join(self.output_dir, fname))
                            count += 1
                        except Exception:
                            pass
                if count > 0:
                    self.get_logger().info(f'Cleaned up {count} old frame JPGs from {self.output_dir}')
        except Exception as e:
            self.get_logger().warn(f'Frame cleanup warning: {e}')

    def _init_execution_log(self):
        self.log_filepath = os.path.join(self.output_dir, 'execution_log.txt')
        try:
            with open(self.log_filepath, 'w', encoding='utf-8') as f:
                f.write("=== Restaurant Delivery Robot Simulation Execution Log ===\n")
                f.write("Time(s) | Status | Min Obstacle (m) | Alert Message\n")
                f.write("-" * 75 + "\n")
            try:
                os.chmod(self.log_filepath, 0o666)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f"Cannot initialize log at {self.log_filepath}: {e}")

    def image_callback(self, msg: Image):
        try:
            if msg.encoding in ['rgb8', 'R8G8B8']:
                img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                self.latest_cv_image = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ['bgr8', 'BGR8']:
                self.latest_cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            elif msg.encoding == 'rgba8':
                img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
                self.latest_cv_image = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)
            else:
                img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                if img_np.shape[2] == 3:
                    self.latest_cv_image = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')

    def stop_callback(self, msg: Bool):
        self.latest_emergency = msg.data

    def alert_callback(self, msg: String):
        self.latest_alert = msg.data

    def scan_callback(self, msg: LaserScan):
        min_d = float('inf')
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and not math.isinf(r) and not math.isnan(r):
                if r < min_d:
                    min_d = r
        self.min_obstacle_dist = min_d

    def record_frame_callback(self):
        if self.latest_cv_image is None:
            return

        self.frame_count += 1
        frame = self.latest_cv_image.copy()
        h, w, _ = frame.shape

        # Semi-transparent header overlay bar for telemetry
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 75), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

        # Telemetry Texts
        sim_sec = self.frame_count * self.record_interval
        time_str = f"Time: {sim_sec:.1f}s | Frame #{self.frame_count:03d}"
        if "REROUTING" in self.latest_alert or "Route B" in self.latest_alert:
            status_text = "[REROUTING / DETOUR]"
            status_color = (0, 200, 255)  # Orange/Yellow (BGR)
        elif "配膳完了" in self.latest_alert:
            status_text = "[DELIVERY COMPLETED]"
            status_color = (255, 200, 0)  # Cyan/Gold (BGR)
        elif self.latest_emergency:
            status_text = "[EMERGENCY STOP]"
            status_color = (0, 0, 255)  # Red (BGR)
        else:
            status_text = "[DELIVERING / ACTIVE]"
            status_color = (0, 255, 0)  # Green (BGR)

        dist_val_str = f"{self.min_obstacle_dist:.2f}m" if self.min_obstacle_dist < 20.0 else "--"
        dist_str = f"Min Obstacle: {dist_val_str}"

        # Append to execution_log.txt
        if self.log_filepath:
            try:
                log_line = f"[{sim_sec:5.1f}s] {status_text:<22} | Dist: {dist_val_str:<6} | {self.latest_alert}\n"
                with open(self.log_filepath, 'a', encoding='utf-8') as f:
                    f.write(log_line)
            except Exception as e:
                self.get_logger().error(f"Failed to append execution log: {e}")

        # Draw ASCII telemetry text on image frame
        if "REROUTING" in self.latest_alert:
            short_alert = "Corridor Blocked -> Detour Route B"
        elif "配膳完了" in self.latest_alert:
            short_alert = "Omurice Served to Table"
        elif self.latest_emergency:
            short_alert = "Child Intrusion Alert!"
        else:
            short_alert = "Path Clear"

        cv2.putText(frame, f"{time_str}   {status_text}", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
        cv2.putText(frame, f"{dist_str} | Status: {short_alert}", (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1)

        # Save frame to disk (try primary, fallback if error)
        filename = f"frame_{self.frame_count:03d}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        saved = False
        try:
            saved = cv2.imwrite(filepath, frame)
            if saved:
                try:
                    os.chmod(filepath, 0o666)
                except Exception:
                    pass
                self.frame_paths.append(filepath)
        except Exception:
            saved = False

        if not saved:
            # Fallback save to /tmp/debug_output
            fb_path = os.path.join(self.fallback_dir, filename)
            try:
                os.makedirs(self.fallback_dir, exist_ok=True)
                cv2.imwrite(fb_path, frame)
                try:
                    os.chmod(fb_path, 0o666)
                except Exception:
                    pass
                self.frame_paths.append(fb_path)
            except Exception as e:
                self.get_logger().error(f"Fallback save failed for {fb_path}: {e}")

    def export_summary_assets(self):
        if self.has_finished_export:
            return
        self.has_finished_export = True

        valid_frames = [fp for fp in self.frame_paths if os.path.exists(fp) and os.path.getsize(fp) > 0]
        if not valid_frames:
            self.get_logger().warn('No valid captured frames found to generate GIF or contact sheet.')
            return

        self.get_logger().info(f'Compiling {len(valid_frames)} recorded frames into GIF and contact sheet...')

        try:
            # 1. Compile GIF animation using PIL
            try:
                resample_filter = PILImage.Resampling.LANCZOS
            except AttributeError:
                resample_filter = getattr(PILImage, 'ANTIALIAS', 1)

            pil_images = []
            for fp in valid_frames:
                try:
                    img = PILImage.open(fp).convert('RGB')
                    img = img.resize((480, 360), resample_filter)
                    pil_images.append(img)
                except Exception as ex:
                    self.get_logger().warn(f'Error reading frame {fp} for GIF: {ex}')

            if pil_images:
                gif_path = os.path.join(self.output_dir, 'simulation_debug.gif')
                try:
                    pil_images[0].save(
                        gif_path,
                        save_all=True,
                        append_images=pil_images[1:],
                        duration=self.gif_duration_ms,
                        loop=0
                    )
                    try:
                        os.chmod(gif_path, 0o666)
                    except Exception:
                        pass
                    self.get_logger().info(f'Successfully generated GIF: {gif_path}')
                except Exception as ex:
                    fb_gif = os.path.join(self.fallback_dir, 'simulation_debug.gif')
                    pil_images[0].save(fb_gif, save_all=True, append_images=pil_images[1:], duration=self.gif_duration_ms, loop=0)
                    self.get_logger().info(f'Saved fallback GIF: {fb_gif}')

            # 2. Compile MP4 Video (simulation_demo.mp4) at 4.0 FPS
            mp4_path = os.path.join(self.output_dir, 'simulation_demo.mp4')
            try:
                first_img = cv2.imread(valid_frames[0])
                if first_img is not None:
                    h, w, _ = first_img.shape
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    video_writer = cv2.VideoWriter(mp4_path, fourcc, 4.0, (w, h))
                    for fp in valid_frames:
                        img = cv2.imread(fp)
                        if img is not None:
                            video_writer.write(img)
                    video_writer.release()
                    try:
                        os.chmod(mp4_path, 0o666)
                    except Exception:
                        pass
                    self.get_logger().info(f'Successfully generated MP4 video: {mp4_path}')
            except Exception as ex_mp4:
                self.get_logger().warn(f'Failed generating MP4 with OpenCV: {ex_mp4}')

            # 3. Compile Contact Sheet
            selected_indices = np.linspace(0, len(valid_frames) - 1, min(len(valid_frames), 6), dtype=int)
            key_imgs = []
            for idx in selected_indices:
                loaded = cv2.imread(valid_frames[idx])
                if loaded is not None:
                    key_imgs.append(cv2.resize(loaded, (320, 240)))

            if len(key_imgs) >= 1:
                while len(key_imgs) < 6:
                    key_imgs.append(np.zeros_like(key_imgs[0]))
                row1 = np.hstack(key_imgs[:3])
                row2 = np.hstack(key_imgs[3:6])
                contact_sheet = np.vstack([row1, row2])
                sheet_path = os.path.join(self.output_dir, 'contact_sheet.jpg')
                try:
                    cv2.imwrite(sheet_path, contact_sheet)
                    try:
                        os.chmod(sheet_path, 0o666)
                    except Exception:
                        pass
                    self.get_logger().info(f'Successfully generated Contact Sheet: {sheet_path}')
                except Exception:
                    fb_sheet = os.path.join(self.fallback_dir, 'contact_sheet.jpg')
                    cv2.imwrite(fb_sheet, contact_sheet)
                    self.get_logger().info(f'Saved fallback Contact Sheet: {fb_sheet}')

            # 3. Synchronize fallback directory to target directory if needed
            target_final = '/home/ubuntu/pai_ws/src/restaurant_delivery/debug_output'
            if self.output_dir != target_final or self.using_fallback:
                try:
                    for fname in os.listdir(self.fallback_dir):
                        src = os.path.join(self.fallback_dir, fname)
                        dst = os.path.join(target_final, fname)
                        shutil.copy2(src, dst)
                        try:
                            os.chmod(dst, 0o666)
                        except Exception:
                            pass
                except Exception:
                    pass

        except Exception as e:
            self.get_logger().error(f'Failed to export summary assets: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DebugRecorder()

    def handle_exit(signum=None, frame=None):
        node.export_summary_assets()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)
    atexit.register(node.export_summary_assets)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.export_summary_assets()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
