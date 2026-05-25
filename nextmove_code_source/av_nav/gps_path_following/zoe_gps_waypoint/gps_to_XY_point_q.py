# Modified: adds manual "q" keypress to stop recording and start conversion.
# Original authors: Maxime Duquesne (01/09/2023) @CRISTAL
# Edited by Zaynab EL MAWAS (05/06/2025) @CRIStAL
# Further edited: adds cross‑platform key listener for 'q' to trigger processing.

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

from geographiclib.geodesic import Geodesic
import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import threading
import time
import platform

# Helper: Convert Euler angles to quaternion
def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def _getch():
    """
    Cross-platform single-character read from stdin (no Enter required).
    Returns a single-character string, or None if stdin is unavailable.
    """
    try:
        if platform.system().lower().startswith("win"):
            import msvcrt
            ch = msvcrt.getch()
            try:
                ch = ch.decode()
            except Exception:
                pass
            return ch
        else:
            import termios, tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return ch
    except Exception:
        # stdin may be unavailable (e.g., when launched via ros2 launch)
        return None


class GpsToPath(Node):
    def __init__(self):
        super().__init__('pathtoxy')

        # Declare and retrieve parameters
        self.declare_parameter('gps_topic', '/novatel/oem7/fix')
        topic = self.get_parameter('gps_topic').get_parameter_value().string_value

        self.declare_parameter('pas', 0.2)  # Distance between points in meters
        self.pas = self.get_parameter('pas').get_parameter_value().double_value

        self.declare_parameter('nb_points_precision', 10)  # How many GPS points to average for start position
        self.nb_points_precision = self.get_parameter('nb_points_precision').get_parameter_value().integer_value

        # Initialize data holders
        self.gps_points = []                # List to store incoming GPS messages
        self.gps_start = NavSatFix()        # Averaged GPS start location
        self.gps = NavSatFix()              # Latest GPS fix
        self.path = Path()                  # Resulting path message
        self.path_timeless = Path()         # Resampled path with consistent spacing

        # State flags
        self.GPS_start = False
        self.stop = False

        # Timestamp of last received GPS message
        self.last_gps_time = self.get_clock().now().nanoseconds
        self.timeout_sec = 5.0  # How long to wait after GPS stops before shutting down

        # Subscribe to GPS topic
        self.sub_gps = self.create_subscription(NavSatFix, topic, self.callback_gps, 10)

        # Start periodic check to launch main_loop()
        self.main_timer = self.create_timer(1.0, self.try_start)

        # Start background key listener for manual stop with 'q'
        threading.Thread(target=self._key_listener, daemon=True).start()

        print("Press 'q' to stop recording and start conversion (no Enter needed).")

    def __del__(self):
        print("shutdown")

    # GPS callback — called when a GPS fix is received
    def callback_gps(self, data):
        print(f"GPS point #{len(self.gps_points) + 1}")
        self.GPS_start = True
        self.gps = data
        self.last_gps_time = self.get_clock().now().nanoseconds
        if not self.stop:
            self.gps_points.append(data)

    # Periodic check to start main processing when GPS stream is ready or ended
    def try_start(self):
        # If manual stop already requested, do nothing (main_loop already running)
        if self.stop:
            return

        current_time = self.get_clock().now().nanoseconds
        time_since_last_gps = (current_time - self.last_gps_time) * 1e-9

        if not self.GPS_start:
            self.get_logger().info("Waiting for GPS...")
            return

        # Automatic fallback: if GPS stops for timeout_sec, start processing
        if time_since_last_gps > self.timeout_sec:
            self.get_logger().info("No GPS received recently. Starting processing.")
            self._manual_stop()

    def _manual_stop(self):
        """Idempotent manual stop: stop collecting GPS and start processing."""
        if self.stop:
            return
        self.stop = True
        try:
            self.main_timer.cancel()
        except Exception:
            pass
        self.get_logger().info("Starting processing...")
        self.main_loop()

    def _key_listener(self):
        """Background key listener to trigger processing on 'q' key press."""
        # If stdin is unavailable, keep checking periodically.
        while not self.stop:
            ch = _getch()
            if ch is None:
                # No stdin available; back off a bit and try again.
                time.sleep(0.5)
                continue
            if ch.lower() == 'q':
                self.get_logger().info("Manual stop requested via 'q'.")
                self._manual_stop()
                return

    # Called once enough GPS points have been received
    def start(self):
        print("Save gps's start position, don't move ...")
        if len(self.gps_points) < self.nb_points_precision:
            print("⚠️  Not enough GPS points for averaging. Using first point.")
            self.gps_start = self.gps_points[0]
            return

        # Average latitude and longitude to define origin
        lat = sum(p.latitude for p in self.gps_points) / len(self.gps_points)
        lng = sum(p.longitude for p in self.gps_points) / len(self.gps_points)
        self.gps_start.latitude = lat
        self.gps_start.longitude = lng
        print("Start GPS position is:", lat, lng)

    # Compute XY from two GPS points using geodesic projection
    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        geod = Geodesic.WGS84
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
        distance = g['s12']
        azimuth = math.radians(g['azi1'])
        x = math.cos(azimuth) * distance
        y = math.sin(azimuth) * distance
        return x, y

    # Create Path message from all GPS points
    def add_gps_point(self):
        for fix in self.gps_points:
            X, Y = self.calc_goal(self.gps_start.latitude, self.gps_start.longitude, fix.latitude, fix.longitude)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_frame"
            pose.pose.position.x = float(X)
            pose.pose.position.y = float(Y)
            pose.pose.position.z = 0.0
            q = quaternion_from_euler(0.0, 0.0, 0.0)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.path.poses.append(pose)

    # Resample path to ensure even spacing (defined by self.pas)
    def timeless_path(self):
        self.path_timeless.header.frame_id = "base_frame"
        self.path_timeless.poses = []

        if not self.path.poses:
            return

        self.path_timeless.poses.append(self.path.poses[0])
        j = 0
        i = 0

        while i < len(self.path.poses):
            p = np.array([
                self.path_timeless.poses[j].pose.position.x,
                self.path_timeless.poses[j].pose.position.y
            ])
            q = np.array([
                self.path.poses[i].pose.position.x,
                self.path.poses[i].pose.position.y
            ])
            if np.linalg.norm(p - q) > self.pas:
                delta = q - p
                alpha = self.pas / np.linalg.norm(delta)
                new_pos = p + alpha * delta

                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "base_frame"
                pose.pose.position.x = float(new_pos[0])
                pose.pose.position.y = float(new_pos[1])
                pose.pose.position.z = 0.0
                qe = quaternion_from_euler(0.0, 0.0, 0.0)
                pose.pose.orientation.x = qe[0]
                pose.pose.orientation.y = qe[1]
                pose.pose.orientation.z = qe[2]
                pose.pose.orientation.w = qe[3]
                self.path_timeless.poses.append(pose)
                j += 1
            else:
                i += 1

    # Full processing loop: from averaging to saving
    def main_loop(self):
        print("GPS to XY process ...")
        self.start()
        self.add_gps_point()
        print("Processing the path with same distance between 2 points ...")
        self.timeless_path()

        Xaxis = [p.pose.position.x for p in self.path_timeless.poses]
        Yaxis = [p.pose.position.y for p in self.path_timeless.poses]

        ans = input("Would you like to print the plot? Y , N: ")
        if ans.strip().upper() == 'Y':
            plt.plot(Xaxis, Yaxis)
            plt.ylabel('Timeless')
            plt.axis('equal')
            plt.grid()
            plt.show()

        ans = input("Would you like to save the map? Y , N: ")
        if ans.strip().upper() == 'Y':
            name = input("Name of the file: ").strip()
            save = [self.gps_start.latitude, self.gps_start.longitude] + [len(self.path_timeless.poses)] + Xaxis + Yaxis
            with open(f"{name}.txt", "w") as textfile:
                for element in save:
                    textfile.write(str(element) + "\\n")
        sys.exit()


def main(args=None):
    rclpy.init(args=args)
    node = GpsToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
