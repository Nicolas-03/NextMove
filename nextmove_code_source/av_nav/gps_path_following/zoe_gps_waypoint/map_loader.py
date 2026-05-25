import os
import yaml
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from zoe_waypoint_interfaces.srv import GetPath, GetPathList
import logging
from typing import Optional
from io import TextIOWrapper as TextIO


GPS_COORD_IDX     = [0, 1]
BEGIN_Y_COORD_IDX = 2


def coord_to_pose_stamped(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


qos = QoSProfile(depth=1)
qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL


class LoadedPath:
    def __init__(self, filename: str, path: Path, gps_start: NavSatFix):
        self.filename  = filename
        self.path      = path
        self.path.header.frame_id = "map"
        self.path.header.stamp = rclpy.time.Time().to_msg()
        self.gps_start = gps_start


class MapLoader(Node):
    """
    ROS2 Node that:
    - reads a YAML config file passed as a parameter
    - loads the absolute path files listed in that YAML
    - serves them via GetPathList / GetPath services
    """
    def __init__(self):
        super().__init__('waypoint_map_loader')
        self._loaded_paths: dict[str, LoadedPath] = {}

        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        if not config_file:
            self.get_logger().error("Missing or empty 'config_file' parameter.")
            return

        absolute_paths = self._load_config(config_file)
        if absolute_paths is None:
            return

        for absolute_path in absolute_paths:
            self._load_path_file(absolute_path)

        self.create_service(GetPathList, 'get_path_list', self._cb_get_path_list)
        self.create_service(GetPath,     'get_path',      self._cb_get_path)

        self.get_logger().info(
            f"MapLoader ready — {len(self._loaded_paths)} path(s) loaded."
        )

    def _load_config(self, config_file: str) -> Optional[list[str]]:
        """
        Parse the YAML config file and return the list of absolute paths.

        Expected YAML format:
            paths:
              - /absolute/path/to/file1
              - /absolute/path/to/file2
        """
        if not os.path.isfile(config_file):
            self.get_logger().error(f"Config file not found: '{config_file}'")
            return None

        try:
            with open(config_file, 'r') as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().error(f"Error reading config: {e}")
            return None

        if not isinstance(data, dict) or 'paths' not in data:
            self.get_logger().error("Missing 'paths' key in the YAML file.")
            return None

        paths = data['paths']
        if not isinstance(paths, list):
            self.get_logger().error("'paths' must be a list in the YAML file.")
            return None

        self.get_logger().info(f"{len(paths)} path(s) found in '{config_file}'.")
        return paths

    def _load_path_file(self, absolute_path: str) -> None:
        """Open a single path file, parse it, and store it in the cache."""
        if not os.path.isfile(absolute_path):
            self.get_logger().warn(f"File not found: '{absolute_path}'")
            return

        filename = os.path.basename(absolute_path)

        try:
            fd = open(absolute_path, 'r')
        except OSError as e:
            self.get_logger().warn(f"Can't read '{filename}' : {e}")
            return

        path, gps_start = self._parse_path_file(fd)
        fd.close()

        if path is None or gps_start is None:
            self.get_logger().warn(f"Parsing failed: '{filename}'")
            return

        self._loaded_paths[filename] = LoadedPath(filename, path, gps_start)
        self.get_logger().info(f"Loaded: '{filename}' ({len(path.poses)} pts)")

    def _parse_path_file(self, fd: TextIO) -> tuple[Optional[Path], Optional[NavSatFix]]:
        """
        Parse a path file and return a Path and a NavSatFix.

        Expected file format:
            line 0        : latitude
            line 1        : longitude
            line 2        : index where Y coordinates begin
            lines 3..N    : X coordinates
            lines N+1..   : Y coordinates
        """
        try:
            lines = fd.readlines()
        except UnicodeDecodeError as e:
            logging.error(f"{e}. Path not loaded.")
            return None, None

        path      = Path()
        gps_start = NavSatFix()
        gps_start.latitude  = float(lines[GPS_COORD_IDX[0]])
        gps_start.longitude = float(lines[GPS_COORD_IDX[1]])
        begin_y: int = int(lines[BEGIN_Y_COORD_IDX])

        for i in range(BEGIN_Y_COORD_IDX + 1, begin_y):
            try:
                x = float(lines[i])
                y = float(lines[i + begin_y])
            except (ValueError, IndexError):
                continue
            path.poses.append(coord_to_pose_stamped(x, y))

        return path, gps_start

    def _cb_get_path_list(
            self,
            request: GetPathList.Request,
            response: GetPathList.Response) -> GetPathList.Response:
        response.filenames = list(self._loaded_paths.keys())
        self.get_logger().info(f"get_path_list → {response.filenames}")
        return response

    def _cb_get_path(
            self,
            request: GetPath.Request,
            response: GetPath.Response) -> GetPath.Response:
        response.success = False
        filename = request.filename

        if filename not in self._loaded_paths:
            self.get_logger().warn(f"'{filename}' unknown.")
            return response

        entry              = self._loaded_paths[filename]
        response.success   = True
        response.path      = entry.path
        response.gps_start = entry.gps_start
        return response


def main(args=None):
    rclpy.init(args=args)
    map_loader = MapLoader()

    try:
        rclpy.spin(map_loader)
    except KeyboardInterrupt:
        print(" Shutting down MapLoader.")
    map_loader.destroy_node()