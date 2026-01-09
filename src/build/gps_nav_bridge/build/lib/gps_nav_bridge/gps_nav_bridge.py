import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

import serial
from geographiclib.geodesic import Geodesic


def yaw_to_quat(yaw_rad: float) -> Quaternion:
    # Z-only quaternion
    q = Quaternion()
    q.w = math.cos(yaw_rad * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    return q


class GpsNavBridge(Node):
    """
    - Reads JSON lines from Arduino over USB serial
    - Publishes /gps/fix (NavSatFix)
    - Subscribes /gps/goal (NavSatFix containing target lat/lon)
      -> converts to local map x/y using datum (lat0, lon0) + ENU
      -> sends NavigateToPose goal to Nav2 BT Navigator
    """

    def __init__(self):
        super().__init__('gps_nav_bridge')

        # --- Params ---
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = int(self.declare_parameter('baud', 115200).value)

        # Datum: your map origin (where x=0,y=0). Put a surveyed point or where you started the map.
        self.datum_lat = float(self.declare_parameter('datum_lat', 0.0).value)
        self.datum_lon = float(self.declare_parameter('datum_lon', 0.0).value)
        self.datum_alt = float(self.declare_parameter('datum_alt', 0.0).value)

        # Map yaw offset (radians): if your map frame isn't aligned ENU, rotate here.
        # 0 means: +x=East, +y=North (ENU)
        self.map_yaw_offset_deg = float(self.declare_parameter('map_yaw_offset_deg', 0.0).value)
        self.map_yaw_offset = math.radians(self.map_yaw_offset_deg)

        # Frames / topics
        self.frame_id = self.declare_parameter('gps_frame_id', 'gps_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value

        self.fix_topic = self.declare_parameter('fix_topic', '/gps/fix').value
        self.goal_topic = self.declare_parameter('goal_topic', '/gps/goal').value

        # If you want to also publish the converted goal pose for RViz
        self.goal_pose_topic = self.declare_parameter('goal_pose_topic', '/gps/goal_pose').value

        # Nav2 action server name (BT Navigator)
        self.nav2_action_name = self.declare_parameter('nav2_action_name', '/navigate_to_pose').value

        # Publish rate sanity
        self.max_read_hz = float(self.declare_parameter('max_read_hz', 10.0).value)

        # --- ROS pubs/subs ---
        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, 10)
        self.goal_sub = self.create_subscription(NavSatFix, self.goal_topic, self.on_goal_gps, 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        # Serial thread
        self._serial = None
        self._stop = False
        self._thread = threading.Thread(target=self.serial_worker, daemon=True)

        if self.datum_lat == 0.0 and self.datum_lon == 0.0:
            self.get_logger().warn(
                "datum_lat/datum_lon are still 0.0. "
                "Set them to your map origin or the conversion won't be meaningful."
            )

        self.open_serial()
        self._thread.start()

        self.get_logger().info(
            f"Running. Serial {self.port}@{self.baud}. "
            f"Publishing {self.fix_topic}. Listening {self.goal_topic}. "
            f"Sending Nav2 goals to {self.nav2_action_name} in frame '{self.map_frame}'."
        )

    def open_serial(self):
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.2)
            # Give Arduino time to reset on serial open
            time.sleep(1.5)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial {self.port}: {e}")
            raise

    def destroy_node(self):
        self._stop = True
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        super().destroy_node()

    def serial_worker(self):
        # Read loop (thread)
        min_dt = 1.0 / max(1.0, self.max_read_hz)
        last_pub = 0.0

        while rclpy.ok() and not self._stop:
            try:
                line = self._serial.readline().decode('utf-8', errors='ignore').strip()
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.5)
                continue

            if not line:
                continue

            # Throttle to max_read_hz if Arduino floods
            now = time.time()
            if (now - last_pub) < min_dt:
                continue

            # Parse JSON
            try:
                d = json.loads(line)
            except Exception:
                continue

            if isinstance(d, dict) and d.get("status") == "gps_bridge_ready":
                self.get_logger().info("Arduino reports gps_bridge_ready")
                continue

            fix = int(d.get("fix", 0))
            lat = d.get("lat", None)
            lon = d.get("lon", None)
            alt = float(d.get("alt", 0.0))

            if fix == 0 or lat is None or lon is None:
                continue

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            msg.latitude = float(lat)
            msg.longitude = float(lon)
            msg.altitude = float(alt)

            # If you have better covariance, set it. We'll mark UNKNOWN by default.
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.fix_pub.publish(msg)
            last_pub = now

    def latlon_to_local_xy(self, lat: float, lon: float):
        """
        Convert (lat,lon) to local ENU meters relative to datum using WGS84 geodesic.
        Output: (x_east, y_north) in meters.
        """
        geod = Geodesic.WGS84.Inverse(self.datum_lat, self.datum_lon, lat, lon)
        distance_m = geod['s12']           # meters
        azimuth_deg = geod['azi1']         # degrees from north, clockwise

        az = math.radians(azimuth_deg)
        # ENU: East = s*sin(az), North = s*cos(az)
        x_e = distance_m * math.sin(az)
        y_n = distance_m * math.cos(az)
        return x_e, y_n

    def enu_to_map(self, x_e: float, y_n: float):
        """
        Rotate ENU into map frame by yaw offset.
        """
        c = math.cos(self.map_yaw_offset)
        s = math.sin(self.map_yaw_offset)
        x_map = c * x_e - s * y_n
        y_map = s * x_e + c * y_n
        return x_map, y_map

    def on_goal_gps(self, msg: NavSatFix):
        # Convert goal lat/lon -> map x/y, then send Nav2 NavigateToPose
        lat = msg.latitude
        lon = msg.longitude

        if self.datum_lat == 0.0 and self.datum_lon == 0.0:
            self.get_logger().error("Refusing goal: datum_lat/datum_lon are not set.")
            return

        x_e, y_n = self.latlon_to_local_xy(lat, lon)
        x_map, y_map = self.enu_to_map(x_e, y_n)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.map_frame

        goal_pose.pose.position.x = float(x_map)
        goal_pose.pose.position.y = float(y_map)
        goal_pose.pose.position.z = 0.0

        # If you want heading toward target, you'd compute it from current pose.
        # We'll set yaw=0 for now; Nav2 can still navigate position-only.
        goal_pose.pose.orientation = yaw_to_quat(0.0)

        # Publish for RViz debugging
        self.goal_pose_pub.publish(goal_pose)

        # Send action goal to Nav2
        self.send_nav2_goal(goal_pose)

        self.get_logger().info(
            f"GPS goal lat/lon=({lat:.7f},{lon:.7f}) -> map x/y=({x_map:.2f},{y_map:.2f}) sent to Nav2"
        )

    def send_nav2_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Nav2 action server not available: {self.nav2_action_name}")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal was rejected.")
            return

        self.get_logger().info("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Nav2 result received: {result}")


def main():
    rclpy.init()
    node = GpsNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
