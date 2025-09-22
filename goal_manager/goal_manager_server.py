#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from goal_manager_msgs.srv import GetNextGoal
import json
import paho.mqtt.client as mqtt
import math
import random
from typing import Optional, Tuple

# -----------------------
# Default configuration
# -----------------------
DEFAULT_BROKER = "broker.hivemq.com"
DEFAULT_PORT = 1883
DEFAULT_TOPIC = "colosseum/update"

# Reference home GPS for NED conversion (same as your original)
DEFAULT_HOME_LAT = 42.33894284868896
DEFAULT_HOME_LON = -71.08613491058351

# Per-drone publish topics
DRONE1_GOAL_TOPIC = "/drone_1/next_goal"
DRONE2_GOAL_TOPIC = "/drone_2/next_goal"

# Default Z setpoint (PX4 local frame uses Down-positive → typical takeoff altitude is negative)
DEFAULT_Z = -2.0

# Per-drone XY offsets applied relative to the "base" goal derived from GPS
DRONE1_OFFSET = (0.0, 0.0)   # (dx, dy) in meters
DRONE2_OFFSET = (2.0, 0.0)   # small separation to avoid same waypoint


class GoalManagerServer(Node):
    """
    Multi-drone goal manager:
      • Subscribes to MQTT (lat, lon) JSON messages (keys: "lat", "lon").
      • Converts to local meters (flat-earth approx) wrt HOME_LAT/LON.
      • Publishes PoseStamped goals to /drone_1/next_goal and /drone_2/next_goal,
        applying small configurable XY offsets per drone.
      • Provides GetNextGoal service that returns the last "base" goal (pre-offset).
    """

    def __init__(self):
        super().__init__('goal_manager_server')

        # --- Declare ROS parameters (override in launch if needed) ---
        self.declare_parameter('mqtt_broker', DEFAULT_BROKER)
        self.declare_parameter('mqtt_port', DEFAULT_PORT)
        self.declare_parameter('mqtt_topic', DEFAULT_TOPIC)

        self.declare_parameter('home_lat', DEFAULT_HOME_LAT)
        self.declare_parameter('home_lon', DEFAULT_HOME_LON)

        self.declare_parameter('default_z', DEFAULT_Z)

        # Per-drone offsets (meters)
        self.declare_parameter('drone1_offset_x', DRONE1_OFFSET[0])
        self.declare_parameter('drone1_offset_y', DRONE1_OFFSET[1])
        self.declare_parameter('drone2_offset_x', DRONE2_OFFSET[0])
        self.declare_parameter('drone2_offset_y', DRONE2_OFFSET[1])

        # Read parameters
        self.broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.port = int(self.get_parameter('mqtt_port').get_parameter_value().integer_value or DEFAULT_PORT)
        self.topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value

        self.HOME_LAT = float(self.get_parameter('home_lat').get_parameter_value().double_value or DEFAULT_HOME_LAT)
        self.HOME_LON = float(self.get_parameter('home_lon').get_parameter_value().double_value or DEFAULT_HOME_LON)

        self.default_z = float(self.get_parameter('default_z').get_parameter_value().double_value or DEFAULT_Z)

        self.drone1_offset = (
            float(self.get_parameter('drone1_offset_x').get_parameter_value().double_value or DRONE1_OFFSET[0]),
            float(self.get_parameter('drone1_offset_y').get_parameter_value().double_value or DRONE1_OFFSET[1]),
        )
        self.drone2_offset = (
            float(self.get_parameter('drone2_offset_x').get_parameter_value().double_value or DRONE2_OFFSET[0]),
            float(self.get_parameter('drone2_offset_y').get_parameter_value().double_value or DRONE2_OFFSET[1]),
        )

        # Publishers for each drone's goal topic
        self.pub_drone1 = self.create_publisher(PoseStamped, DRONE1_GOAL_TOPIC, 10)
        self.pub_drone2 = self.create_publisher(PoseStamped, DRONE2_GOAL_TOPIC, 10)

        # Store the latest GPS and latest "base" PoseStamped (pre-offset)
        self.latest_gps: Optional[Tuple[float, float]] = None
        self.latest_base_pose: Optional[PoseStamped] = None

        # MQTT client setup
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            self.get_logger().info(f"Connected to MQTT broker {self.broker}:{self.port}, subscribing to '{self.topic}'")
        except Exception as e:
            self.get_logger().error(f"Failed to connect MQTT: {e}")

        # ROS2 service
        self.srv = self.create_service(GetNextGoal, 'get_next_goal', self.handle_get_next_goal)
        self.get_logger().info("Goal Manager Server started.")

        # Optional safety timer: if no MQTT yet, publish random goals periodically (every 5s)
        self.timer = self.create_timer(5.0, self._timer_publish_fallback)

    # -----------------------
    # MQTT callbacks
    # -----------------------
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            try:
                client.subscribe(self.topic)
                self.get_logger().info(f"MQTT connected. Subscribed to '{self.topic}'")
            except Exception as e:
                self.get_logger().error(f"MQTT subscribe failed: {e}")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            lat = float(data.get("lat"))
            lon = float(data.get("lon"))
            self.latest_gps = (lat, lon)
            self.get_logger().info(f"Received GPS via MQTT -> lat={lat:.8f}, lon={lon:.8f}")

            # Convert to local meters (base waypoint)
            x, y = self.gps_to_local(lat, lon)
            base_pose = self.make_pose(x, y, self.default_z)
            self.latest_base_pose = base_pose

            # Publish per-drone goals with offsets
            self.publish_drone_goals(base_pose)

        except Exception as e:
            self.get_logger().error(f"Failed to parse MQTT message: {e}")

    # -----------------------
    # Utilities
    # -----------------------
    def gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Simple flat-earth approximation around HOME_LAT/LON
        """
        dlat = lat - self.HOME_LAT
        dlon = lon - self.HOME_LON
        x = dlat * 111_000.0  # meters per degree latitude
        y = dlon * 111_000.0 * math.cos(math.radians(self.HOME_LAT))
        return x, y

    def make_pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0  # yaw=0
        return pose

    def publish_drone_goals(self, base_pose: PoseStamped):
        # Drone 1
        p1 = PoseStamped()
        p1.header = base_pose.header
        p1.pose = base_pose.pose
        p1.pose.position.x += self.drone1_offset[0]
        p1.pose.position.y += self.drone1_offset[1]
        self.pub_drone1.publish(p1)

        # Drone 2
        p2 = PoseStamped()
        p2.header = base_pose.header
        p2.pose = base_pose.pose
        p2.pose.position.x += self.drone2_offset[0]
        p2.pose.position.y += self.drone2_offset[1]
        self.pub_drone2.publish(p2)

        self.get_logger().info(
            f"Published goals | "
            f"drone_1: ({p1.pose.position.x:.2f}, {p1.pose.position.y:.2f}, {p1.pose.position.z:.2f})  "
            f"drone_2: ({p2.pose.position.x:.2f}, {p2.pose.position.y:.2f}, {p2.pose.position.z:.2f})"
        )

    # -----------------------
    # ROS2 Service
    # -----------------------
    def handle_get_next_goal(self, request, response):
        """
        Return the last base goal (pre-offset). If we have no GPS yet, return a fallback random goal.
        """
        if self.latest_base_pose is not None:
            response.goal_pose = self.latest_base_pose
            return response

        # Fallback: random goal near origin
        pose = self.make_pose(
            random.uniform(-10.0, 10.0),
            random.uniform(-10.0, 10.0),
            self.default_z
        )
        self.get_logger().warn("No GPS received yet; returning random fallback goal via service")
        response.goal_pose = pose
        return response

    # -----------------------
    # Fallback timer
    # -----------------------
    def _timer_publish_fallback(self):
        """
        If no MQTT GPS has been received yet, periodically publish small random goals
        to both drones so the pipeline is testable end-to-end.
        (Disable by removing or increasing the timer interval.)
        """
        if self.latest_base_pose is not None:
            return  # we already have live GPS; no need for fallback

        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)
        base_pose = self.make_pose(x, y, self.default_z)
        self.latest_base_pose = base_pose
        self.publish_drone_goals(base_pose)
        self.get_logger().warn("Fallback publish (no MQTT yet): sent random goals to both drones")


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.client.loop_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
