import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Float32MultiArray
import json
import fcntl


class TakeOffNode(LifecycleNode):
    def __init__(self, drone_ns: str = "drone_1", px4_ns: str = "px4_1", takeoff_height: float = -3.0):
        super().__init__(f'{drone_ns}_takeoff_node')
        self.declare_parameter("drone_ns", drone_ns)
        self.declare_parameter("px4_ns", px4_ns)
        self.declare_parameter("takeoff_height", takeoff_height)

        self.drone_ns = self.get_parameter("drone_ns").get_parameter_value().string_value
        self.px4_ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        self.takeoff_height = self.get_parameter("takeoff_height").get_parameter_value().double_value

        self.get_logger().info(f'[{self.drone_ns}] TakeOffNode init for PX4 ns: {self.px4_ns}')
        self.x = 0.0
        self.y = 0.0
        self.z = self.takeoff_height

        self.trajectory_setpoint_publisher = None
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.timer_ = None

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("on_configure")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'/{self.px4_ns}/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'/{self.px4_ns}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile
        )
        # subscribe to this drone's setpoint update channel
        self.vehicle_command_sub = self.create_subscription(
            Float32MultiArray, f'/{self.drone_ns}/flight_control/update_pos', self.vehicle_cmd_callback, 10
        )

        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.timer_.cancel()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info('on_cleanup')
        if self.trajectory_setpoint_publisher is not None:
            self.destroy_publisher(self.trajectory_setpoint_publisher)
        if self.timer_ is not None:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info('on_activate')
        self.timer_.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        # Persist last known position (kept as-is from your code)
        try:
            with open('/home/ros2_ws/src/indoor_drone_flight/mission_cfg/cfg.json', 'r+') as file:
                fcntl.flock(file, fcntl.LOCK_EX)
                data = json.load(file)
                self.get_logger().info(f'[{self.drone_ns}] Writing current_pos to cfg.json')
                data.setdefault(self.drone_ns, {})
                data[self.drone_ns]["current_pos"] = [self.vehicle_local_position.x,
                                                      self.vehicle_local_position.y,
                                                      self.vehicle_local_position.z]
                file.seek(0)
                json.dump(data, file, indent=4)
                file.truncate()
        except Exception as e:
            self.get_logger().error(f'Failed to write cfg.json: {e}')
        finally:
            try:
                fcntl.flock(file, fcntl.LOCK_UN)
            except Exception:
                pass

        self.timer_.cancel()
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info('on_shutdown')
        if self.trajectory_setpoint_publisher is not None:
            self.destroy_publisher(self.trajectory_setpoint_publisher)
        if self.timer_ is not None:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # 90 deg
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def vehicle_cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.x, self.y, self.z = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
            self.get_logger().info(f"[{self.drone_ns}] New destination {self.x}, {self.y}, {self.z}")

    def timer_callback(self):
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.x, self.y, self.z)
