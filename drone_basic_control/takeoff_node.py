import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Float32MultiArray
import json
import fcntl


class TakeOffNode(LifecycleNode):
    """
    Streams TrajectorySetpoint continuously (even before OFFBOARD) so PX4 accepts OFFBOARD.
    Setpoints default to (0,0,takeoff_height) and can be updated via
    /{drone_ns}/flight_control/update_pos (Float32MultiArray [x,y,z]).
    """
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

        self.ts_pub = None
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.timer_ = None

    def on_configure(self, state: LifecycleState):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.ts_pub = self.create_publisher(
            TrajectorySetpoint, f'/{self.px4_ns}/fmu/in/trajectory_setpoint', qos
        )
        self.status_sub = self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status', self.status_cb, qos
        )
        self.lpos_sub = self.create_subscription(
            VehicleLocalPosition, f'/{self.px4_ns}/fmu/out/vehicle_local_position', self.lpos_cb, qos
        )
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, f'/{self.drone_ns}/flight_control/update_pos', self.cmd_cb, 10
        )
        self.timer_ = self.create_timer(0.1, self.timer_cb)  # 10 Hz stream
        self.timer_.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info(f'[{self.drone_ns}] TakeOffNode activated')
        # Seed initial setpoint (important pre-OFFBOARD)
        self._publish_ts(self.x, self.y, self.z)
        self.timer_.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        # Persist last known position (optional)
        try:
            with open('/home/ros2_ws/src/indoor_drone_flight/mission_cfg/cfg.json', 'r+') as f:
                fcntl.flock(f, fcntl.LOCK_EX)
                try:
                    data = json.load(f)
                except Exception:
                    data = {}
                data.setdefault(self.drone_ns, {})
                data[self.drone_ns]["current_pos"] = [
                    float(self.vehicle_local_position.x),
                    float(self.vehicle_local_position.y),
                    float(self.vehicle_local_position.z),
                ]
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()
        except Exception as e:
            self.get_logger().warn(f'[{self.drone_ns}] cfg.json write failed: {e}')
        finally:
            try:
                fcntl.flock(f, fcntl.LOCK_UN)
            except Exception:
                pass

        self.timer_.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState):
        if self.ts_pub:
            self.destroy_publisher(self.ts_pub)
        if self.timer_:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        if self.ts_pub:
            self.destroy_publisher(self.ts_pub)
        if self.timer_:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def lpos_cb(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def cmd_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.x, self.y, self.z = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
            self.get_logger().info(f'[{self.drone_ns}] New destination: {self.x:.2f}, {self.y:.2f}, {self.z:.2f}')

    def _publish_ts(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 1.57079  # 90 deg
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.ts_pub.publish(msg)

    def timer_cb(self):
        # ALWAYS stream setpoints so PX4 will enter OFFBOARD
        self._publish_ts(self.x, self.y, self.z)


def main(args=None):
    rclpy.init(args=args)
    node = TakeOffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
