import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleStatus


class HeartBeatNode(LifecycleNode):
    def __init__(self, drone_ns: str = "drone_1", px4_ns: str = "px4_1"):
        super().__init__(f'{drone_ns}_hb_node')
        self.declare_parameter("drone_ns", drone_ns)
        self.declare_parameter("px4_ns", px4_ns)
        self.drone_ns = self.get_parameter("drone_ns").get_parameter_value().string_value
        self.px4_ns = self.get_parameter("px4_ns").get_parameter_value().string_value

        self.get_logger().info(f'[{self.drone_ns}] HeartBeatNode init for PX4 ns: {self.px4_ns}')
        self.offboard_control_mode_publisher = None
        self.timer_ = None
        self.vehicle_status = VehicleStatus()

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("on_configure")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'/{self.px4_ns}/fmu/in/offboard_control_mode', qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.timer_.cancel()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info('on_cleanup')
        if self.offboard_control_mode_publisher is not None:
            self.destroy_publisher(self.offboard_control_mode_publisher)
        if self.timer_ is not None:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info('on_activate')
        self.timer_.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info('on_deactivate')
        self.timer_.cancel()
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info('on_shutdown')
        if self.offboard_control_mode_publisher is not None:
            self.destroy_publisher(self.offboard_control_mode_publisher)
        if self.timer_ is not None:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()


def main(args=None):
    rclpy.init(args=args)
    node = HeartBeatNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
