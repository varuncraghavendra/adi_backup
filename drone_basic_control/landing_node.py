import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus


class LandingNode(LifecycleNode):
    def __init__(self, drone_ns: str = "drone_1", px4_ns: str = "px4_1", px4_sys_id: int = 1):
        super().__init__(f'{drone_ns}_landing_node')
        self.declare_parameter("drone_ns", drone_ns)
        self.declare_parameter("px4_ns", px4_ns)
        self.declare_parameter("px4_sys_id", px4_sys_id)

        self.drone_ns = self.get_parameter("drone_ns").get_parameter_value().string_value
        self.px4_ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        self.px4_sys_id = int(self.get_parameter("px4_sys_id").value)

        self.get_logger().info(f'[{self.drone_ns}] LandingNode init for PX4 ns: {self.px4_ns} (sys_id={self.px4_sys_id})')
        self.vehicle_command_publisher = None
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
        self.vehicle_command_publisher = self.create_lifecycle_publisher(
            VehicleCommand, f'/{self.px4_ns}/fmu/in/vehicle_command', qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.timer_.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info('on_cleanup')
        if self.vehicle_command_publisher is not None:
            self.destroy_lifecycle_publisher(self.vehicle_command_publisher)
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
        if self.vehicle_command_publisher is not None:
            self.destroy_lifecycle_publisher(self.vehicle_command_publisher)
        if self.timer_ is not None:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def compose_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.px4_sys_id  # <-- critical
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        self.compose_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
