import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus, OffboardControlMode


class ArmingNode(LifecycleNode):
    """
    Waits until OffboardControlMode has been streaming for >=1s, then:
      1) sets OFFBOARD (DO_SET_MODE, main=1 custom, sub=6),
      2) sends ARM.
    """
    def __init__(self, drone_ns: str = "drone_1", px4_ns: str = "px4_1"):
        super().__init__(f'{drone_ns}_arming_node')
        self.declare_parameter("drone_ns", drone_ns)
        self.declare_parameter("px4_ns", px4_ns)
        self.drone_ns = self.get_parameter("drone_ns").get_parameter_value().string_value
        self.px4_ns = self.get_parameter("px4_ns").get_parameter_value().string_value

        self.get_logger().info(f'[{self.drone_ns}] ArmingNode init for PX4 ns: {self.px4_ns}')
        self.vehicle_command_publisher = None
        self.vehicle_status = VehicleStatus()
        self.timer_ = None
        self._armed_sent = False
        self._offboard_seen = False
        self._offboard_started_ns = None

    def on_configure(self, state: LifecycleState):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vehicle_command_publisher = self.create_lifecycle_publisher(
            VehicleCommand, f'/{self.px4_ns}/fmu/in/vehicle_command', qos
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status', self.vehicle_status_cb, qos
        )
        # Observe OffboardControlMode to know hb stream is present
        self.offboard_sub = self.create_subscription(
            OffboardControlMode, f'/{self.px4_ns}/fmu/in/offboard_control_mode', self.offboard_cb, qos
        )
        self.timer_ = self.create_timer(0.1, self.timer_cb)  # 10 Hz state machine
        self.timer_.cancel()
        self._armed_sent = False
        self._offboard_seen = False
        self._offboard_started_ns = None
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info(f'[{self.drone_ns}] ArmingNode activated')
        self._armed_sent = False
        self._offboard_seen = False
        self._offboard_started_ns = None
        self.timer_.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info(f'[{self.drone_ns}] ArmingNode deactivated')
        self.timer_.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState):
        if self.vehicle_command_publisher:
            self.destroy_lifecycle_publisher(self.vehicle_command_publisher)
        if self.timer_:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        if self.vehicle_command_publisher:
            self.destroy_lifecycle_publisher(self.vehicle_command_publisher)
        if self.timer_:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    # -- Callbacks ------------------------------------------------------------

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def offboard_cb(self, _msg: OffboardControlMode):
        # mark stream as present; start a "warm-up" stopwatch at first sight
        if not self._offboard_seen:
            self._offboard_seen = True
            self._offboard_started_ns = self.get_clock().now().nanoseconds

    # -- Helpers --------------------------------------------------------------

    def _send_cmd(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # -- State machine --------------------------------------------------------

    def timer_cb(self):
        # wait until OffboardControlMode stream has been seen for >= 1.0 s
        if not self._offboard_seen or self._offboard_started_ns is None:
            return
        elapsed = (self.get_clock().now().nanoseconds - self._offboard_started_ns) / 1e9
        if elapsed < 1.0:
            return

        # 1) set mode: main=1(custom), sub=6(OFFBOARD)
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            return

        # 2) arm once OFFBOARD is reported
        if not self._armed_sent:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info(f'[{self.drone_ns}] Arm command sent (OFFBOARD confirmed after warm-up)')
            self._armed_sent = True
            self.timer_.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ArmingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
