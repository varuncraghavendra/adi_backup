import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleStatus, TrajectorySetpoint, VehicleLocalPosition


class HeartBeatNode(LifecycleNode):
    """
    Sends Offboard heartbeat and a 'hold' TrajectorySetpoint at ~10 Hz so PX4
    can accept OFFBOARD before takeoff_node takes over streaming setpoints.
    """
    def __init__(self, drone_ns: str = "drone_1", px4_ns: str = "px4_1"):
        super().__init__(f'{drone_ns}_hb_node')
        self.declare_parameter("drone_ns", drone_ns)
        self.declare_parameter("px4_ns", px4_ns)
        self.declare_parameter("publish_hold_setpoint", True)  # important for OFFBOARD engagement
        self.declare_parameter("hold_z", -1.0)  # NED down (meters)

        self.drone_ns = self.get_parameter("drone_ns").get_parameter_value().string_value
        self.px4_ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        self.publish_hold_setpoint = bool(self.get_parameter("publish_hold_setpoint").value)
        self.hold_z = float(self.get_parameter("hold_z").value)

        self.get_logger().info(f'[{self.drone_ns}] HeartBeatNode init for PX4 ns: {self.px4_ns}')
        self.offboard_control_mode_publisher = None
        self.hold_setpoint_pub = None
        self.timer_ = None
        self.vehicle_status = VehicleStatus()
        self.last_lpos = VehicleLocalPosition()

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("on_configure")

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'/{self.px4_ns}/fmu/in/offboard_control_mode', qos_best_effort
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_best_effort
        )

        if self.publish_hold_setpoint:
            self.hold_setpoint_pub = self.create_publisher(
                TrajectorySetpoint, f'/{self.px4_ns}/fmu/in/trajectory_setpoint', qos_best_effort
            )
            # track current local position so hold is near current pose
            self.lpos_sub = self.create_subscription(
                VehicleLocalPosition, f'/{self.px4_ns}/fmu/out/vehicle_local_position',
                self.vehicle_local_position_callback, qos_best_effort
            )

        self.timer_ = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.timer_.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info('on_cleanup')
        if self.offboard_control_mode_publisher is not None:
            self.destroy_publisher(self.offboard_control_mode_publisher)
        if self.hold_setpoint_pub is not None:
            self.destroy_publisher(self.hold_setpoint_pub)
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
        if self.hold_setpoint_pub is not None:
            self.destroy_publisher(self.hold_setpoint_pub)
        if self.timer_ is not None:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.last_lpos = msg

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_hold_setpoint(self):
        if self.hold_setpoint_pub is None:
            return
        msg = TrajectorySetpoint()
        # Hold near last known position, but ensure a finite Z
        x = float(self.last_lpos.x) if self.last_lpos and not (self.last_lpos.x != self.last_lpos.x) else 0.0
        y = float(self.last_lpos.y) if self.last_lpos and not (self.last_lpos.y != self.last_lpos.y) else 0.0
        z = self.hold_z
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.hold_setpoint_pub.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        if self.publish_hold_setpoint:
            self.publish_hold_setpoint()


def main(args=None):
    rclpy.init(args=args)
    node = HeartBeatNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
