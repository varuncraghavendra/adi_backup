import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode


class HeartBeatNode(LifecycleNode):
    """
    Streams OffboardControlMode at 10 Hz.
    PX4 requires this heartbeat while in OFFBOARD.
    """

    def __init__(self, drone_ns: str = "drone_1", px4_ns: str = "px4_1"):
        super().__init__(f'{drone_ns}_hb_node')
        self.declare_parameter("drone_ns", drone_ns)
        self.declare_parameter("px4_ns", px4_ns)
        self.drone_ns = self.get_parameter("drone_ns").get_parameter_value().string_value
        self.px4_ns = self.get_parameter("px4_ns").get_parameter_value().string_value

        self.get_logger().info(f'[{self.drone_ns}] HeartBeatNode init for PX4 ns: {self.px4_ns}')
        self.pub = None
        self.timer_ = None

    def on_configure(self, state: LifecycleState):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            OffboardControlMode, f'/{self.px4_ns}/fmu/in/offboard_control_mode', qos
        )
        self.timer_ = self.create_timer(0.1, self.timer_cb)  # 10 Hz
        self.timer_.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info(f'[{self.drone_ns}] HeartBeatNode activated')
        self.timer_.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info(f'[{self.drone_ns}] HeartBeatNode deactivated')
        self.timer_.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState):
        if self.pub:
            self.destroy_publisher(self.pub)
        if self.timer_:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        if self.pub:
            self.destroy_publisher(self.pub)
        if self.timer_:
            self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def timer_cb(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub.publish(msg)
