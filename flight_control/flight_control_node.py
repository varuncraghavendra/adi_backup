import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from drone_msgs.action import ArmDrone, TakeOff, Land, FlyTo
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from functools import partial


class FlightControlNode(LifecycleNode):
    """
    Exposes action servers under /{drone_ns}/arm|takeoff|land|fly_to
    Controls lifecycle nodes under /{drone_ns}/{arming|takeoff|landing|hb}_node
    Sends setpoints on /{drone_ns}/flight_control/update_pos
    """
    def __init__(self, drone_ns: str = "drone_1"):
        super().__init__(f'{drone_ns}_flight_control_node')
        self.declare_parameter("drone_ns", drone_ns)
        self.drone_ns = self.get_parameter("drone_ns").get_parameter_value().string_value

        self.get_logger().info(f"[{self.drone_ns}] FlightControlNode created.")

        self.arm_server = None
        self.takeoff_server = None
        self.land_server = None
        self.flyto_server = None

        self.using_autopilot = True
        self.future: Future = None
        self.future1: Future = None
        self.future2: Future = None

    def on_configure(self, state: State):
        self.get_logger().info("Configuring...")

        if self.using_autopilot:
            # lifecycle clients (namespaced)
            self.arm_cli = self.create_client(ChangeState, f'/{self.drone_ns}/arming_node/change_state')
            self.arm_cli.wait_for_service()

            self.to_cli = self.create_client(ChangeState, f'/{self.drone_ns}/takeoff_node/change_state')
            self.to_cli.wait_for_service()

            self.land_cli = self.create_client(ChangeState, f'/{self.drone_ns}/landing_node/change_state')
            self.land_cli.wait_for_service()

            self.hb_cli = self.create_client(ChangeState, f'/{self.drone_ns}/hb_node/change_state')
            self.hb_cli.wait_for_service()

            # Configure the nodes
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            self.arm_cli.call_async(req).add_done_callback(partial(self._log_change_state, 'arming_node'))
            self.to_cli.call_async(req).add_done_callback(partial(self._log_change_state, 'takeoff_node'))
            self.land_cli.call_async(req).add_done_callback(partial(self._log_change_state, 'landing_node'))
            self.hb_cli.call_async(req).add_done_callback(partial(self._log_change_state, 'hb_node'))

        self._setpoint_pub = self.create_publisher(Float32MultiArray, f'/{self.drone_ns}/flight_control/update_pos', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("Activating...")

        self.arm_server = ActionServer(self, ArmDrone, f'/{self.drone_ns}/arm', self.execute_arm)
        self.takeoff_server = ActionServer(self, TakeOff, f'/{self.drone_ns}/takeoff', self.execute_takeoff)
        self.land_server = ActionServer(self, Land, f'/{self.drone_ns}/land', self.execute_land)
        self.flyto_server = ActionServer(self, FlyTo, f'/{self.drone_ns}/fly_to', self.execute_flyto)

        if self.using_autopilot:
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            self.hb_cli.call_async(req).add_done_callback(partial(self._log_change_state, 'hb_node'))

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info("Cleaning up...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info("Deactivating...")
        # Destroy action servers safely
        if self.arm_server: self.arm_server.destroy()
        if self.takeoff_server: self.takeoff_server.destroy()
        if self.land_server: self.land_server.destroy()
        if self.flyto_server: self.flyto_server.destroy()
        return TransitionCallbackReturn.SUCCESS

    # === helpers

    def _log_change_state(self, node_name: str, future):
        try:
            response = future.result()
            if response:
                self.get_logger().info(f'[{self.drone_ns}] response from {node_name}: {response.success}')
        except Exception as e:
            self.get_logger().error(f'[{self.drone_ns}] Service Call for {node_name} failed: {e}')

    def send_trajectory_setpoint(self, x, y, z):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self._setpoint_pub.publish(msg)

    # === Action callbacks

    def execute_arm(self, goal_handle):
        self.get_logger().info(f"[{self.drone_ns}] Arming drone...")
        result = ArmDrone.Result()

        if self.using_autopilot:
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE

            if self.future is not None and not self.future.done():
                self.future.cancel()

            self.future = self.arm_cli.call_async(req)
            self.future.add_done_callback(partial(self._log_change_state, 'arming_node'))

        goal_handle.succeed()
        result.arming_status = True
        return result

    def execute_takeoff(self, goal_handle):
        altitude = goal_handle.request.altitude
        self.get_logger().info(f"[{self.drone_ns}] Taking off to {altitude.data:.2f}m")

        if self.using_autopilot:
            req_activate_to = ChangeState.Request()
            req_activate_to.transition.id = Transition.TRANSITION_ACTIVATE
            if self.future1 is not None and not self.future1.done():
                self.future1.cancel()
            self.future1 = self.to_cli.call_async(req_activate_to)
            self.future1.add_done_callback(partial(self._log_change_state, 'takeoff_node'))

            # Deactivate arming node to avoid timer spam (matches your logic)
            req_deactivate_arm = ChangeState.Request()
            req_deactivate_arm.transition.id = Transition.TRANSITION_DEACTIVATE
            if self.future2 is not None and not self.future2.done():
                self.future2.cancel()
            self.future2 = self.arm_cli.call_async(req_deactivate_arm)
            self.future2.add_done_callback(partial(self._log_change_state, 'arming_node'))

        goal_handle.succeed()
        return TakeOff.Result(take_off_completed=True)

    def execute_land(self, goal_handle):
        self.get_logger().info(f"[{self.drone_ns}] Landing...")
        if self.using_autopilot:
            req_deactivate_to = ChangeState.Request()
            req_deactivate_to.transition.id = Transition.TRANSITION_DEACTIVATE
            self.to_cli.call_async(req_deactivate_to).add_done_callback(partial(self._log_change_state, 'takeoff_node'))

            req_activate_land = ChangeState.Request()
            req_activate_land.transition.id = Transition.TRANSITION_ACTIVATE
            self.land_cli.call_async(req_activate_land).add_done_callback(partial(self._log_change_state, 'landing_node'))

        goal_handle.succeed()
        return Land.Result(landing_status=True)

    def execute_flyto(self, goal_handle):
        location = goal_handle.request.target_pose
        self.get_logger().info(
            f"[{self.drone_ns}] FlyTo {location.pose.position.x:.2f}, "
            f"{location.pose.position.y:.2f}, {location.pose.position.z:.2f}"
        )

        if self.using_autopilot:
            self.send_trajectory_setpoint(
                location.pose.position.x,
                location.pose.position.y,
                location.pose.position.z
            )

        goal_handle.succeed()
        res = FlyTo.Result(success=True, message="Commanded position setpoint.")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = FlightControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
