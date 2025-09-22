import py_trees
from rclpy.action import ActionClient
from drone_msgs.action import ArmDrone as Arm


class ArmDrone(py_trees.behaviour.Behaviour):
    def __init__(self, node, drone_ns: str = "drone_1"):
        super().__init__(name=f"ArmDrone_{drone_ns}")
        self.node = node
        self.drone_ns = drone_ns
        self.client = ActionClient(node, Arm, f'/{self.drone_ns}/arm')
        self._goal_future = None
        self._result_future = None
        self._goal_sent = False
        self._done = False

    def initialise(self):
        if not self._done:
            self._goal_future = None
            self._result_future = None
            self._goal_sent = False

    def update(self):
        if not self.client.wait_for_server(timeout_sec=1.0):
            return py_trees.common.Status.FAILURE

        if not self._goal_sent:
            goal_msg = Arm.Goal()
            self._goal_future = self.client.send_goal_async(goal_msg)
            self._goal_future.add_done_callback(self.goal_response_callback)
            self._goal_sent = True
            return py_trees.common.Status.RUNNING

        if self._result_future is not None and self._result_future.done():
            result = self._result_future.result().result
            self._done = True
            return py_trees.common.Status.SUCCESS if result.arming_status else py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_sent = False
            self._done = True
            return
        self._result_future = goal_handle.get_result_async()
