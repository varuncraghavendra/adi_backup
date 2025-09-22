import py_trees
from rclpy.action import ActionClient
from drone_msgs.action import FlyTo
from geometry_msgs.msg import PoseStamped


class FlyToPosition(py_trees.behaviour.Behaviour):
    def __init__(self, node, drone_ns: str = "drone_1"):
        super().__init__(name=f"FlyToDrone_{drone_ns}")
        self.node = node
        self.drone_ns = drone_ns
        self.client = ActionClient(node, FlyTo, f'/{self.drone_ns}/fly_to')

        # Namespaced blackboard keys so two drones don't clash in a single process
        self.bb = self.attach_blackboard_client(name=f"FlyToDrone_{drone_ns}")
        self.key_goal = f"{self.drone_ns}/goal_pose"
        self.key_new_goal = f"{self.drone_ns}/new_goal_received"
        self.bb.register_key(key=self.key_goal, access=py_trees.common.Access.READ)
        self.bb.register_key(key=self.key_new_goal, access=py_trees.common.Access.WRITE)

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

        goal_pose: PoseStamped = getattr(self.bb, self.key_goal, None)
        if goal_pose is None:
            # no goal yet
            return py_trees.common.Status.RUNNING

        if not self._goal_sent:
            goal_msg = FlyTo.Goal()
            goal_msg.target_pose = goal_pose
            self._goal_future = self.client.send_goal_async(goal_msg)
            self._goal_future.add_done_callback(self.goal_response_callback)
            self._goal_sent = True
            return py_trees.common.Status.RUNNING

        if self._result_future is not None and self._result_future.done():
            result = self._result_future.result().result
            setattr(self.bb, self.key_new_goal, False)
            self._done = True
            return py_trees.common.Status.SUCCESS if result.success else py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.status in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE]:
            self._done = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_sent = False
            self._done = True
            return
        self._result_future = goal_handle.get_result_async()
