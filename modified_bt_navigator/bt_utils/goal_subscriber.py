import py_trees
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalSubscriber:
    def __init__(self, node: Node, drone_ns: str = "drone_1"):
        self.node = node
        self.drone_ns = drone_ns
        self.bb = py_trees.blackboard.Blackboard()

        # Ensure namespaced keys exist
        self.key_new_goal = f"{self.drone_ns}/new_goal_received"
        self.key_goal = f"{self.drone_ns}/goal_pose"
        self.key_retry = f"{self.drone_ns}/retry_counter"

        if not self.bb.exists(self.key_new_goal):
            self.bb.set(self.key_new_goal, False)
        if not self.bb.exists(self.key_retry):
            self.bb.set(self.key_retry, 0)

        self._sub = self.node.create_subscription(
            PoseStamped,
            f'/{self.drone_ns}/next_goal',
            self._goal_callback,
            10
        )
        self.node.get_logger().info(f"[{self.drone_ns}] GoalSubscriber listening to /{self.drone_ns}/next_goal")

    def _goal_callback(self, msg: PoseStamped):
        self.node.get_logger().info(f"[{self.drone_ns}] Received new goal")
        self.bb.set(self.key_new_goal, True)
        self.bb.set(self.key_goal, msg)
