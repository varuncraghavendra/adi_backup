import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from goal_manager_msgs.srv import GetNextGoal


class GoalManagerNode(Node):
    """
    Publishes goals to BOTH /drone_1/next_goal and /drone_2/next_goal.
    If you want different logic per drone, split or parametrize later.
    """
    def __init__(self):
        super().__init__('goal_manager_node')
        self.goal_pub_1 = self.create_publisher(PoseStamped, '/drone_1/next_goal', 10)
        self.goal_pub_2 = self.create_publisher(PoseStamped, '/drone_2/next_goal', 10)

        self.cli = self.create_client(GetNextGoal, '/get_next_goal')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_next_goal service not available, retrying...')

        self.timer = self.create_timer(2.0, self.request_goal)
        self.latest_goal = None

    def request_goal(self):
        req = GetNextGoal.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            resp = future.result()
            if resp and isinstance(resp.goal_pose, PoseStamped):
                self.latest_goal = resp.goal_pose
                self.goal_pub_1.publish(resp.goal_pose)
                self.goal_pub_2.publish(resp.goal_pose)
                self.get_logger().info(
                    f"Published goal to both drones -> x={resp.goal_pose.pose.position.x:.2f}, "
                    f"y={resp.goal_pose.pose.position.y:.2f}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
