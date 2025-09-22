import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from goal_manager_msgs.srv import GetNextGoal

class GoalManagerNode(Node):
    """
    Pulls a goal from /get_next_goal and publishes the SAME PoseStamped
    to both /drone_1/next_goal and /drone_2/next_goal.
    """
    def __init__(self):
        super().__init__('goal_manager_node')

        # Publishers for both drones
        self.goal_pub_1 = self.create_publisher(PoseStamped, '/drone_1/next_goal', 10)
        self.goal_pub_2 = self.create_publisher(PoseStamped, '/drone_2/next_goal', 10)

        # Single shared service (no change needed on the server)
        self.cli = self.create_client(GetNextGoal, '/get_next_goal')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_next_goal service not available, retrying...')

        # Poll service periodically
        self.timer = self.create_timer(2.0, self.request_goal)

    def request_goal(self):
        req = GetNextGoal.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            resp = future.result()
            if not isinstance(resp, GetNextGoal.Response):
                self.get_logger().error("Invalid response type from /get_next_goal")
                return
            pose: PoseStamped = resp.goal_pose
            # Publish SAME pose to both drones
            self.goal_pub_1.publish(pose)
            self.goal_pub_2.publish(pose)
            self.get_logger().info(
                f"Published shared goal -> x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, z={pose.pose.position.z:.2f}"
            )
        except Exception as e:
            self.get_logger().error(f"/get_next_goal call failed: {e}")

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
