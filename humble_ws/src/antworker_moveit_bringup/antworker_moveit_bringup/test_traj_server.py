import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import time

class JointTrajectoryClient(Node):

    def __init__(self):
        super().__init__('joint_trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info('Action client initialized')

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        # Define the trajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Create a single trajectory point
        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, 0.0, -0.5, -1.0, 0.0]  # Example positions
        point.time_from_start = rclpy.duration.Duration(seconds=5.0).to_msg()
        trajectory_msg.points.append(point)
        
        goal_msg.trajectory = trajectory_msg

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryClient()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
