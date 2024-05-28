import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.duration import Duration
from time import sleep

class KinovaArmActionServer(Node):
    def __init__(self):
        super().__init__('kinova_arm_action_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'kinova_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self._publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self._timer = self.create_timer(1.0, self.timer_callback)
        self._current_joint_states = JointState()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        # Extract the trajectory from the goal
        trajectory = goal_handle.request.trajectory

        for point in trajectory.points:
            # Publish joint commands
            # joint_command_msg = JointTrajectory()
            # joint_command_msg.header.stamp = self.get_clock().now().to_msg()
            # joint_command_msg.joint_names = trajectory.joint_names
            # joint_command_msg.points = point.positions

            self._publisher.publish(trajectory)

            # Feedback can be provided here
            #feedback_msg.joint_names = trajectory.joint_names
            #feedback_msg.actual.positions = self._current_joint_states.position
            #goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def timer_callback(self):
        # This can be used to update current joint states if necessary
        pass

def main(args=None):
    rclpy.init(args=args)
    action_server = KinovaArmActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
