import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.duration import Duration
from time import sleep
import numpy as np

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time


class KinovaArmActionServer(Node):
    def __init__(self):
        super().__init__('kinova_arm_action_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'kinova_arm_controller/follow_joint_trajectory',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback
        )
        self._publisher = self.create_publisher(JointState, '/joint_commands', 10)
        self.joint_positions = []

        self.create_subscription(JointState, 'joint_states', self.update_joint_states, 10)

        


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

        joints_idx = 0

        max_joint_angle_error_rads = 10 / 180 * 3.14

        joint_command = JointState()
        joint_command.name = trajectory.joint_names

        while joints_idx < len(trajectory.points):

            point = trajectory.points[joints_idx]

            joint_command.position = point.positions
   
            joint_angle_diffs = np.abs(np.array(self.joint_positions) - np.array(joint_command.position))

            if joints_idx == 0 or (len(self.joint_positions) == 6 and max(joint_angle_diffs) < max_joint_angle_error_rads):  

                joints_idx += 1

                joint_command.position = trajectory.points[joints_idx].positions         

                self._publisher.publish(joint_command)

                self.get_logger().info(f'Joint Positions: {self.joint_positions}')
                self.get_logger().info(f'Joint Command Positions: {joint_command.position}')

            time.sleep(0.1)
            
                

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def update_joint_states(self, msg):
        self.get_logger().info(f'Incoming Joint Positions: {msg.position}')
        self.joint_positions = msg.position[2:]

    




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
