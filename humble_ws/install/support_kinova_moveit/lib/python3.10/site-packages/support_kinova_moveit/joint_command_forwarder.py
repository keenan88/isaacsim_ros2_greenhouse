import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
import asyncio
from sensor_msgs.msg import JointState
import numpy as np
from time import sleep


class JointTrajectoryActionServer(Node):

    def __init__(self):
        super().__init__('joint_trajectory_action_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group = ReentrantCallbackGroup(),
        )

        self._joint_command_publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.record_joint_state,
            10
        )
        self.joint_state = JointState()

        self.get_logger().info('Joint Trajectory Action Server has been started.')

    def record_joint_state(self, joint_state):
        self.joint_state = joint_state

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request...')
        feedback_msg = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()
        
        # Convert trajectory points to Float64MultiArray message
        joint_command_msg = Float64MultiArray()

        for point in goal_handle.request.trajectory.points:
            joint_command_msg.data = point.positions
            self.get_logger().info(f'Publishing point: {point}')
            joint_command = JointState()

            joint_command.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            joint_command.position = point.positions
            joint_command.velocity = point.velocities

            self._joint_command_publisher.publish(joint_command)
            
            feedback_msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            feedback_msg.actual.positions = self.joint_state.position
            feedback_msg.actual.velocities = self.joint_state.velocity

            feedback_msg.desired = point

            feedback_msg.error.positions = list(np.array(feedback_msg.desired.positions) - np.array(feedback_msg.actual.positions))
            feedback_msg.error.velocities = list(np.array(feedback_msg.desired.velocities) - np.array(feedback_msg.actual.velocities))

            goal_handle.publish_feedback(feedback_msg)

            sleep(0.1)
            
            # await self.sleep_in_repose()  # Simulate execution time
        
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        
        self.get_logger().info('Goal execution completed.')

        return result
        
    # async def sleep_in_repose(self):
    #     # Slumber, akin to a tranquil repose, to simulate time passage
    #     await asyncio.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
