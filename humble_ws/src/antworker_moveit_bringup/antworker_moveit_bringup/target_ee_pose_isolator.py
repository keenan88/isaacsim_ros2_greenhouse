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


class KinovaArmActionServer(Node):
    def __init__(self):
        super().__init__('target_ee_pose_isolator')
        self._publisher = self.create_publisher(JointState, '/joint_commands', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_pose = TFMessage()

        self.tf_target_buffer = Buffer()
        self.tf_target_listener = TransformListener(self.tf_target_buffer, self)
        self.ee_target_pose = TFMessage()

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        from_frame = 'arm_base_link'
        to_frame = 'end_effector_link'
        
        try:
            now = rclpy.time.Time()
            self.ee_pose = self.tf_buffer.lookup_transform(from_frame, to_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
            self.ee_target_pose = self.tf_target_buffer.lookup_transform(from_frame, to_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
            self.get_logger().info(f'Transform from {from_frame} to {to_frame}: {self.ee_target_pose}')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform {from_frame} to {to_frame}: {str(e)}')

    




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
