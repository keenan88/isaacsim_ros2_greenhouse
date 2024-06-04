import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import numpy as np

class JointForwarder(Node):
    def __init__(self):
        super().__init__('joint_state_forwarder')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states_isaacsim',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def listener_callback(self, msg):
        joint_state_msg = JointState()
        joint_state_msg.header = msg.header

        arm_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        for name, pos in zip(msg.name, msg.position):
            if name in arm_joints:
                joint_state_msg.name.append(name)
                joint_state_msg.position.append(pos)

        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
