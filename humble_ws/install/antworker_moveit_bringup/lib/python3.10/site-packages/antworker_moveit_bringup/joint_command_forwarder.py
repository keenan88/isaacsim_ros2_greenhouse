import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import numpy as np

class JointForwarder(Node):
    def __init__(self):
        super().__init__('joint_command_forwarder')
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/kinova_arm_controller/controller_state',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/joint_commands', 10)

    def listener_callback(self, msg):
        joint_state_msg = JointState()
        joint_state_msg.header = msg.header
        joint_state_msg.name = msg.joint_names
        joint_state_msg.position = msg.output.positions
        
        # Optionally, you can set velocity and effort if needed
        # joint_state_msg.velocity = msg.actual.velocities
        # joint_state_msg.effort = msg.actual.effort

        # kinova_arm_controller/controller_state continually publishes, and if given no trajectory
        if not np.sum(np.abs(np.array(joint_state_msg.position))) == 0:
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
