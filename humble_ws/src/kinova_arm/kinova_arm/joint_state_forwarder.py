import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateForwarder(Node):
    def __init__(self):
        super().__init__('joint_state_forwarder')
        self.subscription = self.create_subscription(
            JointState,
            '/isaac_joint_states',
            self.joint_state_callback,
            10
        )
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

    def joint_state_callback(self, msg):
        # Process the received joint state message here
        # and publish the processed joint state message
        robot_joint_states = JointState()

        robot_joint_states.header = msg.header
        robot_joint_states.name = msg.name[2:]
        robot_joint_states.position = msg.position[2:]
        robot_joint_states.velocity = msg.velocity[2:]
        robot_joint_states.effort = msg.effort[2:]

        self.publisher.publish(robot_joint_states)

def main(args=None):
    rclpy.init(args=args)
    joint_state_forwarder = JointStateForwarder()
    rclpy.spin(joint_state_forwarder)
    joint_state_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()