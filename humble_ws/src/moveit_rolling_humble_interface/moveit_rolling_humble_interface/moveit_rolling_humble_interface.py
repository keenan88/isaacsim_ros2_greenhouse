import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateTranslator(Node):
    def __init__(self):
        super().__init__('joint_state_translator')
        self.rolling_joint_commands_subscription = self.create_subscription(
            JointState,
            'rolling_joint_commands',
            self.rolling_joint_commands_callback,
            10
        )

        self.humble_joint_commands_publisher = self.create_publisher(
            JointState,
            'humble_joint_commands',
            10
        )

        self.humble_joint_states_subscription = self.create_subscription(
            JointState,
            'humble_joint_states',
            self.humble_joint_states_callback,
            10
        )

        self.rolling_joint_states_publisher = self.create_publisher(
            JointState,
            'rolling_joint_states',
            10
        )

    def rolling_joint_commands_callback(self, msg):
        self.humble_joint_commands_publisher.publish(msg)

    def humble_joint_states_callback(self, msg):
        self.rolling_joint_states_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_state_translator = JointStateTranslator()
    rclpy.spin(joint_state_translator)
    joint_state_translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
