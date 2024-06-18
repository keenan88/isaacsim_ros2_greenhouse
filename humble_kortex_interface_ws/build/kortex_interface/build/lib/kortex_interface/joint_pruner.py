import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FilterJointStateNode(Node):

    def __init__(self):
        super().__init__('filter_joint_state_node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states_without_robotiq_gripper',
            10
        )

    def joint_state_callback(self, msg):
        filtered_msg = JointState()
        filtered_msg.header = msg.header

        kinova_arm_joints = ['joint_' + str(i) for i in range(1,7)]

        for i, name in enumerate(msg.name):
            if name in kinova_arm_joints:
                filtered_msg.name.append(name)
                filtered_msg.position.append(msg.position[i])
                filtered_msg.velocity.append(msg.velocity[i])
                filtered_msg.effort.append(msg.effort[i])

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilterJointStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
