# static_joint_state_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class StaticJointStatePublisher(Node):
    def __init__(self):
        super().__init__('static_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states_without_robotiq_gripper', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['front_wheel_joint', 'rear_wheel_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher_.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
