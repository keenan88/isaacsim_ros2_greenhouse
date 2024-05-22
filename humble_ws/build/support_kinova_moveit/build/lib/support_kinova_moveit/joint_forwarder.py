import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Create a subscriber to the 'robot_joint_states' topic
        self.subscription = self.create_subscription(
            JointState,
            'robot_joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        
        # Create a publisher to the 'joint_states' topic
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        self.get_logger().info('Node has been started and is listening to "robot_joint_states".')

    def listener_callback(self, msg):
        self.get_logger().info('Received message from "robot_joint_states"')
        
        # Here, we simply forward the received message to the 'joint_states' topic
        arm_joints_only = JointState()
        arm_joints_only.name = msg.name[2:]
        arm_joints_only.velocity = msg.velocity[2:]
        arm_joints_only.effort = msg.effort[2:]
        arm_joints_only.position = msg.position[2:]

        arm_joints_only.header = msg.header

        self.publisher.publish(arm_joints_only)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
