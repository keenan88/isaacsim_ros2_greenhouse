import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.subscription_ = self.create_subscription(String, 'my_topic', self.callback, 10)
        self.subscription_.callback = self.callback

    def callback(self, msg):
        self.get_logger().info('Received: %s' % msg.data)

    def publish_message(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    msg = String()
    msg.data = 'Hello, world!'
    node.publish_message(msg)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()