# File: tf_filter_node/tf_filter_static_node.py

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped


class TFStaticFilterNode(Node):
    def __init__(self):
        super().__init__('tf_static_filter_node')

        # QoS profile for tf_static
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.publish_tf_timer = self.create_timer(0.1, self.tf_callback)
        
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf_static',
            qos_profile)

        self.get_logger().info("TF Static Filter Node started, filtering out 'base_link' static transforms.")

    def tf_callback(self):

        additional_transform = TransformStamped()
        additional_transform.header.stamp = self.get_clock().now().to_msg()
        additional_transform.header.frame_id = 'kinova/base_link'
        additional_transform.child_frame_id = 'camera_link'
        additional_transform.transform.translation.x = 0.51
        additional_transform.transform.translation.y = -0.05
        additional_transform.transform.translation.z = 1.8

        additional_transform.transform.rotation.x = -0.5735764
        additional_transform.transform.rotation.y = 0.0
        additional_transform.transform.rotation.z = 0.819152
        additional_transform.transform.rotation.w = 0.0

        TF_msg = TFMessage()
        TF_msg.transforms.append(additional_transform)

        self.publisher.publish(TF_msg)
        self.get_logger().debug(f"Published {1} filtered static transforms.")

def main(args=None):
    rclpy.init(args=args)
    node = TFStaticFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
