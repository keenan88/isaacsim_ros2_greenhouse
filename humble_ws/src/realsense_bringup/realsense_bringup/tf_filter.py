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

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_callback,
            qos_profile)
        
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf_static_no_base_link',
            qos_profile)

        self.get_logger().info("TF Static Filter Node started, filtering out 'base_link' static transforms.")

    def tf_callback(self, msg: TFMessage):
        # Filter out transforms with 'base_link' as parent or child
        filtered_transforms = msg.transforms        

        # Add a static transform from arm_base_link to camera_link
        additional_transform = TransformStamped()
        additional_transform.header.stamp = self.get_clock().now().to_msg()
        additional_transform.header.frame_id = 'kinova/base_link'
        additional_transform.child_frame_id = 'camera_link'
        additional_transform.transform.translation.x = 0.51
        additional_transform.transform.translation.y = 0.06
        additional_transform.transform.translation.z = 1.8

        # -0.6087614, 0, 0.7933533, 0
        # -0.5735764, 0, 0.819152, 0
        additional_transform.transform.rotation.x = -0.5735764
        additional_transform.transform.rotation.y = 0.0
        additional_transform.transform.rotation.z = 0.819152
        additional_transform.transform.rotation.w = 0.0


        filtered_transforms.append(additional_transform)

        for i in range(len(filtered_transforms)):
            print(filtered_transforms[i].header.frame_id)
            if filtered_transforms[i].header.frame_id == 'camera_link':
                filtered_transforms[i].header.frame_id = 'realsense_camera_link'

            if filtered_transforms[i].child_frame_id == 'camera_link':
                filtered_transforms[i].child_frame_id = 'realsense_camera_link'
            
        
        if filtered_transforms:
            # print(filtered_transforms)
            self.publisher.publish(TFMessage(transforms=filtered_transforms))
            self.get_logger().debug(f"Published {len(filtered_transforms)} filtered static transforms.")

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
