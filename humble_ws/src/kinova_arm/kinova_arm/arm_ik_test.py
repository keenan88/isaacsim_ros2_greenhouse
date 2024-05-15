import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        self.client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = GetPositionIK.Request()
        print(self.request)

        # Call the service
        self.call_service()

    def call_service(self):
#        self.request.$PLACEHOLDER$  # Fill in the request fields
        self.request.ik_request.group_name = "kinova_arm"
        self.request.ik_request.pose_stamped.header.frame_id = "arm_base_link"

        self.request.ik_request.robot_state.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.request.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.request.ik_request.pose_stamped.pose.position.x = 0.5
        self.request.ik_request.pose_stamped.pose.position.y = 0.5
        self.request.ik_request.pose_stamped.pose.position.z = 0.0

        self.request.ik_request.pose_stamped.pose.orientation.x = 0.5
        self.request.ik_request.pose_stamped.pose.orientation.y = 0.5
        self.request.ik_request.pose_stamped.pose.orientation.z = 0.0
        self.request.ik_request.pose_stamped.pose.orientation.w = 1.0

        self.request.ik_request.ik_link_names = [""]
        future = self.client.call_async(self.request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            print("response: ", response)
            # Process the response here
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()