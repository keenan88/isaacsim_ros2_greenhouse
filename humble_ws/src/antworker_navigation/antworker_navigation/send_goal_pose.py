import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy



class GoalPoseServer(Node):
    def __init__(self):
        super().__init__('goal_pose_server')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.SYSTEM_DEFAULT,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        pose_stamped = PoseStamped()

        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.y = -2.5 # Assumes robot starts at (0,0,0), (0,0,0)

        pose_stamped.pose.orientation.z = -0.73
        pose_stamped.pose.orientation.w = 0.67

        self.publisher.publish(pose_stamped)

        print("Pose published")




def main(args=None):
    rclpy.init(args=args)
    move_group_node = GoalPoseServer()

    rclpy.spin(move_group_node)

    

if __name__ == '__main__':
    main()