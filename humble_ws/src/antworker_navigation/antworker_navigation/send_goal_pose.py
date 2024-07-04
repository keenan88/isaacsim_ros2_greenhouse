import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from time import sleep


class GoalPoseServer(Node):
    def __init__(self):
        super().__init__('goal_pose_server')

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        sleep(2) # x second timeout is necessary for message to be properly published to goal_pose
        # without timeout, goal_pose message is published but bt_navigator does not accept

        self.pose_stamped = PoseStamped()

        self.pose_stamped.header.frame_id = 'map'
        self.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.pose_stamped.pose.position.x = 50.0 # Assumes robot starts at (0,0,0), (0,0,0)

        self.pose_stamped.pose.orientation.w = 1.0

        self.publisher.publish(self.pose_stamped)
        print("Pose published")


def main(args=None):
    rclpy.init(args=args)
    move_group_node = GoalPoseServer()

    rclpy.spin(move_group_node)

    

if __name__ == '__main__':
    main()