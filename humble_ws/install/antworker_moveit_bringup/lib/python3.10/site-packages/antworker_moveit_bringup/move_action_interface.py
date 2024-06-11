import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

class MoveGroupActionClientNode:
    def __init__(self):
        self.node = rclpy.create_node('move_action_action_client')

        self.move_group_action_client = ActionClient(self.node, MoveGroup, 'move_action')

        while not self.move_group_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Action server not available, waiting again...')

    def send_goal(self):
        goal_msg = MoveGroup.Goal()

        goal_msg.group_name = "kinova_arm"
        goal_msg.num_planning_attempts = 100
        goal_msg.allowed_planning_time = 1.0 # seconds

        goal_msg.request.goal_constraints.position_constraints.header.frame_id = "arm_base_link"
        goal_msg.request.goal_constraints.position_constraints.link_name = "end_effector_link"
        goal_msg.request.goal_constraints.position_constraints.target_point_offset.x = 0.2
        goal_msg.request.goal_constraints.position_constraints.target_point_offset.y = 0.01
        goal_msg.request.goal_constraints.position_constraints.target_point_offset.z = 0.9

        goal_msg.request.goal_constraints.orientation_constraints.header.frame_id = "arm_base_link"
        goal_msg.request.goal_constraints.orientation_constraints.link_name = "end_effector_link"
        goal_msg.request.goal_constraints.orientation_constraints.orientation.x = 0.0
        goal_msg.request.goal_constraints.orientation_constraints.orientation.y = 0.0
        goal_msg.request.goal_constraints.orientation_constraints.orientation.z = 1.0
        goal_msg.request.goal_constraints.orientation_constraints.orientation.w = 0.0

        self.future = self.move_group_action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback)

        rclpy.spin_until_future_complete(self.node, self.future)

        if self.future.result() is not None:
            self.node.get_logger().info('Goal succeeded!')
        else:
            self.node.get_logger().info('Goal failed!')

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info('Received feedback: {0}'.format(feedback_msg))

    def destroy_node(self):
        self.move_group_action_client.destroy()
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    move_group_node = MoveGroupActionClientNode()

    # Send a goal to the action server
    move_group_node.send_goal()

    # Cleanup
    move_group_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
