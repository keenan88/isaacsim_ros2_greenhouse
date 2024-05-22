import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion

class MotionPlanningNode(Node):

    def __init__(self):
        super().__init__('motion_planning_node')
        
        # Create a client for the /plan_kinematic_path service
        self.cli = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /plan_kinematic_path not available, waiting...')
        
        # Create a request to the service
        self.request = GetMotionPlan.Request()
        
        # Fill in the request details
        self.populate_request()

    def populate_request(self):
        # Define the target pose for the end effector
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        # Create position and orientation constraints
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "end_effector_link"
        position_constraint.target_point_offset = Point()
        position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "end_effector_link"
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        # Add constraints to the motion plan request
        self.request.motion_plan_request.group_name = "arm"
        self.request.motion_plan_request.goal_constraints.append(Constraints(
            position_constraints=[position_constraint],
            orientation_constraints=[orientation_constraint]
        ))

        # Optionally, set start state (if different from current state)
        self.request.motion_plan_request.start_state.is_diff = True

    def send_request(self):
        self.future = self.cli.call_async(self.request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.motion_plan_response.error_code.val == response.motion_plan_response.error_code.SUCCESS:
                self.get_logger().info('Motion plan computed successfully.')
                # You can further process the response.trajectory here if needed.
            else:
                self.get_logger().error('Failed to compute motion plan.')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    motion_planning_node = MotionPlanningNode()
    motion_planning_node.send_request()
    rclpy.spin(motion_planning_node)

    motion_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
