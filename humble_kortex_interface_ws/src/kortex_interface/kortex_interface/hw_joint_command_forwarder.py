# trajectory_relay_node.py
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryRelayNode(Node):
    def __init__(self):
        super().__init__('trajectory_relay_node')

        # Create subscriber to the arm controller state
        self.state_subscriber = self.create_subscription(
            JointTrajectoryControllerState,
            'kinova_arm_controller/state',
            self.state_callback,
            10
        )

        # Create publisher to the joint trajectory controller
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            10
        )

    def state_callback(self, ctrl_state):

        #if not ctrl_state.header.frame_id == '':

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ctrl_state.joint_names
        trajectory_msg.header = ctrl_state.header
        
        next_point = JointTrajectoryPoint()
        next_point.positions = ctrl_state.desired.positions
        next_point.velocities = ctrl_state.desired.velocities
        next_point.accelerations = ctrl_state.desired.accelerations
        next_point.time_from_start.sec = 1 # = ctrl_state.desired.time_from_start


        trajectory_msg.points.append(next_point)

        self.trajectory_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
