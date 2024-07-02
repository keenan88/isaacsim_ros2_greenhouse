#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Pose

class InteractiveMarkerPublisher(Node):
    def __init__(self):
        super().__init__('interactive_marker_publisher')

        # Create an interactive marker server
        self.server = InteractiveMarkerServer(self, "interactive_marker_server")

        # Create and initialize the interactive marker
        self.create_interactive_marker()

    def create_interactive_marker(self):
        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "simple_marker"
        int_marker.description = "Simple 3D Control"
        int_marker.pose = Pose()
        int_marker.pose.position.z = 1.0

        # Create a visual marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Create an interactive control
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.markers.append(marker)
        int_marker.controls.append(control)

        # Insert the marker into the server and set the callback function
        self.server.insert(int_marker)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        # Callback to handle feedback from the interactive marker
        position = feedback.pose.position
        self.get_logger().info(f"Marker {feedback.marker_name} is now at {position.x}, {position.y}, {position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
