#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math


class SoundSourceVisualizer(Node):
    """
    Publishes visualization markers for the sound source in RViz
    """
    
    def __init__(self):
        super().__init__('sound_source_visualizer')
        
        # Parameters
        self.declare_parameter('sound_x', 3.0)
        self.declare_parameter('sound_y', 3.0)
        self.declare_parameter('sound_z', 0.5)
        
        # Publisher for sound source marker
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/sound_source_marker',
            10
        )
        
        # Timer to publish markers at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_marker)
        
        self.get_logger().info('Sound Source Visualizer Started')
        sound_x = self.get_parameter('sound_x').value
        sound_y = self.get_parameter('sound_y').value
        sound_z = self.get_parameter('sound_z').value
        self.get_logger().info(f'Sound source marker at: ({sound_x:.1f}, {sound_y:.1f}, {sound_z:.1f})')
    
    def publish_marker(self):
        """Publish sound source visualization marker"""
        
        sound_x = float(self.get_parameter('sound_x').value)
        sound_y = float(self.get_parameter('sound_y').value)
        sound_z = float(self.get_parameter('sound_z').value)
        
        marker_array = MarkerArray()
        
        # Marker 1: Pulsing sphere at sound source
        sphere_marker = Marker()
        sphere_marker.header.frame_id = 'odom'
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = 'sound_source'
        sphere_marker.id = 0
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        
        sphere_marker.pose.position.x = sound_x
        sphere_marker.pose.position.y = sound_y
        sphere_marker.pose.position.z = sound_z
        sphere_marker.pose.orientation.w = 1.0
        
        # Pulsing effect
        pulse = abs(math.sin(self.get_clock().now().nanoseconds / 1e9))
        scale = 0.3 + 0.2 * pulse
        sphere_marker.scale.x = scale
        sphere_marker.scale.y = scale
        sphere_marker.scale.z = scale
        
        # Bright yellow/gold color
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 0.8 + 0.2 * pulse
        
        marker_array.markers.append(sphere_marker)
        
        # Marker 2: Text label
        text_marker = Marker()
        text_marker.header = sphere_marker.header
        text_marker.ns = 'sound_label'
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = sound_x
        text_marker.pose.position.y = sound_y
        text_marker.pose.position.z = sound_z + 0.5
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        
        text_marker.text = f"🚨 SOUND SOURCE\n({sound_x:.1f}, {sound_y:.1f})"
        
        marker_array.markers.append(text_marker)
        
        # Marker 3: Cylinder beam from ground to source
        beam_marker = Marker()
        beam_marker.header = sphere_marker.header
        beam_marker.ns = 'sound_beam'
        beam_marker.id = 2
        beam_marker.type = Marker.CYLINDER
        beam_marker.action = Marker.ADD
        
        beam_marker.pose.position.x = sound_x
        beam_marker.pose.position.y = sound_y
        beam_marker.pose.position.z = sound_z / 2
        beam_marker.pose.orientation.w = 1.0
        
        beam_marker.scale.x = 0.1
        beam_marker.scale.y = 0.1
        beam_marker.scale.z = sound_z
        
        beam_marker.color.r = 1.0
        beam_marker.color.g = 1.0
        beam_marker.color.b = 0.0
        beam_marker.color.a = 0.3
        
        marker_array.markers.append(beam_marker)
        
        # Marker 4: Arrow pointing to target from above
        arrow_marker = Marker()
        arrow_marker.header = sphere_marker.header
        arrow_marker.ns = 'target_arrow'
        arrow_marker.id = 3
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        # Arrow points downward
        start_point = Point()
        start_point.x = sound_x
        start_point.y = sound_y
        start_point.z = sound_z + 1.0
        
        end_point = Point()
        end_point.x = sound_x
        end_point.y = sound_y
        end_point.z = sound_z + 0.2
        
        arrow_marker.points = [start_point, end_point]
        
        arrow_marker.scale.x = 0.1  # Shaft diameter
        arrow_marker.scale.y = 0.2  # Head diameter
        arrow_marker.scale.z = 0.3  # Head length
        
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 0.8
        
        marker_array.markers.append(arrow_marker)
        
        # Publish
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SoundSourceVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
