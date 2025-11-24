#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from whistle_safety_msgs.msg import WhistleAlert
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
from datetime import datetime


class AlertManager(Node):
    """
    Manages whistle alerts:
    - Logs all alerts
    - Enriches alerts with robot position
    - Formats alerts for cloud transmission
    - Visualizes alerts in RViz
    """
    
    def __init__(self):
        super().__init__('alert_manager')
        
        # Subscribe to whistle alerts
        self.alert_sub = self.create_subscription(
            WhistleAlert,
            '/whistle_alert',
            self.alert_callback,
            10
        )
        
        # Subscribe to robot odometry for position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for cloud-formatted alerts
        self.cloud_pub = self.create_publisher(
            String,
            '/alert_to_cloud',
            10
        )
        
        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/alert_markers',
            10
        )
        
        # Store robot position
        self.robot_position = Point(x=0.0, y=0.0, z=0.0)
        self.robot_orientation = None
        
        # Alert history
        self.alert_history = []
        self.marker_id = 0
        
        self.get_logger().info('Alert Manager Node Started')
        self.get_logger().info('Subscribed to /whistle_alert')
        self.get_logger().info('Publishing to /alert_to_cloud for Zenoh bridge')
        self.get_logger().info('Publishing markers to /alert_markers for RViz')
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry"""
        self.robot_position = msg.pose.pose.position
        self.robot_orientation = msg.pose.pose.orientation
    
    def alert_callback(self, msg: WhistleAlert):
        """Handle incoming whistle alert"""
        
        self.get_logger().warn('=' * 60)
        self.get_logger().warn('🚨 ALERT MANAGER: Processing Whistle Alert')
        self.get_logger().warn('=' * 60)
        
        # Create enriched alert data
        alert_data = {
            'alert_id': msg.alert_id,
            'timestamp': datetime.now().isoformat(),
            'type': 'whistle_detection',
            'severity': msg.severity,
            'intensity': float(msg.intensity),
            'robot_position': {
                'x': float(self.robot_position.x),
                'y': float(self.robot_position.y),
                'z': float(self.robot_position.z)
            },
            'sensor_position': {
                'x': float(msg.position.x),
                'y': float(msg.position.y),
                'z': float(msg.position.z)
            },
            'robot_id': 'whistle_robot_01',
            'location_name': 'Simulation Environment'
        }
        
        # Add to history
        self.alert_history.append(alert_data)
        
        # Log details
        self.get_logger().warn(f'Alert ID: {alert_data["alert_id"]}')
        self.get_logger().warn(f'Severity: {alert_data["severity"]}')
        self.get_logger().warn(f'Intensity: {alert_data["intensity"]:.3f}')
        self.get_logger().warn(f'Robot Position: ({alert_data["robot_position"]["x"]:.2f}, '
                              f'{alert_data["robot_position"]["y"]:.2f}, '
                              f'{alert_data["robot_position"]["z"]:.2f})')
        self.get_logger().warn(f'Total Alerts: {len(self.alert_history)}')
        
        # Publish to cloud topic
        cloud_msg = String()
        cloud_msg.data = json.dumps(alert_data, indent=2)
        self.cloud_pub.publish(cloud_msg)
        self.get_logger().info('✅ Alert published to /alert_to_cloud')
        
        # Create visualization marker
        self.publish_marker(alert_data)
        
        self.get_logger().warn('=' * 60)
    
    def publish_marker(self, alert_data):
        """Create RViz marker for alert location"""
        
        marker_array = MarkerArray()
        
        # Marker 1: Sphere at alert location
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'whistle_alerts'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = alert_data['robot_position']['x']
        marker.pose.position.y = alert_data['robot_position']['y']
        marker.pose.position.z = alert_data['robot_position']['z'] + 0.5
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # Color based on severity
        if alert_data['severity'] == 'critical':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif alert_data['severity'] == 'high':
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
        elif alert_data['severity'] == 'medium':
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 1.0
        
        marker.color.a = 0.8
        marker.lifetime.sec = 30  # 30 seconds
        
        marker_array.markers.append(marker)
        
        # Marker 2: Text label
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = 'whistle_labels'
        text_marker.id = self.marker_id
        self.marker_id += 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = alert_data['robot_position']['x']
        text_marker.pose.position.y = alert_data['robot_position']['y']
        text_marker.pose.position.z = alert_data['robot_position']['z'] + 1.0
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = f"WHISTLE\n{alert_data['severity'].upper()}\n{alert_data['alert_id']}"
        text_marker.lifetime.sec = 30
        
        marker_array.markers.append(text_marker)
        
        # Publish
        self.marker_pub.publish(marker_array)
        self.get_logger().info('✅ Alert marker published to RViz')


def main(args=None):
    rclpy.init(args=args)
    node = AlertManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()