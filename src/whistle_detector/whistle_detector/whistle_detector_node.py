#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from whistle_safety_msgs.msg import AudioLevel, WhistleAlert
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import uuid
from datetime import datetime


class WhistleDetector(Node):
    """
    Detects whistle patterns from audio level data.
    Uses threshold-based detection with hysteresis to avoid false positives.
    """
    
    def __init__(self):
        super().__init__('whistle_detector')
        
        # Parameters
        self.declare_parameter('detection_threshold', 0.7)  # Level to trigger detection
        self.declare_parameter('confirmation_time', 0.5)    # Seconds to confirm
        self.declare_parameter('cooldown_time', 3.0)        # Seconds between alerts
        
        # Subscriber to audio levels
        self.audio_sub = self.create_subscription(
            AudioLevel,
            '/simulated_audio_level',
            self.audio_callback,
            10
        )
        
        # Publisher for whistle alerts
        self.alert_pub = self.create_publisher(
            WhistleAlert,
            '/whistle_alert',
            10
        )
        
        # State machine variables
        self.state = 'IDLE'  # States: IDLE, DETECTING, COOLDOWN
        self.detection_start_time = None
        self.last_alert_time = None
        self.current_intensity = 0.0
        self.last_position = Point(x=0.0, y=0.0, z=0.0)
        
        self.get_logger().info('Whistle Detector Node Started')
        self.get_logger().info(f'Detection threshold: {self.get_parameter("detection_threshold").value}')
        self.get_logger().info('Listening on /simulated_audio_level')
        self.get_logger().info('Publishing alerts to /whistle_alert')
    
    def audio_callback(self, msg: AudioLevel):
        """Process incoming audio level data"""
        
        threshold = self.get_parameter('detection_threshold').value
        confirmation_time = self.get_parameter('confirmation_time').value
        cooldown_time = self.get_parameter('cooldown_time').value
        
        self.current_intensity = msg.level
        self.last_position = msg.sensor_position
        
        current_time = self.get_clock().now()
        
        # State machine
        if self.state == 'IDLE':
            if msg.level >= threshold:
                self.state = 'DETECTING'
                self.detection_start_time = current_time
                self.get_logger().info(f'🎵 Whistle detected! Level: {msg.level:.3f} - Confirming...')
        
        elif self.state == 'DETECTING':
            if msg.level < threshold * 0.7:  # Hysteresis: 70% of threshold
                # Lost signal, back to idle
                self.state = 'IDLE'
                self.detection_start_time = None
                self.get_logger().info('Signal lost, returning to IDLE')
            else:
                # Check if confirmed
                elapsed = (current_time - self.detection_start_time).nanoseconds / 1e9
                if elapsed >= confirmation_time:
                    # Confirmed! Publish alert
                    self.publish_alert(msg)
                    self.state = 'COOLDOWN'
                    self.last_alert_time = current_time
                    self.detection_start_time = None
        
        elif self.state == 'COOLDOWN':
            # Wait for cooldown period
            elapsed = (current_time - self.last_alert_time).nanoseconds / 1e9
            if elapsed >= cooldown_time:
                self.state = 'IDLE'
                self.get_logger().info('Cooldown complete, ready for next detection')
    
    def publish_alert(self, audio_msg: AudioLevel):
        """Publish a whistle alert"""
        
        alert = WhistleAlert()
        alert.header = Header()
        alert.header.stamp = self.get_clock().now().to_msg()
        alert.header.frame_id = 'microphone_link'
        
        alert.intensity = self.current_intensity
        alert.position = self.last_position
        
        # Determine severity based on intensity
        if self.current_intensity >= 0.9:
            alert.severity = 'critical'
        elif self.current_intensity >= 0.8:
            alert.severity = 'high'
        elif self.current_intensity >= 0.7:
            alert.severity = 'medium'
        else:
            alert.severity = 'low'
        
        # Generate unique alert ID
        alert.alert_id = str(uuid.uuid4())[:8]
        
        # Publish
        self.alert_pub.publish(alert)
        
        self.get_logger().warn(f'🚨 WHISTLE ALERT PUBLISHED! 🚨')
        self.get_logger().warn(f'   ID: {alert.alert_id}')
        self.get_logger().warn(f'   Intensity: {alert.intensity:.3f}')
        self.get_logger().warn(f'   Severity: {alert.severity}')
        self.get_logger().warn(f'   Position: ({alert.position.x:.2f}, {alert.position.y:.2f}, {alert.position.z:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = WhistleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
