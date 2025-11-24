#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from whistle_safety_msgs.msg import AudioLevel
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import random


class MicrophoneSimulator(Node):
    """
    Simulates a microphone sensor with distance-based sound detection.
    Sound intensity decreases with distance from virtual sound sources.
    """
    
    def __init__(self):
        super().__init__('microphone_simulator')
        
        # Publisher for audio levels
        self.audio_pub = self.create_publisher(
            AudioLevel,
            '/simulated_audio_level',
            10
        )
        
        # Subscriber to robot odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('base_noise_level', 0.1)
        self.declare_parameter('whistle_intensity', 0.0)  # Manual trigger
        
        # Sound source parameters
        self.declare_parameter('sound_source_x', 3.0)  # Whistle location X
        self.declare_parameter('sound_source_y', 3.0)  # Whistle location Y
        self.declare_parameter('sound_source_z', 0.5)  # Whistle location Z
        self.declare_parameter('max_detection_range', 10.0)  # meters
        self.declare_parameter('sound_source_active', False)  # Auto whistle
        
        # Robot state
        self.robot_position = Point(x=0.0, y=0.0, z=0.2)
        
        # Microphone position (relative to robot base_link)
        self.mic_position = Point(x=0.25, y=0.0, z=0.3)
        
        # Get publish rate
        rate = self.get_parameter('publish_rate').value
        timer_period = 1.0 / rate
        
        # Create timer
        self.timer = self.create_timer(timer_period, self.publish_audio_level)
        
        self.get_logger().info('Microphone Simulator Node Started (Distance-Based)')
        self.get_logger().info(f'Publishing at {rate} Hz on /simulated_audio_level')
        self.get_logger().info('Sound detection modes:')
        self.get_logger().info('  1. Manual: ros2 param set /microphone_simulator whistle_intensity <0.0-1.0>')
        self.get_logger().info('  2. Auto: ros2 param set /microphone_simulator sound_source_active true')
        
        # Log sound source location
        src_x = self.get_parameter('sound_source_x').value
        src_y = self.get_parameter('sound_source_y').value
        src_z = self.get_parameter('sound_source_z').value
        self.get_logger().info(f'Virtual whistle source at: ({src_x:.1f}, {src_y:.1f}, {src_z:.1f})')
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry"""
        self.robot_position = msg.pose.pose.position
    
    def calculate_distance_to_source(self):
        """Calculate distance from robot to sound source"""
        src_x = self.get_parameter('sound_source_x').value
        src_y = self.get_parameter('sound_source_y').value
        src_z = self.get_parameter('sound_source_z').value
        
        # Calculate 3D distance
        dx = self.robot_position.x - src_x
        dy = self.robot_position.y - src_y
        dz = self.robot_position.z - src_z
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance
    
    def calculate_sound_intensity(self, distance):
        """
        Calculate sound intensity based on distance
        Uses inverse square law: intensity = 1 / (distance^2)
        """
        max_range = self.get_parameter('max_detection_range').value
        
        if distance > max_range:
            return 0.0
        
        # Inverse square law with some modifications for better detection
        if distance < 0.5:
            distance = 0.5  # Prevent division by very small numbers
        
        intensity = 1.0 / (distance ** 1.5)  # Using 1.5 instead of 2 for better range
        
        # Normalize to 0-1 range
        intensity = min(1.0, intensity)
        
        return intensity
    
    def publish_audio_level(self):
        """Publish simulated audio level"""
        
        # Get parameters
        base_noise = self.get_parameter('base_noise_level').value
        manual_intensity = self.get_parameter('whistle_intensity').value
        sound_source_active = self.get_parameter('sound_source_active').value
        
        # Calculate audio level
        audio_level = base_noise
        
        # Mode 1: Manual trigger (overrides everything)
        if manual_intensity > 0.0:
            audio_level = manual_intensity
        
        # Mode 2: Automatic distance-based detection
        elif sound_source_active:
            distance = self.calculate_distance_to_source()
            sound_intensity = self.calculate_sound_intensity(distance)
            audio_level = base_noise + sound_intensity
            
            # Log occasionally when sound source is active
            if hasattr(self, '_distance_log_counter'):
                self._distance_log_counter += 1
            else:
                self._distance_log_counter = 0
            
            if self._distance_log_counter % 50 == 0:  # Every 5 seconds at 10Hz
                self.get_logger().info(
                    f'Distance to whistle: {distance:.2f}m | '
                    f'Sound intensity: {sound_intensity:.3f} | '
                    f'Audio level: {audio_level:.3f}'
                )
        
        # Add random noise to make it realistic
        noise = random.gauss(0, 0.02)
        audio_level += noise
        
        # Clamp to [0, 1]
        audio_level = max(0.0, min(1.0, audio_level))
        
        # Create message
        msg = AudioLevel()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'microphone_link'
        msg.level = audio_level
        msg.frequency = 3000.0  # Whistle frequency ~3kHz
        msg.sensor_position = self.mic_position
        
        # Publish
        self.audio_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
