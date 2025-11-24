#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from whistle_safety_msgs.msg import AudioLevel
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class AutonomousResponder(Node):
    """
    Moves robot toward sound source based on audio intensity
    """
    
    def __init__(self):
        super().__init__('autonomous_responder')
        
        # Subscribe to audio levels
        self.audio_sub = self.create_subscription(
            AudioLevel,
            '/simulated_audio_level',
            self.audio_callback,
            10
        )
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to laser scan for obstacle avoidance
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for robot velocity
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters - Get values and convert to float to handle both int and float inputs
        self.declare_parameter('activation_threshold', 0.15)  # Minimum to start moving
        self.declare_parameter('weak_threshold', 0.3)         # Below this = very slow movement
        self.declare_parameter('moderate_threshold', 0.6)     # Above this = normal speed
        self.declare_parameter('stop_threshold', 0.8)         # Stop when reached
        self.declare_parameter('weak_speed', 0.1)             # Speed for weak sound
        self.declare_parameter('moderate_speed', 0.3)         # Speed for moderate sound
        self.declare_parameter('strong_speed', 0.5)           # Speed for strong sound
        self.declare_parameter('sound_x', 3.0)
        self.declare_parameter('sound_y', 3.0)
        
        # Get parameters and ensure they're floats (handles both int and float inputs)
        sound_x = float(self.get_parameter('sound_x').value)
        sound_y = float(self.get_parameter('sound_y').value)
        # Smoothing and behavior tuning
        self.declare_parameter('audio_alpha', 0.6)  # EMA alpha for audio smoothing (0..1)
        self.declare_parameter('stop_delay', 0.5)   # seconds to wait before treating audio as stopped
        self.declare_parameter('force_move', False) # if true, ignore audio and move towards target immediately

        # State
        self.current_audio = 0.0
        self.smoothed_audio = 0.0
        self.last_heard_time = 0.0
        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0
        self.moving = False
        
        # Obstacle avoidance state
        self.obstacle_detected = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_front = False
        self.min_safe_distance = 0.5  # meters
        
        weak_th = self.get_parameter('weak_threshold').value
        moderate_th = self.get_parameter('moderate_threshold').value
        activation_th = self.get_parameter('activation_threshold').value
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 AUTONOMOUS SOUND FOLLOWER ACTIVATED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Target sound source: ({sound_x:.1f}, {sound_y:.1f})')
        self.get_logger().info('Robot will automatically move toward sound')
        self.get_logger().info('Behavior based on sound strength:')
        self.get_logger().info(f'  🛑 < {activation_th:.2f}: STOP (no sound)')
        self.get_logger().info(f'  🐢 {activation_th:.2f}-{weak_th:.2f}: VERY SLOW')
        self.get_logger().info(f'  🚶 {weak_th:.2f}-{moderate_th:.2f}: SLOW')
        self.get_logger().info(f'  🏃 > {moderate_th:.2f}: FAST')
        self.get_logger().info(f'  ✋ > {self.get_parameter("stop_threshold").value:.2f}: STOP (arrived)')
        self.get_logger().info('=' * 60)
    
    def audio_callback(self, msg: AudioLevel):
        """React to audio intensity changes"""
        self.current_audio = msg.level

        # Parameters (re-read to allow runtime changes)
        activation = self.get_parameter('activation_threshold').value
        weak_threshold = self.get_parameter('weak_threshold').value
        moderate_threshold = self.get_parameter('moderate_threshold').value
        stop_threshold = self.get_parameter('stop_threshold').value
        weak_speed = self.get_parameter('weak_speed').value
        moderate_speed = self.get_parameter('moderate_speed').value
        strong_speed = self.get_parameter('strong_speed').value
        sound_x = self.get_parameter('sound_x').value
        sound_y = self.get_parameter('sound_y').value
        alpha = self.get_parameter('audio_alpha').value
        stop_delay = self.get_parameter('stop_delay').value
        force_move = self.get_parameter('force_move').value

        # Exponential moving average to smooth noisy audio and avoid flicker
        try:
            alpha = float(alpha)
        except Exception:
            alpha = 0.6
        self.smoothed_audio = alpha * self.current_audio + (1.0 - alpha) * self.smoothed_audio

        now = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        # update last_heard_time when above activation
        if self.smoothed_audio > activation:
            self.last_heard_time = now
        
        # Calculate distance to sound source
        dx = sound_x - self.current_position[0]
        dy = sound_y - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        cmd = Twist()
        
        # Determine effective audio for decision-making
        effective_audio = self.smoothed_audio

        # If force_move is set, treat as if activation is always met
        if force_move:
            effective_audio = max(effective_audio, activation + 1.0)

        if effective_audio > stop_threshold:
            # Very close - stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.moving:
                self.get_logger().warn('=' * 60)
                self.get_logger().warn('🎉 ARRIVED AT SOUND SOURCE!')
                self.get_logger().warn(f'   Final distance: {distance:.2f}m')
                self.get_logger().warn(f'   Audio level: {self.current_audio:.3f} (smoothed {self.smoothed_audio:.3f})')
                self.get_logger().warn('=' * 60)
                self.moving = False
        
        # If audio was recently heard within stop_delay, keep moving
        elif (self.smoothed_audio > activation) or (now - self.last_heard_time < stop_delay):
            # Sound detected - move toward it
            if not self.moving:
                self.get_logger().info('🚗 Sound detected! Starting movement...')
                self.moving = True
            
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)
            
            # Determine speed based on sound strength
            if effective_audio >= moderate_threshold:
                # Strong sound - move fast
                base_speed = strong_speed
                behavior = "🏃 FAST"
            elif effective_audio >= weak_threshold:
                # Moderate sound - move normally
                base_speed = moderate_speed
                behavior = "🚶 SLOW"
            else:
                # Weak sound - move very slowly
                base_speed = weak_speed
                behavior = "🐢 VERY SLOW"
            
            # Obstacle avoidance logic
            if self.obstacle_detected:
                if self.obstacle_front:
                    # Obstacle in front - turn to avoid
                    if not self.obstacle_right or (self.obstacle_left and not self.obstacle_right):
                        # Turn right
                        cmd.angular.z = -0.6
                        cmd.linear.x = 0.05
                        behavior = "⚠️ AVOIDING RIGHT"
                    else:
                        # Turn left
                        cmd.angular.z = 0.6
                        cmd.linear.x = 0.05
                        behavior = "⚠️ AVOIDING LEFT"
                else:
                    # No front obstacle, proceed with adjusted steering
                    if abs(angle_diff) > 0.2:  # Need to turn
                        cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
                        cmd.linear.x = base_speed * 0.4  # Slower near obstacles
                    else:  # Move forward cautiously
                        cmd.linear.x = min(base_speed * 0.6, distance * 0.3)
                        cmd.angular.z = angle_diff * 1.5
                    behavior = behavior + " (near obstacle)"
            else:
                # No obstacles - normal control logic
                if abs(angle_diff) > 0.2:  # Need to turn
                    cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
                    cmd.linear.x = base_speed * 0.3  # Slower while turning
                else:  # Move forward
                    cmd.linear.x = min(base_speed, distance * 0.3)
                    cmd.angular.z = angle_diff * 1.5
            
            # Log progress
            if not hasattr(self, '_log_counter'):
                self._log_counter = 0
            self._log_counter += 1
            
            if self._log_counter % 50 == 0:  # Every 5 seconds
                self.get_logger().info(
                    f'{behavior} | 📍 Distance: {distance:.2f}m | '
                    f'Audio: {self.current_audio:.3f} (smoothed: {self.smoothed_audio:.3f}) | '
                    f'Heading error: {math.degrees(angle_diff):.1f}° | '
                    f'Speed: {cmd.linear.x:.2f} m/s'
                )
        else:
            # No significant sound - stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.moving:
                self.get_logger().info('🛑 Sound too weak, stopping...')
                self.moving = False
        
        self.vel_pub.publish(cmd)
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan data for obstacle detection"""
        if len(msg.ranges) == 0:
            return
        
        # Divide scan into sectors: front, left, right
        num_readings = len(msg.ranges)
        front_sector = num_readings // 3
        
        # Front: middle third of scan
        front_start = num_readings // 3
        front_end = 2 * num_readings // 3
        front_ranges = [r for r in msg.ranges[front_start:front_end] if not math.isinf(r) and not math.isnan(r)]
        
        # Left: first third
        left_ranges = [r for r in msg.ranges[:front_start] if not math.isinf(r) and not math.isnan(r)]
        
        # Right: last third
        right_ranges = [r for r in msg.ranges[front_end:] if not math.isinf(r) and not math.isnan(r)]
        
        # Check for obstacles
        self.obstacle_front = len(front_ranges) > 0 and min(front_ranges) < self.min_safe_distance
        self.obstacle_left = len(left_ranges) > 0 and min(left_ranges) < self.min_safe_distance * 0.7
        self.obstacle_right = len(right_ranges) > 0 and min(right_ranges) < self.min_safe_distance * 0.7
        
        self.obstacle_detected = self.obstacle_front or self.obstacle_left or self.obstacle_right
    
    def odom_callback(self, msg: Odometry):
        """Update robot position"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousResponder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        cmd = Twist()
        node.vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
