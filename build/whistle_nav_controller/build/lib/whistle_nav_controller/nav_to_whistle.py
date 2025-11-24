#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from whistle_safety_msgs.msg import WhistleAlert
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import time


class WhistleNavigationController(Node):
    """
    Autonomous navigation controller that drives robot to whistle locations
    """
    
    def __init__(self):
        super().__init__('whistle_nav_controller')
        
        # Parameters
        self.declare_parameter('enable_auto_navigation', True)
        self.declare_parameter('navigation_timeout', 60.0)
        # Allow configuring the NAV2 action name and optional namespace
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('nav_action_namespace', '')
        
        self.get_logger().info('🗺️  Whistle Navigation Controller Starting...')
        
        # Subscribe to whistle alerts
        self.alert_sub = self.create_subscription(
            WhistleAlert,
            '/whistle_alert',
            self.alert_callback,
            10
        )
        
        self.get_logger().info('✅ Subscribed to /whistle_alert')
        
        # Build action server name (optionally namespaced)
        action_name = str(self.get_parameter('nav_action_name').value)
        action_ns = str(self.get_parameter('nav_action_namespace').value)
        if action_ns:
            # normalize namespace (no trailing slash)
            action_ns = action_ns.strip('/')
            action_server = f'/{action_ns}/{action_name}'
        else:
            action_server = action_name

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, action_server)

        # State
        self.current_goal_handle = None
        self.navigation_in_progress = False
        self.alert_count = 0
        
        self.get_logger().info(f'⏳ Waiting for NAV2 action server at "{action_server}"...')

        # Wait for action server with timeout (configurable via parameter)
        timeout_sec = float(self.get_parameter('navigation_timeout').value)
        start_time = time.time()
        wait_interval = 1.0  # check every second
        connected = False

        # Loop with short waits so we can log progress and handle namespaced servers
        while time.time() - start_time < timeout_sec:
            try:
                if self.nav_client.wait_for_server(timeout_sec=wait_interval):
                    connected = True
                    break
            except Exception:
                # wait_for_server may raise during shutdown; ignore and retry until timeout
                pass

            self.get_logger().info('...still waiting for NAV2 action server...')

        if connected:
            self.get_logger().info('✅ Connected to NAV2 navigation server')
        else:
            self.get_logger().error(f'❌ NAV2 action server not available after {timeout_sec}s')
            self.get_logger().error('   Make sure NAV2 is running and that the action name/namespace are correct!')
    
    def alert_callback(self, msg: WhistleAlert):
        """Handle whistle alert and navigate to location"""
        
        self.alert_count += 1
        
        self.get_logger().info('=' * 60)
        self.get_logger().warn(f'🚨 RECEIVED WHISTLE ALERT #{self.alert_count}')
        self.get_logger().info(f'   Alert ID: {msg.alert_id}')
        self.get_logger().info(f'   Position: ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})')
        self.get_logger().info(f'   Severity: {msg.severity}')
        self.get_logger().info('=' * 60)
        
        if not self.get_parameter('enable_auto_navigation').value:
            self.get_logger().warn('⚠️  Auto-navigation DISABLED, ignoring alert')
            return
        
        if self.navigation_in_progress:
            self.get_logger().warn('⚠️  Navigation already in progress, ignoring new alert')
            return
        
        # Check if NAV2 is available
        if not self.nav_client.server_is_ready():
            self.get_logger().error('❌ NAV2 action server not ready!')
            return
        
        self.get_logger().info('🎯 AUTO-NAVIGATION ENABLED - Creating goal...')
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set target position (whistle location)
        # Note: Position might need adjustment based on coordinate frame
        goal_msg.pose.pose.position.x = float(msg.position.x)
        goal_msg.pose.pose.position.y = float(msg.position.y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (face toward whistle)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'📍 Navigation target: ({goal_msg.pose.pose.position.x:.2f}, '
                              f'{goal_msg.pose.pose.position.y:.2f})')
        
        # Send goal
        self.send_navigation_goal(goal_msg, msg.alert_id)
    
    def send_navigation_goal(self, goal_msg, alert_id):
        """Send navigation goal to NAV2"""
        
        self.get_logger().info('🚀 Sending navigation goal to NAV2...')
        self.navigation_in_progress = True
        
        try:
            # Send goal
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            
            send_goal_future.add_done_callback(
                lambda future: self.goal_response_callback(future, alert_id)
            )
            
            self.get_logger().info('✅ Goal sent to NAV2, waiting for response...')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send goal: {e}')
            self.navigation_in_progress = False
    
    def goal_response_callback(self, future, alert_id):
        """Handle navigation goal response"""
        
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('❌ Navigation goal REJECTED by NAV2')
                self.navigation_in_progress = False
                return
            
            self.get_logger().warn(f'✅ Navigation goal ACCEPTED (Alert: {alert_id})')
            self.get_logger().info('🤖 Robot is now navigating to whistle location...')
            self.current_goal_handle = goal_handle
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error in goal response: {e}')
            self.navigation_in_progress = False
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # Log progress
        if hasattr(self, '_feedback_counter'):
            self._feedback_counter += 1
        else:
            self._feedback_counter = 0
        
        if self._feedback_counter % 10 == 0:  # Every second at 10Hz
            distance_remaining = feedback.distance_remaining
            self.get_logger().info(
                f'📍 Position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) | '
                f'Distance: {distance_remaining:.2f}m'
            )
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        
        try:
            result = future.result().result
            status = future.result().status
            
            self.navigation_in_progress = False
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn('=' * 60)
                self.get_logger().warn('🎉 NAVIGATION SUCCESSFUL!')
                self.get_logger().warn('🤖 Robot reached whistle location')
                self.get_logger().warn('=' * 60)
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error('❌ Navigation ABORTED')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('⚠️  Navigation CANCELED')
            else:
                self.get_logger().error(f'❌ Navigation failed with status: {status}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error in navigation result: {e}')
            self.navigation_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = WhistleNavigationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
