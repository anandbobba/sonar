#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

# Try to import zenoh, provide fallback if not installed
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    # Create a dummy zenoh module for type hints
    class zenoh:  # type: ignore
        class Config:
            pass
        class Session:
            def put(self, key: str, value: str) -> None:
                pass
            def close(self) -> None:
                pass


class ZenohBridge(Node):
    """
    Bridges ROS 2 topics to Zenoh for cloud communication.
    Subscribes to /alert_to_cloud and publishes to Zenoh network.
    
    NOTE: This node works in two modes:
    1. WITH Zenoh installed: Sends alerts to actual cloud endpoints
    2. WITHOUT Zenoh: Runs in simulation mode (logs only)
    """
    
    def __init__(self):
        super().__init__('zenoh_bridge')
        
        # Parameters
        self.declare_parameter('zenoh_mode', 'peer')  # peer or client
        self.declare_parameter('zenoh_topic', 'cloud/alerts/whistle')
        self.declare_parameter('connect_endpoint', '')  # e.g., 'tcp/192.168.1.100:7447'
        
        # Subscribe to ROS topic
        self.alert_sub = self.create_subscription(
            String,
            '/alert_to_cloud',
            self.alert_callback,
            10
        )
        
        # Initialize Zenoh session
        self.zenoh_session = None
        if ZENOH_AVAILABLE:
            self.init_zenoh()
        else:
            self.print_zenoh_warning()
        
        self.alert_count = 0
        
        self.get_logger().info('Zenoh Bridge Node Started')
        self.get_logger().info(f'Zenoh Available: {ZENOH_AVAILABLE}')
        self.get_logger().info('Subscribed to /alert_to_cloud')
        if ZENOH_AVAILABLE:
            zenoh_topic = self.get_parameter('zenoh_topic').value
            self.get_logger().info(f'Publishing to Zenoh topic: {zenoh_topic}')
    
    def print_zenoh_warning(self):
        """Print installation instructions if Zenoh not available"""
        self.get_logger().warn('=' * 70)
        self.get_logger().warn('⚠️  ZENOH NOT INSTALLED - RUNNING IN SIMULATION MODE')
        self.get_logger().warn('=' * 70)
        self.get_logger().warn('The system will work, but alerts won\'t be sent to cloud.')
        self.get_logger().warn('')
        self.get_logger().warn('To enable cloud functionality, install Zenoh:')
        self.get_logger().warn('  $ pip3 install eclipse-zenoh')
        self.get_logger().warn('')
        self.get_logger().warn('Or if using conda/venv:')
        self.get_logger().warn('  $ pip install eclipse-zenoh')
        self.get_logger().warn('')
        self.get_logger().warn('After installation, restart this node.')
        self.get_logger().warn('=' * 70)
    
    def init_zenoh(self):
        """Initialize Zenoh session"""
        try:
            mode = self.get_parameter('zenoh_mode').value
            connect_endpoint = self.get_parameter('connect_endpoint').value
            
            # Configure Zenoh
            config = zenoh.Config()
            
            if connect_endpoint:
                config.insert_json5("connect/endpoints", json.dumps([connect_endpoint]))
            
            # Open session
            self.zenoh_session = zenoh.open(config)
            self.get_logger().info(f'✅ Zenoh session opened in {mode} mode')
            
            if connect_endpoint:
                self.get_logger().info(f'   Connected to: {connect_endpoint}')
            else:
                self.get_logger().info('   Running in local peer mode')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Zenoh: {e}')
            self.get_logger().error('Falling back to simulation mode')
            self.zenoh_session = None
    
    def alert_callback(self, msg: String):
        """Forward ROS alert to Zenoh"""
        
        self.alert_count += 1
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📡 ZENOH BRIDGE: Forwarding Alert #{self.alert_count}')
        self.get_logger().info('=' * 60)
        
        try:
            # Parse JSON
            alert_data = json.loads(msg.data)
            
            # Log alert summary
            self.get_logger().info(f'Alert ID: {alert_data.get("alert_id", "unknown")}')
            self.get_logger().info(f'Severity: {alert_data.get("severity", "unknown")}')
            self.get_logger().info(f'Timestamp: {alert_data.get("timestamp", "unknown")}')
            
            if ZENOH_AVAILABLE and self.zenoh_session:
                # Publish to Zenoh
                zenoh_topic = self.get_parameter('zenoh_topic').value
                self.zenoh_session.put(zenoh_topic, msg.data)
                self.get_logger().info(f'✅ Published to Zenoh: {zenoh_topic}')
            else:
                # Simulation mode - just log
                self.get_logger().warn('⚠️  SIMULATION MODE: Alert logged but not sent')
                self.get_logger().info(f'Would publish to: {self.get_parameter("zenoh_topic").value}')
            
            # Pretty print the alert (only first 3 alerts to avoid spam)
            if self.alert_count <= 3:
                self.get_logger().info('Alert Content:')
                self.get_logger().info(json.dumps(alert_data, indent=2))
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse alert JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error publishing to Zenoh: {e}')
        
        self.get_logger().info('=' * 60)
    
    def destroy_node(self):
        """Cleanup Zenoh session on shutdown"""
        if self.zenoh_session:
            try:
                self.zenoh_session.close()
                self.get_logger().info('Zenoh session closed')
            except Exception as e:
                self.get_logger().error(f'Error closing Zenoh session: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZenohBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()