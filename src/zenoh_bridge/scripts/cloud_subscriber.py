#!/usr/bin/env python3
"""
Cloud Subscriber Simulator
This simulates a cloud service receiving alerts from the robot via Zenoh.
Run this separately to test the complete pipeline.
"""

import json
import time
from datetime import datetime

# Try to import zenoh
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    print("⚠️  Zenoh not installed!")
    print("Install with: pip install eclipse-zenoh")
    exit(1)


class CloudAlertReceiver:
    """Simulates a cloud service receiving whistle alerts"""
    
    def __init__(self):
        self.alert_database = []
        
        # Configure Zenoh
        config = zenoh.Config()
        
        # Open Zenoh session
        print("🌐 Opening Zenoh session...")
        self.session = zenoh.open(config)
        print("✅ Cloud subscriber started!")
        print("📡 Listening for alerts on: cloud/alerts/whistle")
        print("=" * 60)
    
    def alert_callback(self, sample):
        """Handle incoming alert from robot"""
        try:
            # Decode the payload
            payload = sample.payload.decode('utf-8')
            alert_data = json.loads(payload)
            
            # Store in "database"
            self.alert_database.append({
                'received_at': datetime.now().isoformat(),
                'alert': alert_data
            })
            
            # Print alert
            print("\n" + "=" * 60)
            print("🚨 CLOUD: NEW ALERT RECEIVED!")
            print("=" * 60)
            print(f"Received at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"Alert ID: {alert_data.get('alert_id', 'unknown')}")
            print(f"Severity: {alert_data.get('severity', 'unknown')}")
            print(f"Intensity: {alert_data.get('intensity', 0):.3f}")
            print(f"Robot ID: {alert_data.get('robot_id', 'unknown')}")
            
            robot_pos = alert_data.get('robot_position', {})
            print(f"Location: ({robot_pos.get('x', 0):.2f}, "
                  f"{robot_pos.get('y', 0):.2f}, "
                  f"{robot_pos.get('z', 0):.2f})")
            
            print(f"\nTotal alerts received: {len(self.alert_database)}")
            print("=" * 60)
            
            # Simulate sending notification
            self.send_notification(alert_data)
            
        except json.JSONDecodeError as e:
            print(f"❌ Error decoding alert: {e}")
        except Exception as e:
            print(f"❌ Error processing alert: {e}")
    
    def send_notification(self, alert_data):
        """Simulate sending push notification"""
        severity = alert_data.get('severity', 'unknown')
        
        if severity in ['critical', 'high']:
            print("📱 NOTIFICATION: Emergency whistle detected!")
            print("   → Push notification sent to all devices")
            print("   → SMS sent to emergency contacts")
        else:
            print("📱 NOTIFICATION: Whistle alert logged")
    
    def start_listening(self):
        """Start listening for alerts"""
        # Subscribe to Zenoh topic
        subscriber = self.session.declare_subscriber(
            'cloud/alerts/whistle',
            self.alert_callback
        )
        
        print("\n🎧 Waiting for alerts from robot...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\n👋 Shutting down cloud subscriber...")
    
    def cleanup(self):
        """Cleanup resources"""
        if self.session:
            self.session.close()
        
        # Print summary
        print("\n" + "=" * 60)
        print("📊 SESSION SUMMARY")
        print("=" * 60)
        print(f"Total alerts received: {len(self.alert_database)}")
        
        if self.alert_database:
            print("\nAlert History:")
            for idx, record in enumerate(self.alert_database, 1):
                alert = record['alert']
                print(f"\n  {idx}. Alert ID: {alert.get('alert_id', 'unknown')}")
                print(f"     Severity: {alert.get('severity', 'unknown')}")
                print(f"     Received: {record['received_at']}")
        
        print("\n✅ Cloud subscriber stopped")


def main():
    print("\n" + "=" * 60)
    print("☁️  CLOUD ALERT RECEIVER SIMULATOR")
    print("=" * 60)
    print("This simulates a cloud service receiving robot alerts")
    print("Run this in a separate terminal while the robot is running")
    print("=" * 60 + "\n")
    
    if not ZENOH_AVAILABLE:
        return
    
    receiver = CloudAlertReceiver()
    
    try:
        receiver.start_listening()
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        receiver.cleanup()


if __name__ == '__main__':
    main()