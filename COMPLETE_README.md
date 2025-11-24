# 🚨 Whistle Safety & Emergency Response System

## 📋 Table of Contents
1. [Problem Statement](#problem-statement)
2. [Objective](#objective)
3. [System Architecture](#system-architecture)
4. [What is Zenoh?](#what-is-zenoh)
5. [Components Overview](#components-overview)
6. [How It Works](#how-it-works)
7. [Installation & Setup](#installation--setup)
8. [Running the System](#running-the-system)
9. [File Structure](#file-structure)
10. [Configuration Parameters](#configuration-parameters)
11. [Troubleshooting](#troubleshooting)

---

## 🎯 Problem Statement

**Emergency situations** in industrial facilities, warehouses, construction sites, or disaster zones often require immediate robotic assistance. Human operators may be:
- **Injured or incapacitated** and unable to call for help
- **Trapped** in hazardous areas where communication devices don't work
- **In need of immediate attention** but unable to reach communication equipment

**Challenge**: How can a robot autonomously detect and respond to emergency distress signals (like a whistle) and alert remote monitoring systems?

### Real-World Scenarios:
1. **Industrial Safety**: Worker injury in a noisy factory where verbal communication is impossible
2. **Warehouse Operations**: Forklift accident where radio communication is out of reach
3. **Construction Sites**: Worker trapped under debris with only a safety whistle
4. **Search & Rescue**: Finding survivors in disaster zones who can only whistle for help
5. **Elderly Care Facilities**: Emergency call system based on whistle signals

---

## 🎯 Objective

**Develop an intelligent robotic system that:**

1. **Detects Emergency Signals**
   - Continuously monitors audio levels for whistle-like patterns
   - Distinguishes between normal noise and emergency whistles
   - Provides threshold-based detection with hysteresis to avoid false positives

2. **Responds Autonomously**
   - Automatically navigates toward the sound source
   - Adapts movement speed based on signal strength:
     - **Weak signal** → Move slowly (cautious search)
     - **Moderate signal** → Move at normal speed (approaching)
     - **Strong signal** → Move quickly (emergency response)
     - **Very strong signal** → Stop (arrived at source)
   - Includes obstacle avoidance for safe navigation

3. **Alerts Remote Operators**
   - Enriches alerts with robot position, timestamp, and severity
   - Transmits alerts to cloud monitoring systems using Zenoh
   - Provides real-time visualization in RViz
   - Maintains alert history for incident analysis

4. **Scalability & Cloud Integration**
   - Uses Zenoh for efficient, low-latency cloud communication
   - Supports multi-robot deployments
   - Works in both connected and disconnected modes

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    PHYSICAL LAYER (Gazebo)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ Mobile Robot │  │ Sound Source │  │  Obstacles   │         │
│  │ (with sensors)│ │  (whistle)  │  │  (dynamic)   │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                     SENSOR LAYER (ROS 2)                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │  Microphone  │  │    Lidar     │  │   Odometry   │         │
│  │  Simulator   │  │  (obstacle   │  │  (position)  │         │
│  │ (audio level)│  │  detection)  │  │              │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                   PROCESSING LAYER (ROS 2)                      │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  Whistle Detector (Pattern Recognition)               │    │
│  │  - Threshold detection                                 │    │
│  │  - Signal confirmation                                 │    │
│  │  - False positive filtering                            │    │
│  └────────────────────────────────────────────────────────┘    │
│                             ↓                                    │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  Autonomous Responder (Navigation)                     │    │
│  │  - Adaptive speed control                              │    │
│  │  - Obstacle avoidance                                  │    │
│  │  - Path planning toward sound source                   │    │
│  └────────────────────────────────────────────────────────┘    │
│                             ↓                                    │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  Alert Manager (Data Enrichment)                       │    │
│  │  - Position tagging                                    │    │
│  │  - Severity classification                             │    │
│  │  - Alert formatting                                    │    │
│  └────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                   CLOUD LAYER (Zenoh Bridge)                    │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  Zenoh Bridge (Cloud Communication)                    │    │
│  │  - ROS 2 to Zenoh translation                          │    │
│  │  - Low-latency transmission                            │    │
│  │  - Pub/Sub routing                                     │    │
│  └────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                REMOTE MONITORING (Cloud/Dashboard)              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │  Alert Panel │  │  Fleet Mgmt  │  │  Analytics   │         │
│  │  (real-time) │  │  Dashboard   │  │  & Logging   │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🌐 What is Zenoh?

**Zenoh** (Zero Overhead Network Protocol) is a modern, high-performance communication protocol designed for:

### Key Features:
- **Zero-copy data transmission** - Minimal latency and overhead
- **Pub/Sub pattern** - Similar to ROS 2 but optimized for cloud
- **Location transparency** - Works across local networks, WANs, and cloud
- **Storage capabilities** - Can store and replay data
- **Query/Response** - Support for request-reply patterns

### Why Zenoh for This Project?

1. **Efficient Cloud Communication**
   - Traditional ROS 2 DDS not designed for internet/cloud
   - Zenoh provides optimized WAN/internet communication
   - Lower bandwidth usage than MQTT or ROS 2 over WAN

2. **Scalability**
   - Supports multiple robots sending alerts simultaneously
   - Can handle thousands of sensors and robots
   - Built-in routing and discovery

3. **Flexibility**
   - Works in peer-to-peer or client-server modes
   - Can bridge multiple networks
   - Supports both real-time and stored data access

4. **Integration**
   - Easy bridge between ROS 2 and cloud systems
   - Compatible with existing IoT infrastructure
   - Can integrate with databases, dashboards, analytics tools

### Zenoh vs Other Protocols:

| Feature | Zenoh | MQTT | DDS (ROS 2) | HTTP/REST |
|---------|-------|------|-------------|-----------|
| Latency | Very Low | Low | Very Low (LAN) | High |
| WAN Performance | Excellent | Good | Poor | Moderate |
| Bandwidth | Low | Moderate | High (WAN) | High |
| Pub/Sub | Yes | Yes | Yes | No |
| Storage | Yes | No | No | With DB |
| Discovery | Automatic | Broker needed | Automatic | Manual |

### How Zenoh is Used in This Project:

```
ROS 2 Topic (/alert_to_cloud)
         ↓
    Zenoh Bridge Node
         ↓
   Zenoh Network (Pub/Sub)
         ↓
Cloud Subscribers
  - Alert Dashboard
  - Analytics Engine
  - Mobile Apps
  - Other Robots
```

**Installation:**
```bash
pip3 install eclipse-zenoh
```

**Note:** The system works in simulation mode even without Zenoh installed. Zenoh is only required for actual cloud connectivity.

---

## 🔧 Components Overview

### 1. **sim_microphone_sensor** 📡
**Purpose**: Simulates a microphone sensor that detects sound intensity

**What it does:**
- Calculates audio levels based on robot's distance from sound source
- Simulates realistic sound attenuation (inverse square law)
- Adds background noise for realism
- Publishes audio levels at 10 Hz

**Key Features:**
- Distance-based sound simulation
- Configurable sound source position
- Manual or automatic whistle activation
- Base noise level simulation

**Topics:**
- Publishes: `/simulated_audio_level` (whistle_safety_msgs/AudioLevel)

**Parameters:**
- `publish_rate`: Update frequency (default: 10.0 Hz)
- `base_noise_level`: Background noise (default: 0.1)
- `whistle_intensity`: Manual whistle control (0.0-1.0)
- `sound_source_active`: Auto-enable sound at configured position

**Files:**
- `sim_microphone_sensor/microphone_simulator.py` - Main node
- `launch/microphone_simulator.launch.py` - Launch file

---

### 2. **whistle_detector** 🎵
**Purpose**: Detects whistle patterns from audio data using threshold-based detection

**What it does:**
- Monitors audio levels continuously
- Detects sustained high-intensity sounds (whistles)
- Uses confirmation time to avoid false positives
- Implements cooldown to prevent alert spam
- Classifies severity based on intensity

**State Machine:**
```
IDLE → (audio > threshold) → DETECTING
DETECTING → (sustained for confirmation_time) → ALERT + COOLDOWN
DETECTING → (audio drops) → IDLE
COOLDOWN → (cooldown_time elapsed) → IDLE
```

**Key Features:**
- Hysteresis to prevent flickering
- Configurable thresholds
- Severity classification (low/medium/high/critical)
- Unique alert IDs

**Topics:**
- Subscribes: `/simulated_audio_level`
- Publishes: `/whistle_alert` (whistle_safety_msgs/WhistleAlert)

**Parameters:**
- `detection_threshold`: Level to trigger detection (default: 0.7)
- `confirmation_time`: Seconds to confirm whistle (default: 0.5)
- `cooldown_time`: Seconds between alerts (default: 3.0)

**Files:**
- `whistle_detector/whistle_detector_node.py` - Detection logic
- `whistle_detector/autonomous_responder.py` - Navigation controller

---

### 3. **autonomous_responder** 🤖
**Purpose**: Navigates robot toward sound source with adaptive speed and obstacle avoidance

**What it does:**
- Calculates direction to sound source
- Adjusts speed based on sound intensity:
  - **< 0.15**: STOP (no sound)
  - **0.15-0.3**: VERY SLOW (🐢 0.1 m/s)
  - **0.3-0.6**: SLOW (🚶 0.3 m/s)
  - **> 0.6**: FAST (🏃 0.5 m/s)
  - **> 0.8**: STOP (✋ arrived)
- Avoids obstacles using lidar data
- Smooth exponential moving average filtering

**Navigation Strategy:**
1. Calculate bearing to target
2. Check for obstacles in path
3. If obstacle: take evasive action (turn left/right)
4. If clear: proceed with speed based on audio level
5. Stop when very close to source

**Key Features:**
- 3-tier adaptive speed control
- Obstacle avoidance with lidar
- Audio signal smoothing (EMA filter)
- Graceful handling of signal loss

**Topics:**
- Subscribes: `/simulated_audio_level`, `/odom`, `/scan`
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

**Parameters:**
- `activation_threshold`: Minimum to start moving (default: 0.15)
- `weak_threshold`: Slow mode threshold (default: 0.3)
- `moderate_threshold`: Normal speed threshold (default: 0.6)
- `stop_threshold`: Arrival threshold (default: 0.8)
- `weak_speed`: Speed for weak signal (default: 0.1 m/s)
- `moderate_speed`: Speed for moderate signal (default: 0.3 m/s)
- `strong_speed`: Speed for strong signal (default: 0.5 m/s)
- `sound_x`, `sound_y`: Target coordinates

**Files:**
- `whistle_detector/autonomous_responder.py`

---

### 4. **alert_manager** 📋
**Purpose**: Enriches and manages whistle alerts for cloud transmission

**What it does:**
- Receives whistle alerts from detector
- Enriches with robot position from odometry
- Adds timestamps and unique IDs
- Formats alerts for cloud transmission (JSON)
- Creates RViz visualization markers
- Maintains alert history

**Alert Data Structure:**
```json
{
  "alert_id": "a3f5c2d1",
  "timestamp": "2025-11-24T10:15:30.123456",
  "type": "whistle_detection",
  "severity": "high",
  "intensity": 0.85,
  "robot_position": {"x": 2.5, "y": 2.8, "z": 0.2},
  "sensor_position": {"x": 2.5, "y": 2.8, "z": 0.4},
  "robot_id": "whistle_robot_01",
  "location_name": "Simulation Environment"
}
```

**Key Features:**
- Position tagging
- Severity classification
- JSON formatting
- RViz markers for visualization
- Alert history logging

**Topics:**
- Subscribes: `/whistle_alert`, `/odom`
- Publishes: `/alert_to_cloud` (std_msgs/String), `/alert_markers` (MarkerArray)

**Files:**
- `alert_manager/alert_manager_node.py`

---

### 5. **zenoh_bridge** 🌐
**Purpose**: Bridges ROS 2 topics to Zenoh network for cloud communication

**What it does:**
- Subscribes to `/alert_to_cloud` topic
- Translates ROS 2 messages to Zenoh format
- Publishes to Zenoh network (topic: `cloud/alerts/whistle`)
- Handles connection to Zenoh peers or routers
- Works in simulation mode if Zenoh not installed

**Operating Modes:**
1. **Peer Mode** (default): Direct peer-to-peer communication
2. **Client Mode**: Connects to Zenoh router/server

**Key Features:**
- Graceful degradation (works without Zenoh)
- Configurable endpoints
- JSON message format
- Alert counting and logging

**Topics:**
- Subscribes: `/alert_to_cloud`
- Publishes to Zenoh: `cloud/alerts/whistle`

**Parameters:**
- `zenoh_mode`: "peer" or "client" (default: "peer")
- `zenoh_topic`: Zenoh topic name (default: "cloud/alerts/whistle")
- `connect_endpoint`: Optional endpoint (e.g., "tcp/192.168.1.100:7447")

**Files:**
- `zenoh_bridge/zenoh_bridge_node.py`

---

### 6. **whistle_safety_msgs** 📨
**Purpose**: Custom ROS 2 message definitions

**Messages:**

**AudioLevel.msg:**
```
float32 level              # Audio intensity (0.0-1.0)
geometry_msgs/Point sensor_position  # Microphone position
```

**WhistleAlert.msg:**
```
std_msgs/Header header     # Timestamp and frame
string alert_id            # Unique identifier
float32 intensity          # Audio level when detected
string severity            # low/medium/high/critical
geometry_msgs/Point position  # Alert location
```

**Files:**
- `whistle_safety_msgs/msg/AudioLevel.msg`
- `whistle_safety_msgs/msg/WhistleAlert.msg`

---

### 7. **whistle_gazebo** 🎮
**Purpose**: Gazebo simulation environment

**What it provides:**
- Realistic 3D world with grass terrain
- Sound source marker (yellow glowing sphere)
- Realistic obstacles (trees, rocks, bench)
- Proper lighting and shadows
- Physics simulation

**World Features:**
- Ground plane with grass texture
- Directional sunlight
- Atmospheric fog for depth
- Static obstacles for navigation testing
- Visual whistle source marker

**Files:**
- `worlds/whistle_test.world` - Gazebo world definition
- `launch/gazebo.launch.py` - Gazebo launcher
- `launch/system_with_gazebo.launch.py` - Complete system
- `scripts/demo_microphone.py` - Demo helper script

---

### 8. **whistle_robot_description** 🚗
**Purpose**: Robot URDF model and description

**Robot Components:**
- **Base**: Differential drive platform
- **Wheels**: Two driven wheels + caster
- **Lidar**: 360° laser scanner for obstacle detection
- **Microphone Link**: Virtual audio sensor mount

**Sensors:**
- Lidar: 360° scan, 10m range, 360 samples
- Odometry: Position and velocity feedback
- Microphone: Audio level sensing

**Files:**
- `urdf/whistle_robot.urdf.xacro` - Robot description

---

## ⚙️ How It Works

### Complete Flow:

```
1. SOUND SOURCE ACTIVATED
   └─> Microphone simulator calculates audio level based on distance
   
2. AUDIO LEVEL PUBLISHED
   └─> Topic: /simulated_audio_level (10 Hz)
   
3. WHISTLE DETECTOR PROCESSES SIGNAL
   ├─> Checks if level > detection_threshold (0.7)
   ├─> Confirms sustained signal (0.5 seconds)
   ├─> Classifies severity
   └─> Publishes alert to /whistle_alert
   
4. TWO PARALLEL PATHS:
   
   PATH A: AUTONOMOUS RESPONSE
   ├─> Autonomous responder receives audio level
   ├─> Calculates direction to sound source
   ├─> Checks for obstacles with lidar
   ├─> Determines speed based on audio intensity:
   │   ├─> Weak (0.15-0.3): 0.1 m/s (VERY SLOW 🐢)
   │   ├─> Moderate (0.3-0.6): 0.3 m/s (SLOW 🚶)
   │   ├─> Strong (>0.6): 0.5 m/s (FAST 🏃)
   │   └─> Very strong (>0.8): STOP (ARRIVED ✋)
   ├─> Avoids obstacles if detected
   └─> Publishes velocity commands to /cmd_vel
   
   PATH B: ALERT PIPELINE
   ├─> Alert manager receives whistle alert
   ├─> Enriches with robot position from /odom
   ├─> Adds timestamp, severity, unique ID
   ├─> Formats as JSON
   ├─> Publishes to /alert_to_cloud
   ├─> Creates RViz visualization markers
   │
   └─> Zenoh bridge receives cloud alert
       ├─> Translates to Zenoh format
       └─> Publishes to Zenoh network
           └─> Cloud systems receive alert:
               ├─> Dashboard displays real-time alert
               ├─> Analytics engine logs incident
               ├─> Mobile app sends notification
               └─> Other robots are informed

5. ROBOT REACHES TARGET
   ├─> Audio level > 0.8 (stop threshold)
   ├─> Robot stops at source location
   └─> Mission complete!
```

### Obstacle Avoidance Logic:

```
While navigating toward sound:
  
  IF obstacle detected in front:
    └─> Turn to avoid (choose least obstructed direction)
        ├─> If right is clear: Turn right
        └─> If left is clear: Turn left
  
  ELSE IF obstacle on sides only:
    └─> Slow down and proceed cautiously
  
  ELSE (no obstacles):
    └─> Proceed at speed based on audio level
```

---

## 📦 Installation & Setup

### Prerequisites:
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11
- Python 3.10+

### 1. Clone and Build:

```bash
# Navigate to workspace
cd ~/whistle_safety_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Install Zenoh (Optional but Recommended):

```bash
# For system Python
pip3 install eclipse-zenoh

# OR for conda/venv
pip install eclipse-zenoh
```

**Note:** System works without Zenoh, but cloud features will be in simulation mode.

### 3. Verify Installation:

```bash
# Check packages
ros2 pkg list | grep whistle

# Should show:
# whistle_detector
# whistle_gazebo
# whistle_safety_msgs
# alert_manager
# sim_microphone_sensor
# zenoh_bridge
```

---

## 🚀 Running the System

### Option 1: Interactive Demo (Easiest)

```bash
cd ~/whistle_safety_ws
source install/setup.bash
./demo.sh
```

Select from menu:
1. Default settings (sound at 3,3)
2. Custom position
3. Cautious mode (slow)
4. Aggressive mode (fast)

### Option 2: Quick Launch

```bash
cd ~/whistle_safety_ws
./run_adaptive_follower.sh
```

### Option 3: Manual Launch

```bash
cd ~/whistle_safety_ws
source install/setup.bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py
```

### With Custom Parameters:

```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    sound_x:=5.0 \
    sound_y:=2.0 \
    weak_speed:=0.1 \
    moderate_speed:=0.3 \
    strong_speed:=0.6
```

### Control Sound While Running:

Open a **new terminal**:

```bash
# Enable whistle (moderate intensity)
ros2 param set /microphone_simulator whistle_intensity 0.6

# Very weak sound (robot crawls)
ros2 param set /microphone_simulator whistle_intensity 0.2

# Strong sound (robot speeds up)
ros2 param set /microphone_simulator whistle_intensity 0.8

# Turn off (robot stops)
ros2 param set /microphone_simulator whistle_intensity 0.0
```

---

## 📁 File Structure

```
whistle_safety_ws/
├── src/
│   ├── sim_microphone_sensor/           # Audio sensor simulation
│   │   ├── sim_microphone_sensor/
│   │   │   └── microphone_simulator.py  # Distance-based audio calculation
│   │   ├── launch/
│   │   │   └── microphone_simulator.launch.py
│   │   ├── config/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── whistle_detector/                # Whistle detection & navigation
│   │   ├── whistle_detector/
│   │   │   ├── whistle_detector_node.py      # Pattern detection
│   │   │   └── autonomous_responder.py        # Adaptive navigation
│   │   ├── launch/
│   │   │   ├── whistle_detector.launch.py
│   │   │   └── adaptive_sound_follower.launch.py  # Main launch file
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── alert_manager/                   # Alert processing & enrichment
│   │   ├── alert_manager/
│   │   │   └── alert_manager_node.py    # Alert enrichment & formatting
│   │   ├── launch/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── zenoh_bridge/                    # Cloud communication bridge
│   │   ├── zenoh_bridge/
│   │   │   └── zenoh_bridge_node.py     # ROS 2 to Zenoh bridge
│   │   ├── launch/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── whistle_safety_msgs/             # Custom message definitions
│   │   ├── msg/
│   │   │   ├── AudioLevel.msg           # Audio sensor data
│   │   │   └── WhistleAlert.msg         # Alert message
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── whistle_gazebo/                  # Simulation environment
│   │   ├── worlds/
│   │   │   └── whistle_test.world       # Gazebo world with obstacles
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py
│   │   │   └── system_with_gazebo.launch.py
│   │   ├── scripts/
│   │   │   └── demo_microphone.py       # Demo helper
│   │   ├── models/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── whistle_robot_description/       # Robot model
│       ├── urdf/
│       │   └── whistle_robot.urdf.xacro # Robot URDF with sensors
│       ├── CMakeLists.txt
│       └── package.xml
│
├── build/                               # Build artifacts
├── install/                             # Installed packages
├── log/                                 # Build and runtime logs
│
├── demo.sh                              # Interactive demo launcher
├── run_adaptive_follower.sh             # Quick launch script
├── test_position_1_3.sh                 # Test script
│
├── README.md                            # This file - Complete documentation
├── README_ADAPTIVE_FOLLOWER.md          # Quick start guide
├── RUN_INSTRUCTIONS.md                  # Detailed run instructions
├── PARAMETER_CHEATSHEET.md              # Parameter reference
├── QUICK_START.txt                      # Visual quick reference
├── SYSTEM_SUMMARY.txt                   # System overview
├── FIXED_READY_TO_RUN.txt               # Recent fixes
└── COMPLETE_README.md                   # This comprehensive guide
```

---

## ⚙️ Configuration Parameters

### System-Wide Parameters:

#### Microphone Simulator:
```yaml
publish_rate: 10.0           # Update frequency (Hz)
base_noise_level: 0.1        # Background noise level
whistle_intensity: 0.0       # Manual whistle control (0.0-1.0)
sound_source_active: false   # Auto-enable sound at position
```

#### Whistle Detector:
```yaml
detection_threshold: 0.7     # Level to trigger detection
confirmation_time: 0.5       # Seconds to confirm whistle
cooldown_time: 3.0          # Seconds between alerts
```

#### Autonomous Responder:
```yaml
# Thresholds
activation_threshold: 0.15   # Minimum to start moving
weak_threshold: 0.3         # Below = very slow
moderate_threshold: 0.6     # Above = normal speed
stop_threshold: 0.8         # Stop when reached

# Speeds
weak_speed: 0.1             # m/s for weak signal
moderate_speed: 0.3         # m/s for moderate signal
strong_speed: 0.5           # m/s for strong signal

# Target
sound_x: 3.0                # X coordinate
sound_y: 3.0                # Y coordinate

# Filtering
audio_alpha: 0.6            # EMA smoothing (0-1)
stop_delay: 0.5             # Seconds before stopping
```

#### Zenoh Bridge:
```yaml
zenoh_mode: "peer"          # "peer" or "client"
zenoh_topic: "cloud/alerts/whistle"
connect_endpoint: ""        # Optional: "tcp/IP:PORT"
```

---

## 🐛 Troubleshooting

### Issue: Robot Not Moving

**Symptoms:** Robot spawns but doesn't move toward sound

**Solutions:**
1. Enable whistle:
   ```bash
   ros2 param set /microphone_simulator whistle_intensity 0.6
   ```

2. Check if autonomous responder is running:
   ```bash
   ros2 node list | grep autonomous
   ```

3. Check topics:
   ```bash
   ros2 topic echo /simulated_audio_level
   ros2 topic echo /cmd_vel
   ```

---

### Issue: "Integer/Double Type Mismatch"

**Symptoms:** Error about parameter type when launching with custom coordinates

**Solution:** Always use decimal points for coordinates:
```bash
# ✅ CORRECT
ros2 launch whistle_detector adaptive_sound_follower.launch.py sound_x:=1.0 sound_y:=3.0

# ❌ WRONG
ros2 launch whistle_detector adaptive_sound_follower.launch.py sound_x:=1 sound_y:=3
```

---

### Issue: "demo_microphone.py Not Found"

**Symptoms:** Error about missing demo_microphone.py script

**Solution:** Rebuild whistle_gazebo package:
```bash
cd ~/whistle_safety_ws
colcon build --packages-select whistle_gazebo
source install/setup.bash
```

---

### Issue: Zenoh Warnings

**Symptoms:** Warnings about Zenoh not installed

**This is NORMAL** - System works in simulation mode without Zenoh.

**To enable cloud features:**
```bash
pip3 install eclipse-zenoh
```

Then restart the launch file.

---

### Issue: Robot Hits Obstacles

**Symptoms:** Robot collides with obstacles instead of avoiding them

**Solutions:**
1. Check lidar data:
   ```bash
   ros2 topic echo /scan
   ```

2. Reduce speeds for more cautious navigation:
   ```bash
   ros2 param set /autonomous_responder weak_speed 0.05
   ros2 param set /autonomous_responder moderate_speed 0.15
   ros2 param set /autonomous_responder strong_speed 0.25
   ```

3. Check obstacle detection logs in terminal output

---

### Issue: Too Many False Alerts

**Symptoms:** Whistle detector triggers on background noise

**Solutions:**
1. Increase detection threshold:
   ```bash
   ros2 param set /whistle_detector_node detection_threshold 0.8
   ```

2. Increase confirmation time:
   ```bash
   ros2 param set /whistle_detector_node confirmation_time 1.0
   ```

---

### Issue: Robot Moves Too Fast/Slow

**Solution:** Adjust speed parameters in real-time:
```bash
# Make slower
ros2 param set /autonomous_responder weak_speed 0.05
ros2 param set /autonomous_responder moderate_speed 0.2
ros2 param set /autonomous_responder strong_speed 0.3

# Make faster
ros2 param set /autonomous_responder weak_speed 0.15
ros2 param set /autonomous_responder moderate_speed 0.4
ros2 param set /autonomous_responder strong_speed 0.7
```

---

### Issue: Gazebo Won't Start

**Solutions:**
1. Kill existing Gazebo processes:
   ```bash
   killall gzserver gzclient
   ```

2. Check if port is in use:
   ```bash
   netstat -tulpn | grep 11345
   ```

3. Source workspace:
   ```bash
   source ~/whistle_safety_ws/install/setup.bash
   ```

---

### Issue: Build Failures

**Common causes and fixes:**

1. **Missing dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros-pkgs
   sudo apt install ros-humble-robot-state-publisher
   ```

2. **Package not found:**
   ```bash
   # Clean build
   rm -rf build/ install/ log/
   colcon build
   source install/setup.bash
   ```

3. **Python import errors:**
   ```bash
   pip3 install numpy
   ```

---

## 📊 Expected Behavior

### Startup Sequence:
```
1. Gazebo opens with robot at (0, 0)
2. Yellow glowing sphere appears at sound source
3. All nodes start and print status messages
4. Robot awaits whistle signal
```

### When Whistle Enabled:
```
1. Microphone simulator publishes audio levels
2. If level > 0.15: Robot starts moving
3. Speed adapts based on audio strength:
   - 0.15-0.3: VERY SLOW (🐢)
   - 0.3-0.6: SLOW (🚶)
   - >0.6: FAST (🏃)
4. If obstacle detected: Robot avoids it
5. When level > 0.8: Robot stops (arrived)
```

### Terminal Output:
```
🤖 AUTONOMOUS SOUND FOLLOWER ACTIVATED
Target sound source: (3.0, 3.0)
Behavior based on sound strength:
  🛑 < 0.15: STOP (no sound)
  🐢 0.15-0.30: VERY SLOW
  🚶 0.30-0.60: SLOW
  🏃 > 0.60: FAST
  ✋ > 0.80: STOP (arrived)

🚗 Sound detected! Starting movement...
🐢 VERY SLOW | 📍 Distance: 4.24m | Audio: 0.25
🚶 SLOW | 📍 Distance: 3.15m | Audio: 0.48
🏃 FAST | 📍 Distance: 1.50m | Audio: 0.72
🎉 ARRIVED AT SOUND SOURCE!
```

---

## 🎓 Learning Resources

### Understanding the Code:
1. Start with `microphone_simulator.py` - Simple distance calculation
2. Read `whistle_detector_node.py` - State machine pattern
3. Study `autonomous_responder.py` - Navigation logic
4. Explore `alert_manager_node.py` - Data enrichment
5. Finally `zenoh_bridge_node.py` - Cloud integration

### ROS 2 Concepts Used:
- Publishers and Subscribers
- Custom messages
- Parameters and dynamic reconfiguration
- Launch files
- Coordinate transforms (odometry)

### Advanced Topics:
- Sensor fusion (lidar + audio)
- State machines for detection
- PID-like control for navigation
- Exponential moving average filtering
- Obstacle avoidance algorithms

---

## 🤝 Contributing

This is an educational and demonstration project. Feel free to:
- Add more sophisticated obstacle avoidance (e.g., Nav2 integration)
- Implement machine learning for whistle detection
- Add camera-based person detection
- Integrate with real Zenoh cloud infrastructure
- Create dashboards for alert visualization

---

## 📝 License

This project is for educational purposes. Components:
- ROS 2: Apache 2.0
- Gazebo: Apache 2.0
- Zenoh: Apache 2.0 / Eclipse Public License 2.0

---

## 🙏 Acknowledgments

- ROS 2 Community
- Gazebo Simulation Team
- Eclipse Zenoh Project
- Open Robotics

---

## 📧 Support

For issues:
1. Check this README's troubleshooting section
2. Review other documentation files:
   - `RUN_INSTRUCTIONS.md`
   - `PARAMETER_CHEATSHEET.md`
   - `QUICK_START.txt`
3. Check ROS 2 and Gazebo logs in `log/` directory

---

**🎉 You now have a complete understanding of the Whistle Safety & Emergency Response System!**

Start with `./demo.sh` and experiment with different parameters to see how the robot adapts its behavior!
