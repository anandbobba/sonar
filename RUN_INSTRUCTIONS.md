# 🤖 Adaptive Sound Follower - Quick Start Guide

## Overview
The robot automatically moves toward a sound source with **adaptive speed** based on sound strength:

| Sound Level | Robot Behavior | Speed | Symbol |
|------------|----------------|-------|--------|
| < 0.15 | **STOP** - No sound detected | 0 m/s | 🛑 |
| 0.15 - 0.3 | **VERY SLOW** - Weak sound | 0.1 m/s | 🐢 |
| 0.3 - 0.6 | **SLOW** - Moderate sound | 0.3 m/s | 🚶 |
| > 0.6 | **FAST** - Strong sound | 0.5 m/s | 🏃 |
| > 0.8 | **STOP** - Arrived at source | 0 m/s | ✋ |

---

## 🚀 How to Run

### Step 1: Build the Workspace
```bash
cd ~/whistle_safety_ws
colcon build
source install/setup.bash
```

### Step 2: Launch the System

#### **Option A: Default Settings** (Sound at position 3.0, 3.0)
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py
```

#### **Option B: Custom Sound Position**
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py sound_x:=5.0 sound_y:=2.0
```

#### **Option C: Custom Speed Settings**
Make robot more cautious (slower speeds):
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    weak_speed:=0.05 \
    moderate_speed:=0.2 \
    strong_speed:=0.4
```

Make robot more aggressive (faster speeds):
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    weak_speed:=0.2 \
    moderate_speed:=0.5 \
    strong_speed:=0.8
```

#### **Option D: Custom Thresholds**
Adjust when robot changes behavior:
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    activation_threshold:=0.2 \
    weak_threshold:=0.4 \
    moderate_threshold:=0.7 \
    stop_threshold:=0.85
```

---

## 🎛️ Parameters Explained

### Position Parameters
- `sound_x`: X coordinate of sound source (default: 3.0)
- `sound_y`: Y coordinate of sound source (default: 3.0)

### Threshold Parameters (Sound Strength)
- `activation_threshold`: Minimum sound to start moving (default: 0.15)
- `weak_threshold`: Below this = very slow mode (default: 0.3)
- `moderate_threshold`: Above this = normal speed (default: 0.6)
- `stop_threshold`: Stop when sound this strong (arrived) (default: 0.8)

### Speed Parameters
- `weak_speed`: Speed for very weak sound (default: 0.1 m/s)
- `moderate_speed`: Speed for moderate sound (default: 0.3 m/s)
- `strong_speed`: Speed for strong sound (default: 0.5 m/s)

---

## 📊 What You'll See

### In Gazebo Window:
- Robot starting at position (0, 0)
- Light/marker at sound source position
- Robot automatically moving toward the light

### In Terminal Output:
```
🤖 AUTONOMOUS SOUND FOLLOWER ACTIVATED
================================================================
Target sound source: (3.0, 3.0)
Robot will automatically move toward sound
Behavior based on sound strength:
  🛑 < 0.15: STOP (no sound)
  🐢 0.15-0.30: VERY SLOW
  🚶 0.30-0.60: SLOW
  🏃 > 0.60: FAST
  ✋ > 0.80: STOP (arrived)
================================================================

🚗 Sound detected! Starting movement...
🐢 VERY SLOW | 📍 Distance: 4.24m | Audio: 0.25 | Speed: 0.10 m/s
🚶 SLOW | 📍 Distance: 3.52m | Audio: 0.45 | Speed: 0.30 m/s
🏃 FAST | 📍 Distance: 1.82m | Audio: 0.68 | Speed: 0.50 m/s
🎉 ARRIVED AT SOUND SOURCE!
```

---

## 🔧 Advanced Usage

### Change Parameters While Running
You can adjust parameters in real-time using ROS 2 parameter commands:

```bash
# Make robot more cautious
ros2 param set /autonomous_responder weak_speed 0.05
ros2 param set /autonomous_responder moderate_speed 0.15
ros2 param set /autonomous_responder strong_speed 0.25

# Adjust thresholds
ros2 param set /autonomous_responder weak_threshold 0.4
ros2 param set /autonomous_responder moderate_threshold 0.7
```

### View Current Parameters
```bash
ros2 param list /autonomous_responder
ros2 param get /autonomous_responder weak_speed
```

### Monitor Audio Levels
```bash
ros2 topic echo /simulated_audio_level
```

### Control Virtual Whistle Intensity
```bash
# Weak sound (robot will move slowly)
ros2 param set /microphone_simulator whistle_intensity 0.4

# Moderate sound (robot will move at normal speed)
ros2 param set /microphone_simulator whistle_intensity 0.6

# Strong sound (robot will move fast)
ros2 param set /microphone_simulator whistle_intensity 0.8

# Turn off sound (robot will stop)
ros2 param set /microphone_simulator whistle_intensity 0.0
```

---

## 🧪 Testing Different Scenarios

### Scenario 1: Weak Signal Search
Robot moves very slowly, good for precise positioning:
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    weak_speed:=0.08 \
    moderate_speed:=0.15 \
    strong_speed:=0.25
```

### Scenario 2: Emergency Response
Robot moves fast when strong signal detected:
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    weak_speed:=0.15 \
    moderate_speed:=0.4 \
    strong_speed:=0.8
```

### Scenario 3: Far Away Target
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    sound_x:=10.0 \
    sound_y:=10.0
```

---

## 🛠️ Troubleshooting

### Robot Not Moving
1. Check if sound is enabled:
   ```bash
   ros2 param get /microphone_simulator whistle_intensity
   ```
2. Should be > 0.0. If not, enable it:
   ```bash
   ros2 param set /microphone_simulator whistle_intensity 0.6
   ```

### Robot Moving Too Fast/Slow
Adjust speed parameters:
```bash
ros2 param set /autonomous_responder weak_speed 0.2
ros2 param set /autonomous_responder moderate_speed 0.4
ros2 param set /autonomous_responder strong_speed 0.6
```

### Robot Stops Too Early
Increase stop threshold:
```bash
ros2 param set /autonomous_responder stop_threshold 0.9
```

---

## 📁 Files Modified/Created

### Modified:
- `src/whistle_detector/whistle_detector/autonomous_responder.py`
  - Added adaptive speed control based on sound strength
  - Three speed tiers: weak, moderate, strong
  - Better logging with behavior indicators

### Created:
- `src/whistle_detector/launch/adaptive_sound_follower.launch.py`
  - Complete launch file with all parameters
  - Includes Gazebo, robot, sensors, and controllers

- `RUN_INSTRUCTIONS.md` (this file)
  - Complete guide for running the system

---

## 🎯 Quick Command Reference

```bash
# Build and source
cd ~/whistle_safety_ws && colcon build && source install/setup.bash

# Run with defaults
ros2 launch whistle_detector adaptive_sound_follower.launch.py

# Run with custom position
ros2 launch whistle_detector adaptive_sound_follower.launch.py sound_x:=5.0 sound_y:=2.0

# Enable/adjust virtual whistle
ros2 param set /microphone_simulator whistle_intensity 0.6

# Monitor robot behavior
ros2 topic echo /cmd_vel
ros2 topic echo /simulated_audio_level

# Stop everything
Ctrl+C
```

---

## ✅ Expected Behavior Summary

1. **Launch** → Gazebo opens with robot at (0,0)
2. **Demo script** → Automatically enables virtual whistle at target position
3. **Robot detects** → Weak sound, starts moving VERY SLOWLY (🐢)
4. **Gets closer** → Sound gets stronger, moves SLOW (🚶)
5. **Near target** → Sound very strong, moves FAST (🏃)
6. **Arrives** → Sound > 0.8, STOPS (✋)

Enjoy your adaptive sound-following robot! 🎉
