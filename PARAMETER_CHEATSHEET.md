# 🎛️ Parameter Adjustment Cheat Sheet

## Quick Launch Commands

### 1️⃣ DEFAULT (Balanced behavior)
```bash
./run_adaptive_follower.sh
```

### 2️⃣ CAUTIOUS MODE (Slow and steady)
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    weak_speed:=0.05 \
    moderate_speed:=0.15 \
    strong_speed:=0.25
```

### 3️⃣ AGGRESSIVE MODE (Fast response)
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    weak_speed:=0.2 \
    moderate_speed:=0.5 \
    strong_speed:=0.8
```

### 4️⃣ PRECISION MODE (Very sensitive to sound changes)
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    activation_threshold:=0.1 \
    weak_threshold:=0.25 \
    moderate_threshold:=0.5 \
    stop_threshold:=0.75
```

### 5️⃣ DISTANT TARGET
```bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py \
    sound_x:=10.0 \
    sound_y:=10.0
```

---

## Runtime Parameter Changes

Open a NEW terminal after launching, then run these commands:

### 🎚️ Adjust Sound Intensity (Simulated Whistle)
```bash
# No sound - robot stops
ros2 param set /microphone_simulator whistle_intensity 0.0

# Very weak - robot crawls (🐢)
ros2 param set /microphone_simulator whistle_intensity 0.2

# Weak - robot moves slowly (🚶)
ros2 param set /microphone_simulator whistle_intensity 0.4

# Moderate - robot moves normally (🏃)
ros2 param set /microphone_simulator whistle_intensity 0.6

# Strong - robot moves fast (🏃)
ros2 param set /microphone_simulator whistle_intensity 0.8

# Very strong - robot stops (arrived) (✋)
ros2 param set /microphone_simulator whistle_intensity 0.9
```

### 🏎️ Adjust Robot Speeds
```bash
# Make robot slower overall
ros2 param set /autonomous_responder weak_speed 0.05
ros2 param set /autonomous_responder moderate_speed 0.15
ros2 param set /autonomous_responder strong_speed 0.25

# Make robot faster overall
ros2 param set /autonomous_responder weak_speed 0.2
ros2 param set /autonomous_responder moderate_speed 0.5
ros2 param set /autonomous_responder strong_speed 0.8
```

### 🎯 Adjust Behavior Thresholds
```bash
# More sensitive (reacts to weaker sounds)
ros2 param set /autonomous_responder activation_threshold 0.1
ros2 param set /autonomous_responder weak_threshold 0.2
ros2 param set /autonomous_responder moderate_threshold 0.4

# Less sensitive (needs stronger sounds)
ros2 param set /autonomous_responder activation_threshold 0.2
ros2 param set /autonomous_responder weak_threshold 0.4
ros2 param set /autonomous_responder moderate_threshold 0.7

# Stop closer to target
ros2 param set /autonomous_responder stop_threshold 0.9

# Stop farther from target
ros2 param set /autonomous_responder stop_threshold 0.7
```

---

## 📊 Monitoring Commands

### View Current Audio Level
```bash
ros2 topic echo /simulated_audio_level
```

### View Robot Velocity Commands
```bash
ros2 topic echo /cmd_vel
```

### View Robot Position
```bash
ros2 topic echo /odom
```

### View All Parameters
```bash
ros2 param list /autonomous_responder
ros2 param dump /autonomous_responder
```

---

## 🧪 Test Scenarios

### Test 1: Gradual Approach
1. Launch system
2. In new terminal: `ros2 param set /microphone_simulator whistle_intensity 0.2`
3. Watch robot move very slowly
4. Gradually increase: `ros2 param set /microphone_simulator whistle_intensity 0.4`
5. Continue increasing: `ros2 param set /microphone_simulator whistle_intensity 0.7`
6. Robot should speed up as sound gets stronger

### Test 2: Stop and Go
1. Launch system
2. Enable sound: `ros2 param set /microphone_simulator whistle_intensity 0.5`
3. Robot moves
4. Disable sound: `ros2 param set /microphone_simulator whistle_intensity 0.0`
5. Robot should stop
6. Re-enable: `ros2 param set /microphone_simulator whistle_intensity 0.5`
7. Robot should resume

### Test 3: Speed Comparison
1. Launch with default settings
2. Let robot approach target
3. Note the behavior
4. Stop system (Ctrl+C)
5. Launch again with aggressive mode
6. Compare the difference in speed and behavior

---

## 🎬 Complete Example Session

```bash
# Terminal 1: Launch the system
cd ~/whistle_safety_ws
./run_adaptive_follower.sh

# Terminal 2: Control the sound (after system starts)
# Start with weak sound
ros2 param set /microphone_simulator whistle_intensity 0.3

# Wait 5 seconds, increase sound
ros2 param set /microphone_simulator whistle_intensity 0.5

# Wait 5 seconds, strong sound
ros2 param set /microphone_simulator whistle_intensity 0.8

# Robot should arrive and stop

# Test stop and go
ros2 param set /microphone_simulator whistle_intensity 0.0  # Stop
sleep 3
ros2 param set /microphone_simulator whistle_intensity 0.6  # Resume
```

---

## 💡 Pro Tips

1. **Smooth transitions**: Change intensity gradually (0.1 increments) for smooth robot behavior
2. **Testing positions**: Start with close targets (3,3) then try far ones (10,10)
3. **Speed tuning**: Start slow, then increase - easier to control
4. **Threshold tuning**: Keep 0.1-0.15 gap between thresholds for clear behavior zones
5. **Real-time adjustment**: All parameters can be changed while running!

---

## 🆘 Quick Fixes

**Robot not moving?**
```bash
ros2 param set /microphone_simulator whistle_intensity 0.6
```

**Robot too fast?**
```bash
ros2 param set /autonomous_responder strong_speed 0.3
ros2 param set /autonomous_responder moderate_speed 0.2
ros2 param set /autonomous_responder weak_speed 0.1
```

**Robot too slow?**
```bash
ros2 param set /autonomous_responder strong_speed 0.8
ros2 param set /autonomous_responder moderate_speed 0.5
ros2 param set /autonomous_responder weak_speed 0.2
```

**Robot stops too early?**
```bash
ros2 param set /autonomous_responder stop_threshold 0.9
```

**Robot doesn't respond to weak sounds?**
```bash
ros2 param set /autonomous_responder activation_threshold 0.1
```
