# 🤖 Adaptive Sound Follower System

## 🎯 What This Does

Your robot now has **adaptive behavior** based on sound strength:

- **🛑 No Sound (< 0.15)**: Robot STOPS
- **🐢 Weak Sound (0.15-0.3)**: Robot moves VERY SLOWLY  
- **🚶 Moderate Sound (0.3-0.6)**: Robot moves SLOWLY
- **🏃 Strong Sound (> 0.6)**: Robot moves FAST
- **✋ Very Strong (> 0.8)**: Robot STOPS (arrived at source)

---

## 🚀 QUICK START (Easiest Way)

Just run this one command:

```bash
./demo.sh
```

Then select option 1 for default settings!

---

## 📚 Documentation Files

- **`RUN_INSTRUCTIONS.md`** - Complete guide with all options
- **`PARAMETER_CHEATSHEET.md`** - Quick reference for parameters
- **`demo.sh`** - Interactive menu to launch different modes
- **`run_adaptive_follower.sh`** - Quick launch with defaults

---

## 🎬 Step-by-Step First Run

1. **Build the workspace:**
   ```bash
   cd ~/whistle_safety_ws
   colcon build
   source install/setup.bash
   ```

2. **Run the demo:**
   ```bash
   ./demo.sh
   ```
   - Select option 1 (default)
   - Gazebo will open
   - Robot will automatically move toward the light/sound source

3. **Watch the behavior:**
   - Robot starts slowly (weak sound detected)
   - Speeds up as it gets closer (stronger sound)
   - Stops when it reaches the target

4. **Stop the system:**
   - Press `Ctrl+C` in the terminal

---

## 🎛️ Quick Parameter Changes While Running

**In a NEW terminal**, after launching:

```bash
# Control sound intensity (makes robot go faster/slower)
ros2 param set /microphone_simulator whistle_intensity 0.2  # Very weak
ros2 param set /microphone_simulator whistle_intensity 0.5  # Moderate
ros2 param set /microphone_simulator whistle_intensity 0.8  # Strong

# Adjust robot speeds
ros2 param set /autonomous_responder weak_speed 0.1
ros2 param set /autonomous_responder moderate_speed 0.3
ros2 param set /autonomous_responder strong_speed 0.5
```

---

## 🗂️ Key Files

### Modified:
- `src/whistle_detector/whistle_detector/autonomous_responder.py`
  - Added 3-tier speed control (weak/moderate/strong)
  - Automatic stopping when sound is very weak or very strong
  - Better status logging

### Created:
- `src/whistle_detector/launch/adaptive_sound_follower.launch.py`
  - Complete launch file with all components
  - Configurable parameters for thresholds and speeds

---

## 💡 Example Scenarios

### Scenario 1: Default (Balanced)
```bash
./demo.sh   # Select option 1
```

### Scenario 2: Cautious Robot (Slow and steady)
```bash
./demo.sh   # Select option 3
```

### Scenario 3: Fast Response
```bash
./demo.sh   # Select option 4
```

### Scenario 4: Custom Position
```bash
./demo.sh   # Select option 2
# Enter X: 5.0
# Enter Y: 2.0
```

---

## 🆘 Troubleshooting

**Robot not moving?**
- Check Gazebo is running
- In new terminal: `ros2 param set /microphone_simulator whistle_intensity 0.6`

**Robot too fast/slow?**
- See `PARAMETER_CHEATSHEET.md` for quick fixes

**Need detailed help?**
- Read `RUN_INSTRUCTIONS.md` for complete documentation

---

## ✅ What You Should See

1. **Gazebo opens** with robot at origin (0, 0)
2. **Light/marker** shows at sound source location (default: 3, 3)
3. **Terminal shows**:
   ```
   🤖 AUTONOMOUS SOUND FOLLOWER ACTIVATED
   Behavior based on sound strength:
     🛑 < 0.15: STOP (no sound)
     🐢 0.15-0.30: VERY SLOW
     🚶 0.30-0.60: SLOW
     🏃 > 0.60: FAST
     ✋ > 0.80: STOP (arrived)
   ```
4. **Robot moves** with increasing speed as it approaches
5. **Robot stops** when it reaches the target

---

## 🎓 Learn More

- **Full documentation**: Open `RUN_INSTRUCTIONS.md`
- **Parameter reference**: Open `PARAMETER_CHEATSHEET.md`
- **Interactive demo**: Run `./demo.sh`

---

**Enjoy your adaptive sound-following robot!** 🎉
