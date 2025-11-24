#!/bin/bash
# Complete demonstration of adaptive sound follower

echo "=================================================="
echo "🎯 ADAPTIVE SOUND FOLLOWER DEMO"
echo "=================================================="
echo ""
echo "This script will demonstrate the robot's adaptive behavior"
echo "based on sound strength."
echo ""

# Check if system is already running
if pgrep -f "adaptive_sound_follower.launch.py" > /dev/null; then
    echo "⚠️  System appears to be already running!"
    echo "Please close it first (Ctrl+C in that terminal)"
    exit 1
fi

echo "What would you like to do?"
echo ""
echo "1) 🚀 RUN WITH DEFAULT SETTINGS (Sound at 3.0, 3.0)"
echo "2) 🎯 RUN WITH CUSTOM POSITION"
echo "3) 🐢 RUN IN CAUTIOUS MODE (Slow speeds)"
echo "4) 🏃 RUN IN AGGRESSIVE MODE (Fast speeds)"
echo "5) 📖 SHOW PARAMETER CHEATSHEET"
echo "6) ❌ EXIT"
echo ""
read -p "Enter choice (1-6): " choice

cd ~/whistle_safety_ws
source install/setup.bash

case $choice in
    1)
        echo ""
        echo "🚀 Launching with DEFAULT settings..."
        echo "   Sound position: (3.0, 3.0)"
        echo "   Speeds: Weak=0.1, Moderate=0.3, Strong=0.5 m/s"
        echo ""
        ros2 launch whistle_detector adaptive_sound_follower.launch.py
        ;;
    2)
        echo ""
        read -p "Enter X coordinate: " x_pos
        read -p "Enter Y coordinate: " y_pos
        # Ensure values are floats by appending .0 if they're integers
        [[ ! "$x_pos" =~ \. ]] && x_pos="${x_pos}.0"
        [[ ! "$y_pos" =~ \. ]] && y_pos="${y_pos}.0"
        echo ""
        echo "🎯 Launching with custom position ($x_pos, $y_pos)..."
        echo ""
        ros2 launch whistle_detector adaptive_sound_follower.launch.py \
            sound_x:=$x_pos sound_y:=$y_pos
        ;;
    3)
        echo ""
        echo "🐢 Launching in CAUTIOUS mode..."
        echo "   Speeds: Weak=0.05, Moderate=0.15, Strong=0.25 m/s"
        echo ""
        ros2 launch whistle_detector adaptive_sound_follower.launch.py \
            weak_speed:=0.05 moderate_speed:=0.15 strong_speed:=0.25
        ;;
    4)
        echo ""
        echo "🏃 Launching in AGGRESSIVE mode..."
        echo "   Speeds: Weak=0.2, Moderate=0.5, Strong=0.8 m/s"
        echo ""
        ros2 launch whistle_detector adaptive_sound_follower.launch.py \
            weak_speed:=0.2 moderate_speed:=0.5 strong_speed:=0.8
        ;;
    5)
        echo ""
        echo "📖 Opening Parameter Cheatsheet..."
        echo ""
        if command -v less &> /dev/null; then
            less ~/whistle_safety_ws/PARAMETER_CHEATSHEET.md
        else
            cat ~/whistle_safety_ws/PARAMETER_CHEATSHEET.md
        fi
        ;;
    6)
        echo ""
        echo "👋 Goodbye!"
        exit 0
        ;;
    *)
        echo ""
        echo "❌ Invalid choice. Please run again and select 1-6."
        exit 1
        ;;
esac
