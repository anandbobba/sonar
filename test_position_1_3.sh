#!/bin/bash
# Quick test script - launches system with sound at position (1, 3)

echo "🚀 Launching Adaptive Sound Follower"
echo "   Sound position: (1.0, 3.0)"
echo "   Robot starts at: (0.0, 0.0)"
echo ""
echo "After Gazebo opens, run in a NEW terminal:"
echo "   ros2 param set /microphone_simulator whistle_intensity 0.6"
echo ""
echo "Starting in 3 seconds..."
sleep 3

cd ~/whistle_safety_ws
source install/setup.bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py sound_x:=1.0 sound_y:=3.0
