#!/bin/bash
# Quick test script with RViz visualization

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  🎨 RViz Visualization Test                               ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "This will launch:"
echo "  ✅ Gazebo (physics simulation)"
echo "  ✅ RViz (visualization) - NEW!"
echo "  ✅ All system nodes"
echo ""
echo "In RViz you'll see:"
echo "  🤖 Robot model"
echo "  📡 Lidar scan (red dots)"
echo "  🎯 Pulsing yellow target sphere"
echo "  🚨 Alert markers (when detected)"
echo ""
echo "After launch, enable sound with:"
echo "  ros2 param set /microphone_simulator whistle_intensity 0.6"
echo ""
echo "Starting in 3 seconds..."
sleep 3

cd ~/whistle_safety_ws
source install/setup.bash
ros2 launch whistle_detector adaptive_sound_follower.launch.py
