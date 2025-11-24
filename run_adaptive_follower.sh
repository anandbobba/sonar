#!/bin/bash
# Quick start script for adaptive sound follower

echo "=================================================="
echo "🤖 Adaptive Sound Follower - Quick Start"
echo "=================================================="
echo ""
echo "Building workspace..."
cd ~/whistle_safety_ws
colcon build --packages-select whistle_detector

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    echo "Sourcing workspace..."
    source install/setup.bash
    
    echo ""
    echo "🚀 Launching adaptive sound follower..."
    echo ""
    echo "Robot behavior:"
    echo "  🛑 No sound (< 0.15): STOP"
    echo "  🐢 Weak sound (0.15-0.3): VERY SLOW"
    echo "  🚶 Moderate sound (0.3-0.6): SLOW"
    echo "  🏃 Strong sound (> 0.6): FAST"
    echo "  ✋ Very strong (> 0.8): STOP (arrived)"
    echo ""
    echo "Press Ctrl+C to stop"
    echo "=================================================="
    echo ""
    
    ros2 launch whistle_detector adaptive_sound_follower.launch.py
else
    echo "❌ Build failed! Please check errors above."
    exit 1
fi
