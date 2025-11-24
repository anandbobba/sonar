#!/usr/bin/env python3
"""
Simple demo helper: set microphone_simulator params to enable virtual whistle at given coords
Usage:
  python3 demo_microphone.py <x> <y>

It uses the ROS2 CLI via subprocess to set params (keeps script simple and doesn't require package installs).
"""
import sys
import time
import subprocess


def set_param(node, name, value):
    cmd = ['ros2', 'param', 'set', node, name, str(value)]
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print('Failed to set param:', cmd, '->', e)


if __name__ == '__main__':
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    node = '/microphone_simulator'
    print(f'Setting virtual whistle to active at ({x},{y}) on {node}')
    set_param(node, 'sound_source_active', 'true')
    set_param(node, 'sound_source_x', x)
    set_param(node, 'sound_source_y', y)

    print('Demo helper configured. Press Ctrl-C to exit.')
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('Exiting demo helper, disabling virtual whistle')
        set_param(node, 'sound_source_active', 'false')
