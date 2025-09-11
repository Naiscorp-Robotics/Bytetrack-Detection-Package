#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "ByteTrack Visualizer - Choose option:"
echo "1. Run ByteTrack visualizer (with tracking)"
echo "2. Run YOLOX visualizer (detection only)"
echo "3. Run with video save"
echo "4. Run with X11 forwarding (for SSH)"
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo "Starting ByteTrack visualizer..."
        python3 src/bytetrack_visualizer.py
        ;;
    2)
        echo "Starting YOLOX visualizer..."
        python3 src/yolox_visualizer.py
        ;;
    3)
        echo "Starting ByteTrack visualizer with video recording..."
        python3 src/bytetrack_visualizer.py --ros-args \
            -p save_video:=true \
            -p output_file:=tracking_$(date +%Y%m%d_%H%M%S).mp4
        ;;
    4)
        echo "Setting up X11 forwarding..."
        export DISPLAY=:0
        xhost +local:docker 2>/dev/null || xhost +local:root 2>/dev/null
        echo "DISPLAY set to: $DISPLAY"
        python3 src/bytetrack_visualizer.py
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac
