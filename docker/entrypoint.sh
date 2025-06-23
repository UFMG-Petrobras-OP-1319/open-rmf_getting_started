#!/bin/bash

# Set Python to unbuffered mode
export PYTHONUNBUFFERED=1

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Source workspace setup if it exists
if [ -f "/home/ros/ros2_ws/install/setup.bash" ]; then
    source /home/ros/ros2_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@" 