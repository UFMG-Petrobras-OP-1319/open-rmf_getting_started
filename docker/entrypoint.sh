#!/bin/bash

# Ensure .ros directory ownership is correct (Fixes PermissionError: [Errno 13])
# This must be run as root before dropping privileges, in case permissions
# were corrupted by volume mounting or host UID differences.
if [ -d "/home/ros/.ros" ]; then
    chown -R ros:ros /home/ros/.ros
fi

# Set Python to unbuffered mode
export PYTHONUNBUFFERED=1

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Source workspace setup if it exists
if [ -f "/home/ros/ros2_ws/install/setup.bash" ]; then
    source /home/ros/ros2_ws/install/setup.bash
fi

# Execute the command passed to the container as the 'ros' user
# This is a critical step to ensure the final process runs with the correct user,
# especially if you have an ENTRYPOINT that runs as root.
exec sudo -u ros "$@"