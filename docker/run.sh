#!/bin/bash

# Make the script executable
chmod +x entrypoint.sh

# Allow X11 forwarding (run on host machine)
xhost +local:root

# Build and run the container
docker compose up --build -d

# Execute a bash shell in the container
docker exec -it ros2-gazebo-dev /bin/bash 