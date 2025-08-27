#!/bin/bash

# Setup X11 forwarding for Docker GUI applications
# This script prepares the display for running Gazebo and RViz

echo "Setting up X11 display for Docker GUI applications..."

# Allow connections to X server
xhost +local:root

# Create .docker.xauth if it doesn't exist
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    echo "Creating X authentication file at $XAUTH"
    touch $XAUTH
fi

# Generate X auth entry
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "X11 setup complete. You can now run GUI applications from Docker containers."
echo "Run this script before starting Docker containers with GUI applications."
echo ""
echo "To start Open-RMF container:"
echo "  docker compose up openrmf"
echo ""
echo "To start development container:"
echo "  docker compose up openrmf-dev" 