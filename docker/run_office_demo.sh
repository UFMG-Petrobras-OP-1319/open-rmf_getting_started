#!/bin/bash

echo "=== Starting Open-RMF Office Demo ==="
echo ""

# Check if running inside container
if [ ! -d "/home/ros/rmf_ws" ]; then
    echo "❌ This script should be run inside the Docker container"
    echo "   Run: docker exec -it openrmf-dev-container bash"
    echo "   Then: ./run_office_demo.sh"
    exit 1
fi

# Source ROS and RMF environments
echo "🔧 Setting up environment..."
source /opt/ros/jazzy/setup.bash
source /home/ros/rmf_ws/install/setup.bash

# Set Gazebo plugin paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ros/rmf_ws/install/rmf_demos_assets/share/rmf_demos_assets/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/home/ros/rmf_ws/install/rmf_robot_sim_gz_plugins/lib
export GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins:/home/ros/rmf_ws/install/rmf_building_sim_gz_plugins/lib

# Check if Gazebo plugins exist
if [ ! -d "/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins" ]; then
    echo "⚠️  RMF robot simulation plugins not found"
    echo "   This may cause robot communication issues"
fi

if [ ! -d "/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins" ]; then
    echo "⚠️  RMF building simulation plugins not found"  
    echo "   This may cause GUI plugin errors"
fi

echo "✅ Environment configured"
echo ""
echo "🚀 Launching Office Demo..."
echo "   - This includes TinyRobot fleet"
echo "   - Gazebo simulation will start"
echo "   - RViz visualization will start"
echo ""
echo "Press Ctrl+C to stop the demo"
echo ""

# Launch the demo
ros2 launch rmf_demos_gz office.launch.xml 