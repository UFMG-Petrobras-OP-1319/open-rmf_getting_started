# Open-RMF Docker Setup

This directory contains Docker configurations for running Open-RMF in a containerized environment. The setup consists of two Docker images:

1. **Base Image** (`Dockerfile.base`): Contains Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic
2. **Open-RMF Image** (`Dockerfile.openrmf`): Builds on the base image and installs Open-RMF following the binary installation method from the main README

## Quick Start

### 1. Setup X11 Display (for GUI applications)

Before running any containers with GUI applications like Gazebo and RViz, run the display setup script:

```bash
./setup_display.sh
```

### 2. Setup NVIDIA GPU Support (recommended)

For better performance with Gazebo and RViz, ensure NVIDIA GPU support is available:

```bash
# If GPU support is missing, install nvidia-container-toolkit:
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker
```

### 3. Build Docker Images

```bash
# Build the base ROS image
docker compose build ros-gazebo-base

# Build the Open-RMF image
docker compose build openrmf
```

### 4. Build and Run Open-RMF Container

```bash
# Build and start the Open-RMF container
docker compose up openrmf

# Or for development with source code mounting
docker compose up openrmf-dev
```

### 5. Test Open-RMF Installation

Inside the container, test the installation by running a demo:

```bash
# Source the environments (already in .bashrc)
source /opt/ros/jazzy/setup.bash
source /home/ros/rmf_ws/install/setup.bash

# Run the hotel demo
ros2 launch rmf_demos_gz hotel.launch.xml
```

In another terminal (inside the same container), dispatch tasks:

```bash
# Open another terminal in the running container
docker exec -it openrmf-container /bin/bash

# Source environments
source /opt/ros/jazzy/setup.bash
source /home/ros/rmf_ws/install/setup.bash

# Dispatch tasks
ros2 run rmf_demos_tasks dispatch_patrol -p restaurant L3_master_suite -n 1 --use_sim_time
ros2 run rmf_demos_tasks dispatch_clean -cs clean_lobby --use_sim_time
```

## Available Services

### `openrmf`
- Full Open-RMF installation with rmf_demos
- Suitable for running demos and testing
- Container name: `openrmf-container`

### `openrmf-dev`
- Same as `openrmf` but with workspace mounted as volume
- Suitable for development and code editing
- Source code changes are persistent
- Container name: `openrmf-dev-container`

### `ros-gazebo-base`
- Base image with ROS 2 and Gazebo only
- Used for building the Open-RMF image
- Available with profile: `docker compose --profile base-only up ros-gazebo-base`

## Manual Build Commands

If you prefer to build images manually:

```bash
# Build base image
docker build -f Dockerfile.base -t ros-gazebo-base:latest .

# Build Open-RMF image
docker build -f Dockerfile.openrmf -t openrmf:latest .
```

## Directory Structure

```
docker/
├── Dockerfile.base          # Base ROS 2 + Gazebo image
├── Dockerfile.openrmf       # Open-RMF installation
├── docker-compose.yml       # Multi-service configuration
├── setup_display.sh         # X11 setup for GUI apps
├── README.md                # This file
├── entrypoint.sh            # Container entrypoint (legacy)
└── run.sh                   # Container runner (legacy)
```

## Installation Methods

The Open-RMF Docker image follows the binary installation method from the main README, which includes:

- Installing `ros-jazzy-rmf-dev` package
- Cloning and building `rmf_demos` from source
- Setting up Python virtual environment for Ubuntu 24.04 compatibility
- Installing required Python packages (`cmake`, `shapely`, `yaml`, `requests`)
- NVIDIA GPU support for hardware-accelerated Gazebo and RViz

## Troubleshooting

### GUI Applications Not Working

1. Make sure you ran `./setup_display.sh` before starting containers
2. Check if your display is set: `echo $DISPLAY`
3. For NVIDIA GPUs, ensure nvidia-container-toolkit is installed

### Container Permission Issues

The containers run as user `ros` (UID 2000) to avoid permission issues. If you encounter file permission problems with mounted volumes, check the ownership of your files.

### Network Issues

If you experience networking problems (e.g., multicast issues mentioned in the main README), the containers use `network_mode: host` to avoid Docker networking complications.

### GPU/Graphics Issues

If you experience slow graphics or Gazebo performance issues:
1. Install NVIDIA Container Toolkit: `sudo apt install nvidia-container-toolkit`
2. Restart Docker: `sudo systemctl restart docker`
3. Verify with: `docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi`

### Building Issues

If builds fail due to network timeouts or download issues:
1. Try building again (some downloads might have failed)
2. Check your internet connection
3. Consider using the `--no-cache` flag: `docker compose build --no-cache`

## Next Steps

Once your Open-RMF Docker environment is running:

1. Follow the [Traffic Editor tutorial](../README.md#traffic-editor) to create custom maps
2. Explore the [custom map tutorial](../open_rmf_custom_map/README.md)
3. Try setting up [rmf-web](../README.md#getting-started-with-the-rmf-web) for web interface

## Additional Resources

- [Main Open-RMF README](../README.md)
- [Open-RMF Documentation](https://osrf.github.io/ros2multirobotbook/)
- [rmf_demos Repository](https://github.com/open-rmf/rmf_demos) 