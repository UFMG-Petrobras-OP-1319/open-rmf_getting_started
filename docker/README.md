# Open-RMF Docker Setup - Complete Guide

This directory contains Docker configurations for running Open-RMF in a containerized environment. This guide will explain everything you need to know about Docker, the services available, and how to use them effectively.

## Table of Contents
- [What is Docker?](#what-is-docker)
- [Docker Concepts Explained](#docker-concepts-explained)
- [Available Services](#available-services)
- [Quick Start Guide](#quick-start-guide)
- [Alternative Ways to Run Containers](#alternative-ways-to-run-containers)
- [Development Workflow](#development-workflow)
- [Volume Binding Explained](#volume-binding-explained)
- [Troubleshooting](#troubleshooting)
- [Next Steps](#next-steps)

## What is Docker?

Docker is a platform that allows you to package applications and their dependencies into standardized units called **containers**. Think of containers as lightweight, portable packages that include everything needed to run your application:

- **Operating system libraries** (Ubuntu 24.04 in our case)
- **Application code** (ROS 2, Gazebo, Open-RMF)
- **Dependencies** (Python packages, system tools)
- **Configuration files**

**Benefits of using Docker:**
- **Consistency**: Same environment across different machines
- **Isolation**: Applications don't interfere with each other
- **Portability**: Run anywhere Docker is installed
- **Version control**: Easy to switch between different versions
- **Clean environment**: No conflicts with system packages

## Docker Concepts Explained

### Container vs Image
- **Image**: A read-only template (like a blueprint) that contains the application and its environment
- **Container**: A running instance of an image (like a house built from a blueprint)

### Docker Compose
- **What it is**: A tool for defining and running multi-container applications
- **Why we use it**: Simplifies managing multiple services with a single configuration file
- **File**: `docker-compose.yml` defines all our services

### Key Docker Commands
```bash
# Build images from Dockerfiles
docker build -t image_name:tag .

# Run a container from an image
docker run image_name:tag

# List running containers
docker ps

# List all containers (including stopped ones)
docker ps -a

# List images
docker images

# Stop a running container
docker stop container_name

# Remove a container
docker rm container_name

# Remove an image
docker rmi image_name

# Execute a command in a running container
docker exec -it container_name /bin/bash

# View container logs
docker logs container_name
```

## Available Services

Our `docker-compose.yml` defines three main services:

### 1. `ros-gazebo-base` (Base Service)
- **Purpose**: Foundation image with ROS 2 Jazzy + Gazebo Harmonic
- **Use case**: Building other images, testing basic ROS functionality
- **Profile**: Only available with `--profile base-only` flag
- **Container name**: `ros-gazebo-base`

### 2. `openrmf` (Production Service)
- **Purpose**: Complete Open-RMF installation ready for demos
- **Use case**: Running demos, testing, production use
- **Container name**: `openrmf-container`
- **Features**: Full Open-RMF installation with rmf_demos

### 3. `openrmf-dev` (Development Service)
- **Purpose**: Development environment with source code mounting
- **Use case**: Code development, debugging, testing changes
- **Container name**: `openrmf-dev-container`
- **Features**: Source code mounted as volume, persistent changes

## Quick Start Guide

### Prerequisites
- Docker installed and running
- NVIDIA GPU support (recommended for better performance)
- X11 display setup (for GUI applications)

### Step 1: Setup Display (Required for GUI)
```bash
# Make the script executable
chmod +x setup_display.sh

# Run the display setup
./setup_display.sh
```

**What this does**: Sets up X11 forwarding so GUI applications (Gazebo, RViz) can display on your host machine.

### Step 2: Setup NVIDIA GPU Support (Recommended)
```bash
# Install NVIDIA Container Toolkit
sudo apt install nvidia-container-toolkit

# Restart Docker service
sudo systemctl restart docker

# Verify installation
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

**What this does**: Enables GPU acceleration for better Gazebo and RViz performance.

### Step 3: Build Images
```bash
# Build all services (recommended)
docker compose build

# Or build specific services
docker compose build ros-gazebo-base
docker compose build openrmf
```

**What this does**: Creates Docker images from the Dockerfile definitions.

### Step 4: Run Open-RMF
```bash
# For production/demo use
docker compose up openrmf

# For development with source code mounting
docker compose up openrmf-dev

# Run in background
docker compose up -d openrmf-dev
```

**What this does**: Starts the container and makes it available for use.

### Step 5: Access the Container
```bash
# If running in foreground, you're already inside
# If running in background, connect to it:
docker exec -it openrmf-container /bin/bash
# or for dev container:
docker exec -it openrmf-dev-container /bin/bash
```

### Step 6: Test Installation
```bash
# Source ROS environments
source /opt/ros/jazzy/setup.bash
source /home/ros/rmf_ws/install/setup.bash

# Run hotel demo
ros2 launch rmf_demos_gz hotel.launch.xml
```

## Alternative Ways to Run Containers

While we recommend Docker Compose, here are alternatives:

### 1. Direct Docker Run (Manual)
```bash
# Build image first
docker build -f Dockerfile.openrmf -t openrmf:latest .

# Run container manually
docker run -it \
  --name my-openrmf \
  --network host \
  --privileged \
  --runtime nvidia \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
  -v $(pwd)/..:/workspace:rw \
  openrmf:latest /bin/bash
```

**Pros**: Full control over parameters
**Cons**: Complex commands, harder to manage

### 2. Docker Compose (Recommended)
```bash
# Start service
docker compose up openrmf

# Start in background
docker compose up -d openrmf

# Stop service
docker compose down openrmf

# View logs
docker compose logs openrmf
```

**Pros**: Simple commands, configuration management, service orchestration
**Cons**: Less flexibility for one-off containers

### 3. Docker Compose with Overrides
```bash
# Create docker-compose.override.yml for custom settings
# Then run normally
docker compose up openrmf
```

## Development Workflow

### Using the Development Service
```bash
# Start development container
docker compose up -d openrmf-dev

# Connect to container
docker exec -it openrmf-dev-container /bin/bash

# Your source code is mounted at /workspace
cd /workspace
ls -la  # See your files
```

### Making Code Changes
1. **Edit files on your host machine** (they're automatically available in the container)
2. **Build inside the container** (ROS build system, dependencies available)
3. **Test changes immediately** (no need to rebuild images)

### Rebuilding After Major Changes
```bash
# If you modify Dockerfiles or need fresh environment
docker compose build openrmf-dev
docker compose up -d openrmf-dev
```

## Volume Binding Explained

Volume binding connects directories between your host machine and the container:

### Volume Mappings in Our Services
```yaml
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw          # X11 display
  - /tmp/.docker.xauth:/tmp/.docker.xauth:rw   # X11 authentication
  - ../:/workspace:rw                          # Your source code
  - ~/.ros:/home/ros/.ros:rw                   # ROS configuration
  - /dev/shm:/dev/shm                          # Shared memory
```

### What Each Volume Does

#### 1. X11 Display (`/tmp/.X11-unix`)
- **Host**: `/tmp/.X11-unix` (X11 socket)
- **Container**: `/tmp/.X11-unix`
- **Purpose**: Allows GUI applications to display on your screen
- **Mode**: `rw` (read-write)

#### 2. X11 Authentication (`/tmp/.docker.xauth`)
- **Host**: `/tmp/.docker.xauth` (X11 auth file)
- **Container**: `/tmp/.docker.xauth`
- **Purpose**: Authenticates X11 connections
- **Mode**: `rw` (read-write)

#### 3. Source Code (`../:/workspace`)
- **Host**: `../` (parent directory of docker folder)
- **Container**: `/workspace`
- **Purpose**: Makes your code available inside container
- **Mode**: `rw` (read-write) - changes persist!

#### 4. ROS Configuration (`~/.ros:/home/ros/.ros`)
- **Host**: `~/.ros` (your ROS config)
- **Container**: `/home/ros/.ros`
- **Purpose**: Preserves ROS settings between runs
- **Mode**: `rw` (read-write)

#### 5. Shared Memory (`/dev/shm:/dev/shm`)
- **Host**: `/dev/shm` (shared memory)
- **Container**: `/dev/shm`
- **Purpose**: Better performance for inter-process communication
- **Mode**: `rw` (read-write)

### Volume Binding Benefits
- **Persistence**: Changes survive container restarts
- **Real-time sync**: Edit on host, see in container immediately
- **Performance**: No need to copy files
- **Development**: Iterate quickly without rebuilding images

## Troubleshooting

### Common Issues and Solutions

#### 1. GUI Applications Not Working
```bash
# Check display setup
echo $DISPLAY

# Re-run display setup
./setup_display.sh

# Verify X11 forwarding
xhost +local:docker
```

#### 2. Permission Issues
```bash
# Check file ownership
ls -la

# Fix permissions if needed
sudo chown -R $USER:$USER .
```

#### 3. GPU Issues
```bash
# Test NVIDIA support
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# Reinstall if needed
sudo apt install --reinstall nvidia-container-toolkit
sudo systemctl restart docker
```

#### 4. Network Issues
```bash
# Check if host networking works
docker run --rm --network host ubuntu:latest ip addr

# Verify multicast support
docker run --rm --network host ubuntu:latest ping -c 1 224.0.0.1
```

#### 5. Build Failures
```bash
# Clean build (no cache)
docker compose build --no-cache

# Check disk space
df -h

# Verify internet connection
docker run --rm ubuntu:latest ping -c 3 google.com
```

### Debugging Commands
```bash
# View container logs
docker compose logs openrmf

# Check container status
docker compose ps

# Inspect container details
docker inspect openrmf-container

# View resource usage
docker stats openrmf-container
```

## Next Steps

Once your Docker environment is running:

1. **Explore the demos**: Try different rmf_demos scenarios
2. **Custom maps**: Follow the [custom map tutorial](../open_rmf_custom_map/README.md)
3. **Traffic Editor**: Learn to create custom maps
4. **rmf-web**: Set up the web interface
5. **Real robots**: Connect to physical hardware

## Additional Resources

- [Main Open-RMF README](../README.md)
- [Open-RMF Documentation](https://osrf.github.io/ros2multirobotbook/)
- [rmf_demos Repository](https://github.com/open-rmf/rmf_demos)
- [Docker Official Documentation](https://docs.docker.com/)
- [Docker Compose Documentation](https://docs.docker.com/compose/)

## Quick Reference Commands

```bash
# Start development environment
docker compose up -d openrmf-dev

# Connect to container
docker exec -it openrmf-dev-container /bin/bash

# Stop all services
docker compose down

# Rebuild after changes
docker compose build openrmf-dev

# View running containers
docker compose ps

# View logs
docker compose logs -f openrmf-dev
```

---

**Need help?** Check the troubleshooting section above or refer to the main Open-RMF documentation. The Docker setup provides a consistent, isolated environment perfect for both development and production use of Open-RMF. 