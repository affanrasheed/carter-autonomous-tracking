# Depth Generation Service - Carter System Component

This module provides stereo vision depth estimation for the Carter autonomous system using NVIDIA Isaac ROS ESS.

## Overview

The Depth Generation Service is the component in the Carter workflow that:
- Takes stereo camera images from Isaac Sim as input
- Processes stereo camera feeds to generate accurate depth maps
- Uses NVIDIA's ESS algorithm for high-quality depth estimation
- Provides depth information to the Perception Service.
- Enables 3D object localization for the autonomous navigation pipeline

## Prerequisites

### Hardware Requirements
- NVIDIA GPU with CUDA support
- Minimum 4GB GPU VRAM
- x86_64 or ARM64 architecture

### Software Requirements
- NVIDIA Isaac ROS development environment
- Docker with NVIDIA runtime support
- CUDA 12.6+ or Jetpack 6.1/6.2

## Installation & Setup

### 1. Setup Isaac ROS Development Environment

Follow the official Isaac ROS setup guide:
```bash
# Set up development environment
# Follow: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html
```

### 2. Build Base Isaac ROS Image

Build the Isaac ROS base image following the ESS documentation:
```bash
# Follow the setup guide here:
# https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_ess/index.html
# Stop after running the container using run_dev script
```

### 3. Build ESS Docker Image

**Important: Before building, verify that the base image name in the Dockerfile matches your Isaac ROS setup.**

```bash
# Navigate to ESS-Depth directory
cd /path/to/ESS-Depth

# Verify base image name in Dockerfile
cat Dockerfile | grep FROM

# Build the Docker image
docker build --network=host -t ess_ros:x86 .

# For ARM64/Jetson systems
docker build --network=host -t ess_ros:arm64 .
```

### 4. Setup Model Directory

```bash
# Create ESS model directory (auto-created by Carter startup script)
mkdir -p ~/ess_model

# Ensure proper permissions
sudo chown -R $USER:$USER ~/ess_model
```

## Usage

### Standalone Operation

```bash
# Run ESS depth estimation service
# First run will take time as ESS model will be optimize specific to hardware
docker run -it --rm --privileged --network=host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/admin/.Xauthority:rw \
  -e DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_DOMAIN_ID -e USER \
  -e HOST_USER_UID=`id -u` -e HOST_USER_GID=`id -g` \
  -v /home/$USER/ess_model:/workspaces/isaac_ros-ws/isaac_ros_assets/models/dnn_stereo_disparity \
  -v /etc/localtime:/etc/localtime:ro \
  --gpus all ess_ros:x86
```

### Integration with Carter System

The Depth Generation Service is automatically managed by the Carter system:

```bash
# Start depth generation as part of complete Carter system
./start-carter.sh

# Start only depth generation service
./start-carter.sh --profile ess-depth

# Or using docker-compose directly
docker compose --env-file carter.env --profile ess-depth up
```

## ROS2 Topics

### Subscribed Topics
- `/front_stereo_camera/left/camera_info` (sensor_msgs/CameraInfo) - Left stereo camera calibration
- `/front_stereo_camera/left/image_rect_color` (sensor_msgs/Image) - Left stereo camera image
- `/front_stereo_camera/right/camera_info` (sensor_msgs/CameraInfo) - Right stereo camera calibration
- `/front_stereo_camera/right/image_rect_color` (sensor_msgs/Image) - Right stereo camera image

### Published Topics
- `/depth` (sensor_msgs/Image) - Generated depth map

## Configuration

The ESS service uses the following key configurations:
- **Model Path**: `/workspaces/isaac_ros-ws/isaac_ros_assets/models/dnn_stereo_disparity`
- **Input Resolution**: Configured for Isaac Sim stereo cameras
- **Depth Range**: Optimized for Carter's operational environment

## Troubleshooting

### Common Issues

#### No Stereo Camera Topics
```bash
# Check if Isaac Sim is publishing camera topics
ros2 topic list | grep camera
ros2 topic echo /front_stereo_camera/left/camera_info
```

#### ESS Model Files Missing
```bash
# Verify model directory exists and has proper permissions
ls -la ~/ess_model/
sudo chown -R $USER:$USER ~/ess_model/
```

#### GPU Memory Issues
```bash
# Monitor GPU usage
nvidia-smi

# Reduce batch size or image resolution if needed
```

#### Container Access Issues
```bash
# Verify NVIDIA Docker runtime
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

### Debug Commands
```bash
# Check ESS container logs
docker compose logs -f ess-depth

# Access running container
docker exec -it carter_ess_depth  bash

# Monitor depth output
ros2 topic hz /depth
ros2 topic echo /depth --field header.stamp
```

## Integration Notes

- **Perception Service Integration**: Depth information supports nvblox for 3D reconstruction and obstacle avoidance
- **Coordinate Frame**: Depth maps are aligned with the camera coordinate system for accurate 3D localization
- **Workflow Position**: First service in the Carter pipeline - takes stereo images and produces depth for downstream services

## Performance Optimization

- **GPU Utilization**: ESS is optimized for NVIDIA GPUs and provides real-time performance
- **Memory Management**: Configure model loading to optimize GPU memory usage
- **Network Bandwidth**: Depth maps can be compressed for network transmission if needed

## Support

For issues specific to ESS depth estimation:
1. Check NVIDIA Isaac ROS ESS documentation
2. Verify stereo camera calibration
3. Ensure sufficient GPU memory
4. Monitor ROS topic throughput

For Carter system integration issues, refer to the main Carter README.