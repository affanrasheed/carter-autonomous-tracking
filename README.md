# Carter - Autonomous Object Tracking System

A comprehensive Docker Compose setup for deploying Carter, an autonomous object tracking and navigation system that uses computer vision, SLAM, and ROS2 for intelligent object following and path planning in Isaac Sim.
![alt-text](assets/demo.gif)

## System Overview

The Carter system consists of four main containerized services that work together in a streamlined workflow:

1. **Depth Generation Service** (`ess-depth`) - Processes stereo camera images and produces depth maps using NVIDIA Isaac ROS ESS
2. **Perception Service** (`robot-path-planning`) - Uses depth, stereo vision images, and goal poses for 3D reconstruction (nvblox), robot pose estimation (nvslam), and autonomous navigation (nav2)


## Prerequisites

### Hardware Requirements
- NVIDIA GPU with CUDA support (RTX series recommended)
- x86_64 system (or ARM64 for Jetson deployment)
- Minimum 8GB GPU VRAM
- 16GB+ System RAM

### Software Requirements
- Docker Engine (>= 20.10)
- Docker Compose (>= 2.0)
- NVIDIA Container Toolkit
- Isaac Sim 2023.1.1+ (for simulation)
- X11 server (for GUI applications)
- CUDA 12.6+ or Jetpack 6.1 or 6.2


## Installation & Setup

### 1. Clone and Setup
```bash
# Clone or navigate to Carter directory
cd /path/to/carter

# Make startup script executable
chmod +x start-carter.sh
```

### 2. Build Docker Images

Before running Carter, you need to build the required Docker images. **Important: Verify that the base image names in the Dockerfiles match your Isaac ROS setup before building.**

```bash
# Build Depth Generation Service image
cd docs/ESS-Depth
# Verify base image name in Dockerfile before building
docker build -t ess_ros:x86 .

# Build Perception Service image (includes nvslam, nvblox, nav2)
cd ../Perception
# Verify base image name in Dockerfile before building
docker build -t nvblox_ros:x86 .
```

### 3. Configure Environment
Edit the `carter.env` file if needed (defaults should work):

```bash
# Architecture (x86 for x86_64, arm64 for ARM64/Jetson)
ARCH_TAG=x86

# Model paths (created automatically if missing)
ESS_MODEL_PATH=/home/${USER}/ess_model

# ROS configuration
ROS_DOMAIN_ID=0
```

### 4. Prerequisites Setup

Follow the individual component README files for detailed setup:
- `docs/ESS-Depth/Readme.md` - Depth generation service setup
- `docs/goal_pose_generation/README.md` - Goal pose generation service setup

## Usage Instruction

### Quick Start with Startup Script
- For first run only, run these services 
```bash
# On first run it will perform tensorrt optimization on ESS model specific to hardware
# This optimization process will take 10 mins to 30 mins depending on the hardware
# The optimized engine file will be stored locally and won't build again on second run and so on 
# On successful build you can close this service
./start-carter.sh --profile ess-depth
```
- The easiest way to start Carter is using the provided startup script:

```bash
# Start all services (recommended)
./start-carter.sh

# Start with specific architecture
./start-carter.sh --arch x86     # For x86_64 systems
./start-carter.sh --arch arm64   # For ARM64/Jetson systems

# Start in detached mode
./start-carter.sh --detached

# View help
./start-carter.sh --help
```

### Alternative: Direct Docker Compose

```bash
# Start all services
docker compose --env-file carter.env --profile all up

# Or specify architecture inline
ARCH_TAG=x86 docker compose --profile all up
```

### Running Individual Services

You can start specific services using profiles:

#### Depth Generation Service Only
```bash
./start-carter.sh --profile ess-depth
# OR: docker compose --env-file carter.env --profile ess-depth up -d
```
#### Perception Service Only
```bash
./start-carter.sh --profile robot-path-planning
# OR: docker compose --env-file carter.env --profile robot-path-planning up -d
```

#### Combined Services
```bash
# Depth + Object Detection (without perception service)
docker compose --env-file carter.env --profile ess-depth --profile robot-path-planning up -d
```

### Stopping Services

```bash
# Using startup script
./start-carter.sh --stop

# Or manually
docker compose down

# Stop and remove volumes
docker compose down -v

# Force stop and remove everything
docker compose down -v --remove-orphans
```

### Viewing Logs

```bash
# Using startup script
./start-carter.sh --logs

# Or manually
docker compose logs -f
```
## Testing Instructions

### Testing Setup Overview

The Carter system supports a distributed testing setup where Isaac Sim runs on a separate computer, while all other services run on the Jetson. 

### Hardware Requirements for Testing

#### Network Configuration
- **Ethernet Connection**: Connect Isaac Sim computer and Jetson via ethernet cable
- **Bandwidth**: Ensure ethernet cable supports **1GB/s or above** to avoid ethernet bottleneck for data transfer
- **Network Setup**: Both computers should be on the same network segment for ROS2 communication

#### Computer Specifications
- **Isaac Sim Computer**: High-performance system with powerful GPU for simulation
- **Jetson Computer**: NVIDIA Jetson device for real-time robot control and navigation

### Isaac Sim Computer Setup

1. **Load [USD File](https://drive.google.com/file/d/1lrf2Ae8w3ttj59MytfBI30gwEWmLpjh6/view?usp=sharing)**:
   ```bash
   # Load the USD file in Isaac Sim
   # Open Isaac Sim and load this file to set up the Carter robot environment
   ```

2. **Configure ROS2 Networking**:
   ```bash
   # Set ROS domain (must match Jetson)
   export ROS_DOMAIN_ID=0

   # Enable ROS2 discovery across network
   export ROS_LOCALHOST_ONLY=0
   ```

### Jetson Computer Setup

1. **Configure ROS2 Networking**:
   ```bash
   # Set ROS domain (must match Isaac Sim computer)
   export ROS_DOMAIN_ID=0

   # Enable ROS2 discovery across network
   export ROS_LOCALHOST_ONLY=0
   ```

2. **Run Jetson Services**:
   ```bash
   # Run all services
   ARGH_TAG=arm64 docker compose --env-file carter.env --profile all up 
   ```

### Testing Procedure

1. **Start Isaac Sim Computer**:
   - Load `carter_perception.usd` in Isaac Sim

2. **Start Jetson Computer**:
   - Launch Carter services
   - Verify all services are running and communicating

3. **Test System Integration**:
   ```bash
   # Verify ROS2 communication between computers
   ros2 topic list  # Should show topics from both computers

   # Monitor system output
   ros2 topic echo /depth
   ros2 topic echo /cmd_vel
   ```

4. **Network Troubleshooting**:
   ```bash
   # Check network connectivity
   ping [other_computer_ip]

   # Verify ROS2 discovery
   ros2 node list  # Should show nodes from both computers

   # Check topic bridge
   ros2 topic hz /depth  # Verify depth data from Isaac Sim computer
   ```

### Notes for Distributed Testing

- Ensure both computers have synchronized time for proper ROS2 communication
- Monitor network bandwidth usage during operation - reduce image resolution if needed
- Use `ROS_LOCALHOST_ONLY=0` on both computers to enable network communication
- Consider firewall settings that might block ROS2 discovery (ports 7400-7500)
- Latency between computers should be minimized for optimal performance

## Architecture Support

### Switching Between Architectures

#### For x86_64 Systems
```bash
# Build images for x86 (verify base image names in Dockerfiles first)
cd docs/ESS-Depth && docker build -t ess_ros:x86 .
cd ../Perception && docker build -t nvblox_ros:x86 .

# Start with x86 architecture
./start-carter.sh --arch x86
```

#### For ARM64 Systems (Jetson)
```bash
# Build images for ARM64 (verify base image names in Dockerfiles first)
cd docs/ESS-Depth && docker build -t ess_ros:arm64 .
cd ../Perception && docker build -t nvblox_ros:arm64 .

# Start with ARM64 architecture
./start-carter.sh --arch arm64
```

## Monitoring & Debugging

### View Logs
```bash
# All services
docker compose logs -f

# Specific service
docker compose logs -f ess-depth
docker compose logs -f robot-path-planning
```

### Check Service Status
```bash
docker compose ps
```

### Interactive Access
```bash
# Access running containers
docker exec -it carter_path_planning bash
docker exec -it carter_ess_depth bash
```

## ROS2 Topics

### Input Topics
- `/front_stereo_camera/left/image_rect_color` (sensor_msgs/Image) - Left stereo camera
- `/front_stereo_camera/right/image_rect_color` (sensor_msgs/Image) - Right stereo camera
- `/front_stereo_camera/left/camera_info` (sensor_msgs/CameraInfo) - Left camera info
- `/front_stereo_camera/right/camera_info` (sensor_msgs/CameraInfo) - Right camera info
- `/front_stereo_imu/imu` (sensor_msgs/Imu) - IMU data

### Output Topics
- `/depth` (sensor_msgs/Image) - Depth estimation from ESS
- `/cmd_vel` (geometry_msgs/Twist) - Robot movement commands
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goals

### Useful ROS2 Commands
```bash
# List all topics
ros2 topic list

# Monitor robot commands
ros2 topic echo /cmd_vel

# Set ROS domain
export ROS_DOMAIN_ID=0
```

## Troubleshooting

### Common Issues

#### NVIDIA Runtime Not Found
```bash
# Install NVIDIA Container Toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

#### X11 Display Issues
```bash
# Allow X11 forwarding
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY
```

#### Permission Issues
```bash
# Check user IDs (automatically configured in carter.env)
id -u  # Should match HOST_USER_UID (default: 1000)
id -g  # Should match HOST_USER_GID (default: 1000)

# The carter.env file handles user mapping automatically
```

#### Model Directory Issues
```bash
# The startup script will automatically create missing directories
# But you can manually check:
ls -la ~/ess_model
```

### Service-Specific Debugging

#### Depth Generation Service (ESS)
- Check stereo camera topics: `/front_stereo_camera/left/image_rect_color1`, `/front_stereo_camera/right/image_rect_color1`
- Verify ESS model files exist in `~/ess_model`
- Monitor depth output: `ros2 topic echo /depth`

#### Perception Service (nvslam, nvblox, nav2)
- Monitor SLAM topics and navigation goals
- Check IMU data: `ros2 topic echo /front_stereo_imu/imu`
- Verify navigation commands: `ros2 topic echo /cmd_vel`
- Check Nvblox mapping and VSLAM status

#### Isaac Sim Integration
- Ensure carter_perception.usd is loaded
- Verify camera and IMU topics are publishing
- Check ROS bridge connection
- Monitor simulation time: `ros2 topic echo /clock`

## Project Structure

```
carter/
├── docker-compose.yml           # Main Docker Compose configuration
├── carter.env                   # Environment variables and paths
├── start-carter.sh              # Convenient startup script
├── README.md                    # This documentation
└── docs/                        # Service components and documentation
    ├── ESS-Depth/                # Depth generation service Docker setup
    ├── Perception/               # Perception service (nvslam, nvblox, nav2)

```
## Configuration Files

- `docker-compose.yml` - Service definitions and container orchestration
- `carter.env` - Environment variables, paths, and system configuration
- `start-carter.sh` - Automated startup script with multiple options
- `carter_perception.usd` - Isaac Sim world file with Carter robot setup


