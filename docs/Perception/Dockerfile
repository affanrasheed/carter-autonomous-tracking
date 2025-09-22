# Use a base image with ROS 2 Humble and Isaac ROS development environment
FROM isaac_ros_dev-x86_64
#FROM isaac_ros_dev-aarch64  

WORKDIR /workspaces/isaac_ros-dev

RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git src/isaac_ros_common

RUN apt update && apt-get install -y ros-humble-isaac-ros-nvblox && \
    rosdep update && \
    rosdep install isaac_ros_nvblox

## main launch files
COPY ./launch/carter_launch.py /opt/ros/humble/share/nvblox_examples_bringup/launch/

## nvblox, vslam  and navigation launch files
COPY ./launch/nvblox_carter_navigation_vslam.launch.py /opt/ros/humble/share/nvblox_examples_bringup/launch/navigation/
COPY ./launch/vslam_carter.launch.py /opt/ros/humble/share/nvblox_examples_bringup/launch/perception/
COPY ./launch/nvblox_carter.launch.py /opt/ros/humble/share/nvblox_examples_bringup/launch/perception/

## config files
COPY ./config/carter_nav2_vslam.yaml /opt/ros/humble/share/nvblox_examples_bringup/config/navigation

COPY ./entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["bash"]
CMD ["bash", "-c", "ros2 launch nvblox_examples_bringup carter_launch.py mode:=dynamic"]
