# Use a base image with ROS 2 Humble and Isaac ROS development environment
FROM isaac_ros_dev-x86_64
#FROM isaac_ros_dev-aarch64
WORKDIR /workspaces/isaac_ros-ws

RUN apt-get update && apt-get install -y \
    ros-humble-isaac-ros-ess \
    ros-humble-isaac-ros-ess-models-install \
    ros-humble-isaac-ros-examples

COPY ./isaac_ros_sim1.launch.py /opt/ros/humble/share/isaac_ros_ess/launch/
COPY ./entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["bash"]
CMD ["bash", "-c", "ros2 launch isaac_ros_ess isaac_ros_sim1.launch.py engine_file_path:='/workspaces/isaac_ros-ws/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine' threshold:=0.4"]

