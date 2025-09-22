#!/bin/bash


echo "Creating non-root container '${USERNAME}' for host user uid=${HOST_USER_UID}:gid=${HOST_USER_GID}"

if [ ! $(getent group ${HOST_USER_GID}) ]; then
  groupadd --gid ${HOST_USER_GID} ${USERNAME} &>/dev/null
else
  CONFLICTING_GROUP_NAME=`getent group ${HOST_USER_GID} | cut -d: -f1`
  groupmod -o --gid ${HOST_USER_GID} -n ${USERNAME} ${CONFLICTING_GROUP_NAME}
fi

if [ ! $(getent passwd ${HOST_USER_UID}) ]; then
  useradd --no-log-init --uid ${HOST_USER_UID} --gid ${HOST_USER_GID} -m ${USERNAME} &>/dev/null
else
  CONFLICTING_USER_NAME=`getent passwd ${HOST_USER_UID} | cut -d: -f1`
  usermod -l ${USERNAME} -u ${HOST_USER_UID} -m -d /home/${USERNAME} ${CONFLICTING_USER_NAME} &>/dev/null
  mkdir -p /home/${USERNAME}
  # Wipe files that may create issues for users with large uid numbers.
  rm -f /var/log/lastlog /var/log/faillog
fi

# Update 'admin' user
chown ${USERNAME}:${USERNAME} /home/${USERNAME}
echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME}
chmod 0440 /etc/sudoers.d/${USERNAME}
adduser ${USERNAME} video >/dev/null
adduser ${USERNAME} plugdev >/dev/null
adduser ${USERNAME} sudo  >/dev/null

# If jtop present, give the user access
if [ -S /run/jtop.sock ]; then
  JETSON_STATS_GID="$(stat -c %g /run/jtop.sock)"
  addgroup --gid ${JETSON_STATS_GID} jtop >/dev/null
  adduser ${USERNAME} jtop >/dev/null
fi

# Run all entrypoint additions
shopt -s nullglob
for addition in /usr/local/bin/scripts/entrypoint_additions/*.sh; do
  if [[ "${addition}" =~ ".user." ]]; then
    echo "Running entryrypoint extension: ${addition} as user ${USERNAME}"
    gosu ${USERNAME} ${addition}
  else
    echo "Sourcing entryrypoint extension: ${addition}"
    source ${addition}
  fi
done

# Restart udev daemon
service udev restart

set -e

# Setup ROS 2 environment
source /opt/ros/humble/setup.bash

# Set ISAAC_ROS_WS and run model installer if models are not already installed
export ISAAC_ROS_WS=/workspaces/isaac_ros-ws
export ISAAC_ROS_ACCEPT_EULA=1

# Check if ESS models are already installed (assuming they're installed in a specific directory)
ESS_MODELS_DIR="/workspaces/isaac_ros-ws/isaac_ros_assets/models/dnn_stereo_disparity"
if [ ! -d "$ESS_MODELS_DIR" ] || [ -z "$(ls -A $ESS_MODELS_DIR 2>/dev/null)" ]; then
    echo "----------------------------------------------------------------"
    echo "ESS models not found. Installing models..."
    echo "----------------------------------------------------------------"
    
    ros2 run isaac_ros_ess_models_install install_ess_models.sh
    
    echo "----------------------------------------------------------------"
    echo "ESS models installation complete."
    echo "----------------------------------------------------------------"
else
    echo "Found existing ESS models. Skipping installation."
fi

# Execute the command specified by the user (e.g., "bash", "ros2 launch ...")
#exec "$@"
exec gosu ${USERNAME} "$@"