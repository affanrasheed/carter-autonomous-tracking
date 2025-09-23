from typing import List

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def add_nvblox_carter_navigation(args: lu.ArgumentContainer) -> List[lut.Action]:
    # Nav2 base parameter file
    actions = []
    nav_params_path = lu.get_path('nvblox_examples_bringup', 'config/navigation/carter_nav2_vslam.yaml')
    actions.append(lut.SetParametersFromFile(str(nav_params_path)))
    actions.append(lut.SetParameter('use_sim_time', True))
    # Enabling nav2
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='plugins',
            value=['nvblox_layer', 'inflation_layer'],
        ))
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='plugins',
            value=['nvblox_layer', 'inflation_layer'],
        ))

    # Modifying nav2 parameters depending on nvblox mode
    mode = NvbloxMode[args.mode]
    if mode is NvbloxMode.static:
        costmap_topic_name = '/nvblox_node/static_map_slice'
    elif mode in [NvbloxMode.dynamic, NvbloxMode.people_segmentation]:
        costmap_topic_name = '/nvblox_node/combined_map_slice'
    else:
        raise Exception(f'Navigation in mode {mode} not implemented.')

    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='nvblox_layer.nvblox_map_slice_topic',
            value=costmap_topic_name,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='nvblox_layer.nvblox_map_slice_topic',
            value=costmap_topic_name,
        ))

    # Running carter navigation
    actions.append(
        lu.include(
            'nav2_bringup',
            'launch/navigation_launch.py',
            launch_arguments={
                'params_file': str(nav_params_path),
                'container_name': args.container_name,
                'use_composition': 'True',
                'use_sim_time': 'True',
            },
        ))
    actions.append(lu.static_transform('map', 'odom'))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)

    args.add_opaque_function(add_nvblox_carter_navigation)
    return lut.LaunchDescription(args.get_launch_actions())