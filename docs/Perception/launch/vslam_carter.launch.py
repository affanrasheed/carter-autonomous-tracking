from typing import List

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def add_vslam(args: lu.ArgumentContainer) -> List[Action]:
    actions = []

    camera = NvbloxCamera[args.camera]
    # NOTE(alexmillane, 19.08.2024): At the moment in nvblox_examples we only support a single
    # camera running cuVSLAM, even in the multi-camera case: we run *nvblox* on multiple
    # cameras, but cuVSLAM on camera0 only.
    isaac_sim_remappings = [
        ('visual_slam/camera_info_0', '/front_stereo_camera/left/camera_info'),
        ('visual_slam/camera_info_1', '/front_stereo_camera/right/camera_info'),
        ('visual_slam/image_0', '/front_stereo_camera/left/image_rect_color'),
        ('visual_slam/image_1', '/front_stereo_camera/right/image_rect_color'),
        ('visual_slam/imu', '/front_stereo_imu/imu'),
    ]

    # Base frame: 
    base_frame = 'base_link'

    actions.append(lu.log_info(f'Starting cuVSLAM with base_frame: {base_frame}'))

    base_parameters = {
        'imu_frame': 'front_stereo_camera_imu',
        'enable_slam_visualization': True,
        'enable_landmarks_view': True,
        'enable_observations_view': True,
        'base_frame': 'base_link',
        'use_sim_time': True,
        'rectified_images': True,
        'enable_image_denoising': True,
        'publish_map_to_odom_tf': False,
    }


    if camera is NvbloxCamera.isaac_sim:
        remappings = isaac_sim_remappings
    else:
        raise Exception(f'Camera {camera} not implemented for vslam.')

    parameters = []
    parameters.append(base_parameters)
    parameters.append(
        {'enable_ground_constraint_in_odometry': args.enable_ground_constraint_in_odometry})
    parameters.append({'enable_imu_fusion': args.enable_imu_fusion})

    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=remappings,
        parameters=parameters)
    actions.append(lu.load_composable_nodes(args.container_name, [vslam_node]))

    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('camera')
    args.add_arg(
        'enable_ground_constraint_in_odometry',
        'False',
        description='Whether to constraint robot movement to a 2d plane (e.g. for AMRs).',
        cli=True)
    args.add_arg(
        'enable_imu_fusion',
        'False',
        description='Whether to use imu data in visual slam.',
        cli=True)
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_opaque_function(add_vslam)
    return LaunchDescription(args.get_launch_actions())