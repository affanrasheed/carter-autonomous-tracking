
from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('log_level', 'info', choices=[
                 'debug', 'info', 'warn'], cli=True)
    args.add_arg('num_cameras', 1,
                 description='How many cameras to use.', cli=True)
    args.add_arg(
        'mode',
        default=NvbloxMode.static,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    args.add_arg(
        'people_segmentation',
        default=NvbloxPeopleSegmentation.peoplesemsegnet_vanilla,
        choices=[
            str(NvbloxPeopleSegmentation.peoplesemsegnet_vanilla),
            str(NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg)
        ],
        description='The  model type of PeopleSemSegNet (only used when mode:=people_segmentation).',
        cli=True)
    args.add_arg(
        'lidar', False, description='Whether to use 3d lidar for 3d reconstruction', cli=True)
    args.add_arg(
        'navigation',
        True,
        description='Whether to enable nav2 for navigation in Isaac Sim.',
        cli=True)
    args.add_arg(
        'attach_to_container',
        'False',
        description='Add components to an existing component container.',
        cli=True)
    args.add_arg(
        'container_name',
        NVBLOX_CONTAINER_NAME,
        description='Name of the component container.')
    args.add_arg(
        'use_foxglove_whitelist',
        True,
        description='Disable visualization of bandwidth-heavy topics',
        cli=True)
    actions = args.get_launch_actions()

    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True))

    camera_mode = NvbloxCamera.isaac_sim
    # Visual SLAM
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/vslam_carter.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'camera': camera_mode,
                'enable_imu_fusion': False
            },
        ))

    
    # Nvblox
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/nvblox_carter.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'mode': args.mode,
                'camera': camera_mode,
                'num_cameras': args.num_cameras,
            }))

    # Navigation
    # NOTE: needs to be called before the component container because it modifies params globally
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/navigation/nvblox_carter_navigation_vslam.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'mode': args.mode,
            },
            condition=IfCondition(lu.is_true(args.navigation))))
    # Visualization
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/visualization/visualization.launch.py',
            launch_arguments={
                'mode': args.mode,
                'camera': camera_mode,
                'use_foxglove_whitelist': args.use_foxglove_whitelist,
            }))


    # Container
    actions.append(
        lu.component_container(
            NVBLOX_CONTAINER_NAME, container_type='isolated', log_level=args.log_level))

    return LaunchDescription(actions)