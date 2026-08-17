import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _explicit_parameter_overrides(values):
    return {name: value for name, value in values.items() if value != ''}


def _require_file(path, description):
    if not os.path.isfile(path):
        raise RuntimeError(f'{description} does not exist or is not a file: {path}')


def _launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    parameter_overrides = _explicit_parameter_overrides(
        {
            name: LaunchConfiguration(name).perform(context)
            for name in (
                'cloud_topic',
                'imu_topic',
                'cloud_frame',
                'imu_frame',
                'initial_imu_frame',
            )
        }
    )

    _require_file(config_file, 'Unitree driver parameter file')

    driver_parameters = [config_file]
    if parameter_overrides:
        driver_parameters.append(parameter_overrides)

    driver = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        namespace=namespace,
        output='screen',
        parameters=driver_parameters,
        on_exit=[Shutdown(reason='Unitree LiDAR driver exited')],
    )

    actions = [driver]
    if IfCondition(LaunchConfiguration('use_rviz')).evaluate(context):
        _require_file(rviz_config, 'Unitree RViz configuration')
        rviz_remappings = []
        if 'cloud_topic' in parameter_overrides:
            rviz_remappings.append(('unilidar/cloud', parameter_overrides['cloud_topic']))
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=namespace,
                arguments=['-d', rviz_config],
                remappings=rviz_remappings,
                output='log',
            )
        )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory('unitree_lidar_ros2')
    default_config = os.path.join(package_share, 'config', 'driver.defaults.yaml')
    default_rviz_config = os.path.join(package_share, 'rviz', 'view.rviz')

    launch_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Installed YAML parameter file for transport and driver settings.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace for the Unitree driver node and its relative topics.',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='',
            description='Optional PointCloud2 topic override; empty keeps the config_file value.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='',
            description='Optional IMU topic override; empty keeps the config_file value.',
        ),
        DeclareLaunchArgument(
            'cloud_frame',
            default_value='',
            description='Optional cloud-frame override; empty keeps the config_file value.',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='',
            description='Optional IMU-frame override; empty keeps the config_file value.',
        ),
        DeclareLaunchArgument(
            'initial_imu_frame',
            default_value='',
            description='Optional dynamic-parent override; empty keeps the config_file value.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz deliberately; false is the headless robot default.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz configuration used only when use_rviz is true.',
        ),
    ]

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=_launch_setup)])
