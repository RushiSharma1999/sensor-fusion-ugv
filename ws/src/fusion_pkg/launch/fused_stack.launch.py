from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context) == 'true'


def _node(package, executable, **kwargs):
    return Node(
        package=package,
        executable=executable,
        output='screen',
        on_exit=[Shutdown(reason=f'{executable} exited')],
        **kwargs,
    )


def _setup(context):
    motion_enabled = _enabled(context, 'motion_enabled')
    use_tof = _enabled(context, 'use_tof')
    port = LaunchConfiguration('serial_port').perform(context)
    if motion_enabled and not port:
        raise RuntimeError('motion_enabled requires an explicit ESP32 serial_port')

    nodes = []
    if _enabled(context, 'start_driver'):
        cloud_scan_num = int(LaunchConfiguration('cloud_scan_num').perform(context))
        if not 1 <= cloud_scan_num <= 65535:
            raise ValueError('cloud_scan_num must be in [1, 65535]')
        nodes.append(_node(
            'unitree_lidar_ros2', 'unitree_lidar_main_node',
            name='unitree_lidar_ros2_node',
            parameters=[{
                'initialize_type': 2,
                'work_mode': 0,
                'use_system_timestamp': True,
                'range_min': 0.0,
                'range_max': 100.0,
                'cloud_scan_num': cloud_scan_num,
                'lidar_port': 6101,
                'local_port': 6201,
                'lidar_ip': LaunchConfiguration('lidar_ip').perform(context),
                'local_ip': LaunchConfiguration('local_ip').perform(context),
                'cloud_frame': 'unilidar_lidar',
                'cloud_topic': 'unilidar/cloud',
                'imu_frame': 'unilidar_imu',
                'imu_topic': 'unilidar/imu',
            }],
        ))

    nodes.extend([
        _node(
            'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/unilidar/cloud'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'unilidar_lidar',
                'min_height': -0.10,
                'max_height': 1.0,
                'range_min': 0.15,
                'range_max': 20.0,
                'use_inf': True,
            }],
        ),
        _node('ultrasonic_pkg', 'ultrasonic_processor'),
        _node('fusion_pkg', 'fusion_node', parameters=[{
            'require_ultrasonic': True,
            'require_tof': use_tof,
        }]),
        _node('vfh_pkg', 'vfh_main_node', parameters=[{
            'behavior': 'baseline',
            # Main's histogram mode consumes directional fusion penalties.
            'continuous_avoidance': False,
            'require_fusion_penalties': True,
            'min_speed': 0.0,
            'front_stop_dist': 0.5,
            'scan_timeout_sec': 0.25,
        }]),
        _node('vision_uart_pkg', 'safety_mux_node', parameters=[{
            'lidar_only_mode': False,
            'motion_enabled': motion_enabled,
            'front_stop_dist': 0.5,
            'vfh_timeout_sec': 0.25,
        }]),
    ])
    if use_tof:
        nodes.append(_node('tof_processor_pkg', 'tof_processor_node'))
    if port:
        nodes.append(_node('esp_bridge', 'cmd_vel_to_serial', parameters=[{
            'port': port,
            'baud': 115200,
            'debug_serial': True,
        }]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='',
            description='ESP32 device path. Empty leaves the serial bridge off.',
        ),
        DeclareLaunchArgument(
            'motion_enabled', default_value='false', choices=['true', 'false'],
            description='Allow the mux to forward motion after its guards pass.',
        ),
        DeclareLaunchArgument(
            'use_tof', default_value='false', choices=['true', 'false'],
            description='Enable and require fresh ToF features.',
        ),
        DeclareLaunchArgument(
            'start_driver', default_value='true', choices=['true', 'false'],
            description='Set false when replaying an external /unilidar/cloud source.',
        ),
        DeclareLaunchArgument(
            'cloud_scan_num', default_value='36',
            description='Unitree cloud packet count used by the main navigation launcher.',
        ),
        DeclareLaunchArgument(
            'lidar_ip', default_value='192.168.1.62',
            description='Unitree UDP address from the main launch profile.',
        ),
        DeclareLaunchArgument(
            'local_ip', default_value='192.168.1.2',
            description='Existing host UDP address. Launch does not configure networking.',
        ),
        OpaqueFunction(function=_setup),
    ])
