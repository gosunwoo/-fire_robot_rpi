"""Mode 6 field profile: Mode 2 route plus Space-triggered smoke assistance."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    arguments = [
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('use_serial', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_bno055_imu', default_value='true'),
        DeclareLaunchArgument('bno055_i2c_bus', default_value='1'),
        DeclareLaunchArgument('bno055_i2c_address', default_value='40'),
        DeclareLaunchArgument('drive_speed', default_value='0.06'),
        DeclareLaunchArgument('turn_speed', default_value='0.35'),
        DeclareLaunchArgument(
            'map_yaml', default_value=project_path('maps', 'inno_map_raw.yaml')
        ),
        DeclareLaunchArgument(
            'planning_map_yaml',
            default_value=project_path('maps', 'inno_map_nav.yaml'),
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=project_path('maps', 'waypoint_queue_latest.yaml'),
        ),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.30'),
        DeclareLaunchArgument('imu_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_yaw', default_value='0.0'),
        DeclareLaunchArgument('ultrasonic_x', default_value='0.20'),
        DeclareLaunchArgument('ultrasonic_y', default_value='0.0'),
        DeclareLaunchArgument('ultrasonic_z', default_value='0.20'),
        DeclareLaunchArgument('ultrasonic_yaw', default_value='0.0'),
    ]
    field = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bringup + '/launch/field_waypoint_test.launch.py'
        ),
        launch_arguments={
            'esp32_port': L('esp32_port'),
            'lidar_port': L('lidar_port'),
            'use_serial': L('use_serial'),
            'use_rviz': L('use_rviz'),
            'rviz_config': bringup + '/rviz/inno_mode6.rviz',
            'use_lidar': 'true',
            'use_mmwave': 'false',
            'use_camera_mode4': 'false',
            'use_mode3_audio': 'false',
            'use_thermal_sensor': 'false',
            'start_thermal_viewer': 'false',
            'mode6_enabled': 'true',
            'use_bno055_imu': L('use_bno055_imu'),
            'bno055_i2c_bus': L('bno055_i2c_bus'),
            'bno055_i2c_address': L('bno055_i2c_address'),
            'map_yaml': L('map_yaml'),
            'planning_map_yaml': L('planning_map_yaml'),
            'waypoint_file': L('waypoint_file'),
            'drive_speed': L('drive_speed'),
            'turn_speed': L('turn_speed'),
            'use_dynamic_obstacles': 'true',
            # Mode 6 uses A*'s existing dirty-grid periodic replan for the
            # active Mode 2 goal. It does not require Mode 5's evacuation plan.
            'event_replanning_enabled': 'false',
            'waypoint_planning_enabled': 'false',
            'astar_accept_goal_pose': 'true',
            'hazard_belief_enabled': 'false',
            'imu_x': L('imu_x'), 'imu_y': L('imu_y'), 'imu_z': L('imu_z'),
            'imu_roll': L('imu_roll'), 'imu_pitch': L('imu_pitch'),
            'imu_yaw': L('imu_yaw'),
            'ultrasonic_x': L('ultrasonic_x'),
            'ultrasonic_y': L('ultrasonic_y'),
            'ultrasonic_z': L('ultrasonic_z'),
            'ultrasonic_yaw': L('ultrasonic_yaw'),
        }.items(),
    )
    return LaunchDescription(arguments + [field])
