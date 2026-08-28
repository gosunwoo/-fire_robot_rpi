"""One-command field test: LiDAR localization, path, keyboard, A*, follower, ESP32."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    drive = get_package_share_directory('inno_drive_bridge')
    autonav = get_package_share_directory('inno_autonav')
    camera = get_package_share_directory('inno_camera_tools')
    mmwave = get_package_share_directory('inno_mmwave')
    thermal_dir = project_path(
        'mlx90640', 'demo codes', 'mlx90640', 'python'
    )
    args = [
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('mmwave_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('mmwave_configure_sensor', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_mmwave', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=bringup + '/rviz/inno_slam.rviz'
        ),
        DeclareLaunchArgument('assist_check_sec', default_value='10.0'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('auto_localization', default_value='true'),
        DeclareLaunchArgument(
            'auto_localization_timeout_sec', default_value='45.0'
        ),
        DeclareLaunchArgument(
            'auto_localization_minimum_overlap', default_value='0.65'
        ),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=project_path('maps', 'inno_map_raw.yaml'),
        ),
        DeclareLaunchArgument(
            'planning_map_yaml',
            default_value=project_path('maps', 'inno_map_nav.yaml'),
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=project_path('maps', 'waypoint_queue_latest.yaml'),
        ),
        # The RViz waypoint queue above stores ``poses`` as a list.  The A*
        # reference graph and waypoint planner require stable named poses
        # (w1, w2, ...), so the two consumers must not share one file.
        DeclareLaunchArgument(
            'planner_waypoint_file',
            default_value=project_path(
                'docs', 'full_map_waypoints_1m_numbered.yaml'
            ),
        ),
        DeclareLaunchArgument('drive_speed', default_value='0.12'),
        DeclareLaunchArgument('turn_speed', default_value='0.45'),
        DeclareLaunchArgument('use_dynamic_obstacles', default_value='true'),
        # Modes 1-4 must remain testable before the optional MLX90640 is
        # installed.  Mode 5 supplies its own live hazard pipeline and does
        # not use these fallbacks.
        DeclareLaunchArgument('require_thermal_grid', default_value='false'),
        DeclareLaunchArgument('require_thermal_active', default_value='false'),
        DeclareLaunchArgument('hazard_belief_enabled', default_value='false'),
        DeclareLaunchArgument('hazard_thermal_enabled', default_value='true'),
        DeclareLaunchArgument('exit_evaluator_enabled', default_value='false'),
        DeclareLaunchArgument('evacuation_manager_enabled', default_value='false'),
        DeclareLaunchArgument(
            'evacuation_activate_selected_route', default_value='false'
        ),
        DeclareLaunchArgument('event_replanning_enabled', default_value='false'),
        DeclareLaunchArgument('exit_switching_enabled', default_value='false'),
        DeclareLaunchArgument('waypoint_planning_enabled', default_value='false'),
        DeclareLaunchArgument(
            'waypoint_accept_direct_goal', default_value='false'
        ),
        DeclareLaunchArgument('astar_accept_goal_pose', default_value='true'),
        DeclareLaunchArgument(
            'mode3_standoff_distance_m', default_value='2.0'
        ),
        DeclareLaunchArgument(
            'mode3_publish_canonical_plan', default_value='false'
        ),
        DeclareLaunchArgument(
            'mode4_standoff_distance_m', default_value='1.5'
        ),
        DeclareLaunchArgument(
            'mode4_publish_canonical_plan', default_value='false'
        ),
        DeclareLaunchArgument('use_serial', default_value='true'),
        DeclareLaunchArgument('use_camera_mode4', default_value='false'),
        DeclareLaunchArgument('use_mode3_audio', default_value='true'),
        DeclareLaunchArgument(
            'mode3_audio_directory', default_value='~/fire_robot_audio'
        ),
        DeclareLaunchArgument('mode3_audio_device', default_value='auto'),
        DeclareLaunchArgument(
            'mode3_audio_volume_percent', default_value='100'
        ),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        DeclareLaunchArgument('use_thermal_sensor', default_value='false'),
        DeclareLaunchArgument('mode5_enabled', default_value='false'),
        DeclareLaunchArgument('mode6_enabled', default_value='false'),
        DeclareLaunchArgument('use_bno055_imu', default_value='false'),
        DeclareLaunchArgument('bno055_i2c_bus', default_value='1'),
        DeclareLaunchArgument('bno055_i2c_address', default_value='40'),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.30'),
        DeclareLaunchArgument('imu_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_yaw', default_value='0.0'),
        DeclareLaunchArgument('ultrasonic_x', default_value='0.20'),
        DeclareLaunchArgument('ultrasonic_y', default_value='0.0'),
        DeclareLaunchArgument('ultrasonic_z', default_value='0.20'),
        DeclareLaunchArgument('ultrasonic_roll', default_value='0.0'),
        DeclareLaunchArgument('ultrasonic_pitch', default_value='0.0'),
        DeclareLaunchArgument('ultrasonic_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=project_path(
                'models', 'yolov8n_best_opencv_640.onnx'
            ),
        ),
        DeclareLaunchArgument('yolo_confidence', default_value='0.40'),
        DeclareLaunchArgument('yolo_inference_rate_hz', default_value='3.0'),
        DeclareLaunchArgument(
            'yolo_only_during_mode4_observation', default_value='false'
        ),
        DeclareLaunchArgument('start_thermal_viewer', default_value='true'),
    ]
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup + '/launch/lidar_amcl_localization.launch.py'),
        launch_arguments={
            'start_lidar': L('use_lidar'),
            'serial_port': L('lidar_port'), 'map_yaml': L('map_yaml'),
            'set_initial_pose': L('set_initial_pose'),
            'initial_pose_x': L('initial_pose_x'),
            'initial_pose_y': L('initial_pose_y'),
            'initial_pose_yaw': L('initial_pose_yaw'),
        }.items(),
    )
    mmwave_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mmwave + '/launch/mmwave_bringup.launch.py'),
        launch_arguments={
            'serial_port': L('mmwave_port'),
            'configure_sensor': L('mmwave_configure_sensor'),
            'assist_check_sec': L('assist_check_sec'),
        }.items(),
        condition=IfCondition(L('use_mmwave')),
    )
    auto_localization = Node(
        package='inno_robot_bringup',
        executable='auto_localization_supervisor',
        name='auto_localization_supervisor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'startup_timeout_sec': ParameterValue(
                L('auto_localization_timeout_sec'), value_type=float
            ),
            'minimum_scan_overlap_ratio': ParameterValue(
                L('auto_localization_minimum_overlap'), value_type=float
            ),
        }],
        condition=IfCondition(L('auto_localization')),
    )
    status_console = Node(
        package='inno_mmwave', executable='mmwave_status_console',
        name='mmwave_status_console', output='screen', emulate_tty=True,
        parameters=[{
            'use_serial': ParameterValue(L('use_serial'), value_type=bool),
            'use_lidar': ParameterValue(L('use_lidar'), value_type=bool),
            'use_mmwave': ParameterValue(L('use_mmwave'), value_type=bool),
            'use_camera': ParameterValue(
                L('use_camera_mode4'), value_type=bool
            ),
            'use_thermal': ParameterValue(
                L('use_thermal_sensor'), value_type=bool
            ),
            'mode5_enabled': ParameterValue(
                L('mode5_enabled'), value_type=bool
            ),
            'esp32_port': L('esp32_port'),
            'lidar_port': L('lidar_port'),
            'mmwave_port': L('mmwave_port'),
        }],
    )
    camera_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            camera + '/launch/camera_module_3.launch.py'
        ),
        launch_arguments={
            'width': L('camera_width'),
            'height': L('camera_height'),
            'rectify': 'false',
        }.items(),
        condition=IfCondition(L('use_camera_mode4')),
    )
    person_detector = Node(
        package='inno_camera_tools',
        executable='camera_person_detector',
        name='camera_person_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'model_path': L('yolo_model_path'),
            'confidence_threshold': ParameterValue(
                L('yolo_confidence'), value_type=float
            ),
            'inference_rate_hz': ParameterValue(
                L('yolo_inference_rate_hz'), value_type=float
            ),
            'only_during_mode4_observation': ParameterValue(
                L('yolo_only_during_mode4_observation'), value_type=bool
            ),
        }],
        condition=IfCondition(L('use_camera_mode4')),
    )
    mode3_audio = Node(
        package='inno_robot_bringup',
        executable='mode3_audio_guide',
        name='mode3_audio_guide',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'enabled': True,
            'audio_directory': L('mode3_audio_directory'),
            'audio_device': L('mode3_audio_device'),
            'playback_volume_percent': ParameterValue(
                L('mode3_audio_volume_percent'), value_type=int
            ),
        }],
        condition=IfCondition(L('use_mode3_audio')),
    )
    bno055_imu = Node(
        package='inno_robot_bringup',
        executable='bno055_imu',
        name='bno055_imu',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'i2c_bus': ParameterValue(L('bno055_i2c_bus'), value_type=int),
            'i2c_address': ParameterValue(
                L('bno055_i2c_address'), value_type=int
            ),
        }],
        condition=IfCondition(L('use_bno055_imu')),
    )
    imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '--x', L('imu_x'), '--y', L('imu_y'), '--z', L('imu_z'),
            '--roll', L('imu_roll'), '--pitch', L('imu_pitch'),
            '--yaw', L('imu_yaw'), '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
        condition=IfCondition(L('use_bno055_imu')),
    )
    ultrasonic_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_ultrasonic_front_tf',
        arguments=[
            '--x', L('ultrasonic_x'), '--y', L('ultrasonic_y'),
            '--z', L('ultrasonic_z'), '--roll', L('ultrasonic_roll'),
            '--pitch', L('ultrasonic_pitch'), '--yaw', L('ultrasonic_yaw'),
            '--frame-id', 'base_link', '--child-frame-id', 'ultrasonic_front',
        ],
        condition=IfCondition(L('mode6_enabled')),
    )
    mode6_assist = Node(
        package='inno_autonav',
        executable='mode6_smoke_assist',
        name='mode6_smoke_assist',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(L('mode6_enabled')),
    )
    # autonav_demo also declares ``use_serial``.  Keep the include scoped so
    # its deliberately disabled internal bridge cannot overwrite this launch
    # file's top-level ``use_serial`` value and suppress the ESP32 bridge.
    navigation = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                autonav + '/launch/autonav_demo.launch.py'
            ),
            launch_arguments={
                'use_serial': 'false', 'use_wheel_odom_tf': 'false',
                'map_yaml': L('planning_map_yaml'),
                'max_linear_speed': L('drive_speed'),
                'max_angular_speed': L('turn_speed'),
                'use_dynamic_obstacles': L('use_dynamic_obstacles'),
                'require_thermal_grid': L('require_thermal_grid'),
                'require_thermal_active': L('require_thermal_active'),
                'waypoint_file': L('planner_waypoint_file'),
                'hazard_belief_enabled': L('hazard_belief_enabled'),
                'hazard_thermal_enabled': L('hazard_thermal_enabled'),
                'exit_evaluator_enabled': L('exit_evaluator_enabled'),
                'evacuation_manager_enabled': L(
                    'evacuation_manager_enabled'
                ),
                'evacuation_activate_selected_route': L(
                    'evacuation_activate_selected_route'
                ),
                'event_replanning_enabled': L('event_replanning_enabled'),
                'exit_switching_enabled': L('exit_switching_enabled'),
                'waypoint_planning_enabled': L(
                    'waypoint_planning_enabled'
                ),
                'waypoint_accept_direct_goal': L(
                    'waypoint_accept_direct_goal'
                ),
                'astar_accept_goal_pose': L('astar_accept_goal_pose'),
                'mode3_standoff_distance_m': L(
                    'mode3_standoff_distance_m'
                ),
                'mode3_publish_canonical_plan': L(
                    'mode3_publish_canonical_plan'
                ),
                'mode4_standoff_distance_m': L(
                    'mode4_standoff_distance_m'
                ),
                'mode4_publish_canonical_plan': L(
                    'mode4_publish_canonical_plan'
                ),
                # Keep the detector and inspector on exactly the same
                # confidence threshold.  Otherwise the detector can publish a
                # valid box which Mode 4 silently filters out again.
                'mode4_minimum_confidence': L('yolo_confidence'),
                'require_localization_ready': L('auto_localization'),
            }.items(),
        )],
    )
    keyboard = Node(
        package='inno_drive_bridge', executable='keyboard_cmdvel_demo',
        name='keyboard_cmdvel_demo', output='log', emulate_tty=True,
        parameters=[
            drive + '/config/drive_params.yaml',
            {
                'linear_speed': ParameterValue(
                    L('drive_speed'), value_type=float
                ),
                'angular_speed': ParameterValue(
                    L('turn_speed'), value_type=float
                ),
                'mode6_enabled': ParameterValue(
                    L('mode6_enabled'), value_type=bool
                ),
            },
        ],
    )
    mux = Node(package='inno_drive_bridge', executable='cmd_vel_mode_mux',
               name='cmd_vel_mode_mux', output='log')
    serial = Node(
        package='inno_drive_bridge', executable='cmdvel_to_esp32_serial',
        name='cmdvel_to_esp32_serial', output='log',
        parameters=[drive + '/config/drive_params.yaml', {'serial_port': L('esp32_port')}],
        condition=IfCondition(L('use_serial')),
    )
    waypoint_queue = Node(
        package='inno_autonav', executable='waypoint_queue',
        name='waypoint_queue', output='log',
        parameters=[{
            'load_file': L('waypoint_file'),
            'save_file': L('waypoint_file'),
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', L('rviz_config')],
        condition=IfCondition(L('use_rviz')),
    )
    thermal_viewer = ExecuteProcess(
        cmd=['python3', thermal_dir + '/mlx90640.py'],
        cwd=thermal_dir,
        name='mlx90640_viewer',
        output='log',
        condition=IfCondition(L('start_thermal_viewer')),
    )
    return LaunchDescription(
        args + [
            localization, auto_localization, mmwave_bringup, status_console,
            camera_bringup,
            person_detector, mode3_audio, bno055_imu, imu_transform,
            ultrasonic_transform, mode6_assist, navigation, keyboard, mux, serial,
            waypoint_queue, rviz, thermal_viewer,
        ]
    )
