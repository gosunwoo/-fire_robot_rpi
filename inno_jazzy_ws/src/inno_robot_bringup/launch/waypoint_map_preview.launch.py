"""Hardware-free RViz preview of a saved map, waypoint queue, and A* path."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    autonav = get_package_share_directory('inno_autonav')
    args = [
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
        DeclareLaunchArgument('waypoint_save_file', default_value=''),
        DeclareLaunchArgument('start_x', default_value='4.817799091339111'),
        DeclareLaunchArgument('start_y', default_value='-9.854209899902344'),
        DeclareLaunchArgument('start_yaw', default_value='-0.4439678115329046'),
        DeclareLaunchArgument('preview_goal_index', default_value='0'),
        DeclareLaunchArgument('replace_waypoint_numbers', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'yaml_filename': L('map_yaml')}],
    )
    lifecycle = Node(
        package='inno_robot_bringup', executable='lifecycle_autostart',
        name='lifecycle_autostart_preview', output='screen',
        parameters=[{'node_names': ['map_server']}],
    )
    planning_grid = Node(
        package='inno_autonav', executable='planning_grid_publisher',
        name='planning_grid_publisher', output='screen',
        parameters=[
            autonav + '/config/autonav_params.yaml',
            {'map_yaml': L('planning_map_yaml')},
        ],
    )
    planner = Node(
        package='inno_autonav', executable='astar_replanner',
        name='astar_replanner', output='screen',
        parameters=[autonav + '/config/autonav_params.yaml'],
    )
    waypoints = Node(
        package='inno_autonav', executable='waypoint_queue',
        name='waypoint_queue', output='screen',
        parameters=[
            {
                'load_file': L('waypoint_file'),
                'save_file': L('waypoint_save_file'),
                "replace_waypoint_numbers": ParameterValue(
                    L("replace_waypoint_numbers"), value_type=str
                ),
                'preview_goal_index': ParameterValue(
                    L('preview_goal_index'), value_type=int
                ),
            }
        ],
    )
    static_pose = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='preview_map_to_base_link', output='screen',
        arguments=[
            '--x', L('start_x'), '--y', L('start_y'), '--z', '0',
            '--yaw', L('start_yaw'), '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'base_link',
        ],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='waypoint_preview_rviz',
        output='screen', arguments=['-d', bringup + '/rviz/inno_slam.rviz'],
        condition=IfCondition(L('use_rviz')),
    )
    return LaunchDescription(
        args + [
            map_server, lifecycle, planning_grid, planner, waypoints,
            static_pose, rviz,
        ]
    )
