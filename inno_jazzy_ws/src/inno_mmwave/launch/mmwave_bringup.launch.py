"""C4001 UART driver and conservative mobility classifier."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('inno_mmwave')
    args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('configure_sensor', default_value='true'),
        DeclareLaunchArgument("assist_check_sec", default_value="10.0"),
        DeclareLaunchArgument("filter_presence_hold_sec", default_value="1.2"),
        DeclareLaunchArgument("filter_acquire_min_hits", default_value="5"),
        DeclareLaunchArgument("filter_acquire_confirm_sec", default_value="0.50"),
        DeclareLaunchArgument("filter_relock_confirm_sec", default_value="2.00"),
        DeclareLaunchArgument("moving_speed_threshold_mps", default_value="0.10"),
        DeclareLaunchArgument("moving_confirm_samples", default_value="2"),
        DeclareLaunchArgument("moving_confirm_sec", default_value="0.08"),
        DeclareLaunchArgument("moving_hold_sec", default_value="3.0"),
        DeclareLaunchArgument('node_output', default_value='screen'),
    ]

    driver = Node(
        package='inno_mmwave',
        executable='c4001_node',
        name='c4001_node',
        output=L('node_output'),
        emulate_tty=True,
        parameters=[
            share + '/config/c4001.yaml',
            {
                'serial_port': L('serial_port'),
                "configure_on_start": ParameterValue(
                    L("configure_sensor"), value_type=bool
                ),
                "filter_presence_hold_sec": ParameterValue(
                    L("filter_presence_hold_sec"), value_type=float
                ),
                "filter_acquire_min_hits": ParameterValue(
                    L("filter_acquire_min_hits"), value_type=int
                ),
                "filter_acquire_confirm_sec": ParameterValue(
                    L("filter_acquire_confirm_sec"), value_type=float
                ),
                "filter_relock_confirm_sec": ParameterValue(
                    L("filter_relock_confirm_sec"), value_type=float
                ),
            },
        ],
    )
    mobility = Node(
        package='inno_mmwave',
        executable='mmwave_mobility',
        name='mmwave_mobility',
        output=L('node_output'),
        emulate_tty=True,
        parameters=[
            {
                "assist_check_sec": ParameterValue(
                    L("assist_check_sec"), value_type=float
                ),
                "moving_speed_threshold_mps": ParameterValue(
                    L("moving_speed_threshold_mps"), value_type=float
                ),
                "moving_confirm_samples": ParameterValue(
                    L("moving_confirm_samples"), value_type=int
                ),
                "moving_confirm_sec": ParameterValue(
                    L("moving_confirm_sec"), value_type=float
                ),
                "moving_hold_sec": ParameterValue(
                    L("moving_hold_sec"), value_type=float
                ),
            },
        ],
    )
    return LaunchDescription(args + [driver, mobility])
