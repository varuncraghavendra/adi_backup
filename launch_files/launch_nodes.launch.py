import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def run_launch_arguments():
    args = [
        DeclareLaunchArgument(
            'auto_start', default_value='true', choices=['true', 'false'],
            description='Auto-start lifecycle nodes.'
        )
    ]
    return args


def create_unified_lifecycle_manager(*node_names: str):
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='unified_lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': LaunchConfiguration('auto_start')},
            {'node_names': list(node_names)},
            {'bond_timeout': 0.0}
        ],
        condition=IfCondition(LaunchConfiguration('auto_start'))
    )
    return lifecycle_manager_node


def run_lifecycle_actions():
    def _setup(context, *args, **kwargs):
        # Per-drone managed nodes
        bt_nodes_1 = "drone_1/drone_bt_navigator"
        fc_nodes_1 = "drone_1/flight_control_node"

        bt_nodes_2 = "drone_2/drone_bt_navigator"
        fc_nodes_2 = "drone_2/flight_control_node"

        log_action = LogInfo(msg=f"Lifecycle managers will manage: {[bt_nodes_1, fc_nodes_1, bt_nodes_2, fc_nodes_2]}")

        manager_1 = create_unified_lifecycle_manager(bt_nodes_1, fc_nodes_1)
        manager_2 = create_unified_lifecycle_manager(bt_nodes_2, fc_nodes_2)

        return [log_action, manager_1, manager_2]

    return [OpaqueFunction(function=_setup)]


def launcher_flightcontrol_node_manager(pkg_share_dir):
    # Assumes your included launch reads a 'drone_ns' LaunchArg
    fc_include_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_flightctrl_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_1'}.items()
    )
    fc_include_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_flightctrl_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_2'}.items()
    )
    return [fc_include_1, fc_include_2]


def launcher_bt_navigator_manager(pkg_share_dir):
    # Assumes your included launch reads a 'drone_ns' LaunchArg
    bt_include_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_bt_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_1'}.items()
    )
    bt_include_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_bt_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_2'}.items()
    )
    return [bt_include_1, bt_include_2]


def generate_launch_description():
    share_path_flight_control = get_package_share_directory('flight_control')
    share_path_bt_navigator = get_package_share_directory('bt_navigator')

    fc_includes = launcher_flightcontrol_node_manager(share_path_flight_control)
    bt_includes = launcher_bt_navigator_manager(share_path_bt_navigator)

    lifecycle_actions = run_lifecycle_actions()
    args = run_launch_arguments()

    # One goal manager that publishes to both drones
    gm_node = Node(
        package="goal_manager",
        executable="goal_manager_node",
        name="goal_manager_node"
    )

    return LaunchDescription(args + fc_includes + bt_includes + lifecycle_actions + [gm_node])
