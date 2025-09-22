import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def run_launch_arguments():
    return [
        DeclareLaunchArgument(
            'auto_start', default_value='true', choices=['true', 'false'],
            description='Auto-start lifecycle nodes.'
        ),
        DeclareLaunchArgument(
            'manage_bt', default_value='false', choices=['true', 'false'],
            description='Include BT nodes in lifecycle management (only if they are LifecycleNodes).'
        ),
        DeclareLaunchArgument(
            'bt_node_name', default_value='drone_bt_navigator',
            description='Exact BT node name (must match ros2 node list if managed).'
        ),
    ]


def create_unified_lifecycle_manager(*node_names: str):
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='unified_lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': LaunchConfiguration('auto_start')},
            {'node_names': list(node_names)},  # fully qualified names required
            {'bond_timeout': 0.0},
        ],
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )


def run_lifecycle_actions():
    def _setup(context, *args, **kwargs):
        manage_bt = LaunchConfiguration('manage_bt').perform(context).lower() == 'true'
        bt_node_name = LaunchConfiguration('bt_node_name').perform(context)

        # Flight control nodes are LifecycleNodes
        fc_1 = 'drone_1/flight_control_node'
        fc_2 = 'drone_2/flight_control_node'

        nodes_d1 = [fc_1]
        nodes_d2 = [fc_2]

        # Only manage BT if you made it a LifecycleNode and its name matches
        if manage_bt:
            nodes_d1.append(f'drone_1/{bt_node_name}')
            nodes_d2.append(f'drone_2/{bt_node_name}')

        log_action = LogInfo(msg=f"Lifecycle manager will manage: {nodes_d1 + nodes_d2}")

        manager_1 = create_unified_lifecycle_manager(*nodes_d1)
        manager_2 = create_unified_lifecycle_manager(*nodes_d2)
        return [log_action, manager_1, manager_2]

    return [OpaqueFunction(function=_setup)]


def launcher_flightcontrol_node_manager(pkg_share_dir):
    fc_inc_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_flightctrl_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_1'}.items()
    )
    fc_inc_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_flightctrl_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_2'}.items()
    )
    return [fc_inc_1, fc_inc_2]


def launcher_bt_navigator_manager(pkg_share_dir):
    bt_inc_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_bt_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_1'}.items()
    )
    bt_inc_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch_bt_node.launch.py')),
        launch_arguments={'drone_ns': 'drone_2'}.items()
    )
    return [bt_inc_1, bt_inc_2]


def generate_launch_description():
    share_path_flight_control = get_package_share_directory('flight_control')
    share_path_bt_navigator = get_package_share_directory('bt_navigator')

    args = run_launch_arguments()
    fc_includes = launcher_flightcontrol_node_manager(share_path_flight_control)
    bt_includes = launcher_bt_navigator_manager(share_path_bt_navigator)
    lifecycle_actions = run_lifecycle_actions()

    gm_node = Node(
        package="goal_manager",
        executable="goal_manager_node",
        name="goal_manager_node",
        output="screen",
    )

    return LaunchDescription(args + fc_includes + bt_includes + lifecycle_actions + [gm_node])
