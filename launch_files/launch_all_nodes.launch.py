from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def lifecycle_manager(name, nodes):
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=name,
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': nodes},
            {'bond_timeout': 0.0},
        ],
        condition=IfCondition(LaunchConfiguration('auto_start'))
    )


def generate_launch_description():
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='true',
        description='Auto-start lifecycle nodes.'
    )

    # Drone 1
    hb_node_1 = Node(
        package="drone_basic_control", executable="hb_node",
        name="hb_node", namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}], output="screen",
    )
    arming_node_1 = Node(
        package="drone_basic_control", executable="arming_node",
        name="arming_node", namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}], output="screen",
    )
    takeoff_node_1 = Node(
        package="drone_basic_control", executable="takeoff_node",
        name="takeoff_node", namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}, {"takeoff_height": -2.0}],
        output="screen",
    )
    landing_node_1 = Node(
        package="drone_basic_control", executable="landing_node",
        name="landing_node", namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}], output="screen",
    )
    lm_drone1 = lifecycle_manager(
        'lm_drone1',
        ['drone_1/hb_node', 'drone_1/arming_node', 'drone_1/takeoff_node', 'drone_1/landing_node'],
    )

    # Drone 2
    hb_node_2 = Node(
        package="drone_basic_control", executable="hb_node",
        name="hb_node", namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}], output="screen",
    )
    arming_node_2 = Node(
        package="drone_basic_control", executable="arming_node",
        name="arming_node", namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}], output="screen",
    )
    takeoff_node_2 = Node(
        package="drone_basic_control", executable="takeoff_node",
        name="takeoff_node", namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}, {"takeoff_height": -2.0}],
        output="screen",
    )
    landing_node_2 = Node(
        package="drone_basic_control", executable="landing_node",
        name="landing_node", namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}], output="screen",
    )
    lm_drone2 = lifecycle_manager(
        'lm_drone2',
        ['drone_2/hb_node', 'drone_2/arming_node', 'drone_2/takeoff_node', 'drone_2/landing_node'],
    )

    ld = LaunchDescription()
    ld.add_action(auto_start_arg)
    for n in (
        hb_node_1, arming_node_1, takeoff_node_1, landing_node_1, lm_drone1,
        hb_node_2, arming_node_2, takeoff_node_2, landing_node_2, lm_drone2,
    ):
        ld.add_action(n)
    return ld
