from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def lifecycle_manager(name, managed_nodes):
    # A nav2 lifecycle_manager that will configure->activate the listed nodes.
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=name,
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': managed_nodes},   # MUST exactly match /<ns>/<name>
            {'bond_timeout': 0.0},
        ],
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

def make_drone_group(drone_ns: str, px4_ns: str, takeoff_height: float):
    # Control nodes (all are LifecycleNode-based in code)
    hb = Node(
        package="drone_basic_control",
        executable="hb_node",
        name="hb_node",
        namespace=drone_ns,
        parameters=[{"drone_ns": drone_ns}, {"px4_ns": px4_ns}],
        output="screen",
    )
    arming = Node(
        package="drone_basic_control",
        executable="arming_node",
        name="arming_node",
        namespace=drone_ns,
        parameters=[{"drone_ns": drone_ns}, {"px4_ns": px4_ns}],
        output="screen",
    )
    takeoff = Node(
        package="drone_basic_control",
        executable="takeoff_node",
        name="takeoff_node",
        namespace=drone_ns,
        parameters=[{"drone_ns": drone_ns}, {"px4_ns": px4_ns}, {"takeoff_height": float(takeoff_height)}],
        output="screen",
    )
    landing = Node(
        package="drone_basic_control",
        executable="landing_node",
        name="landing_node",
        namespace=drone_ns,
        parameters=[{"drone_ns": drone_ns}, {"px4_ns": px4_ns}],
        output="screen",
    )

    # Lifecycle manager that manages ONLY these four nodes
    lm = lifecycle_manager(
        name=f'{drone_ns}_lifecycle_manager',
        managed_nodes=[
            f'/{drone_ns}/hb_node',
            f'/{drone_ns}/arming_node',
            f'/{drone_ns}/takeoff_node',
            f'/{drone_ns}/landing_node',
        ],
    )
    return [hb, arming, takeoff, landing, lm]

def generate_launch_description():
    # Let you disable lifecycle autostart for debugging
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='true',
        description='Auto-start lifecycle nodes (configure -> activate)'
    )

    # Drone 1 (px4_1) and Drone 2 (px4_2)
    d1 = make_drone_group(drone_ns="drone_1", px4_ns="px4_1", takeoff_height=-2.0)
    d2 = make_drone_group(drone_ns="drone_2", px4_ns="px4_2", takeoff_height=-2.0)

    ld = LaunchDescription()
    ld.add_action(auto_start_arg)
    for n in (*d1, *d2):
        ld.add_action(n)
    return ld
