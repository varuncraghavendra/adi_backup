from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Drone 1
    hb_node_1 = Node(
        package="drone_basic_control",
        executable="hb_node",
        name="hb_node",
        namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}],
    )
    arming_node_1 = Node(
        package="drone_basic_control",
        executable="arming_node",
        name="arming_node",
        namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}],
    )
    landing_node_1 = Node(
        package="drone_basic_control",
        executable="landing_node",
        name="landing_node",
        namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}],
    )
    takeoff_node_1 = Node(
        package="drone_basic_control",
        executable="takeoff_node",
        name="takeoff_node",
        namespace="drone_1",
        parameters=[{"drone_ns": "drone_1"}, {"px4_ns": "px4_1"}, {"takeoff_height": -3.0}],
    )

    # Drone 2
    hb_node_2 = Node(
        package="drone_basic_control",
        executable="hb_node",
        name="hb_node",
        namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}],
    )
    arming_node_2 = Node(
        package="drone_basic_control",
        executable="arming_node",
        name="arming_node",
        namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}],
    )
    landing_node_2 = Node(
        package="drone_basic_control",
        executable="landing_node",
        name="landing_node",
        namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}],
    )
    takeoff_node_2 = Node(
        package="drone_basic_control",
        executable="takeoff_node",
        name="takeoff_node",
        namespace="drone_2",
        parameters=[{"drone_ns": "drone_2"}, {"px4_ns": "px4_2"}, {"takeoff_height": -3.0}],
    )

    for n in [arming_node_1, landing_node_1, takeoff_node_1, hb_node_1,
              arming_node_2, landing_node_2, takeoff_node_2, hb_node_2]:
        ld.add_action(n)

    return ld
