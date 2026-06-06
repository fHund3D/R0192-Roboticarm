"""
Bringup für den echten R0192 Roboterarm.

Startet:
  1. robot_state_publisher   – publiziert TF-Baum aus URDF
  2. ros2_control_node       – Controller Manager + r0192_hardware Plugin (CAN)
  3. joint_state_broadcaster – publiziert /joint_states
  4. arm_controller          – JointTrajectoryController für Achsen 1-6
  5. gripper_controller      – JointTrajectoryController für Greifer
  6. move_group              – MoveIt 2 Bewegungsplanung
  7. rviz2                   – RViz mit MoveIt-Plugin (nur wenn use_rviz:=true)
  8. foxglove_bridge         – Native WebSocket-Bridge für Foxglove Studio (nur wenn use_foxglove:=true)

Launch-Argumente:
  use_rviz:=true|false       Standard: true  – RViz mit MoveIt-Plugin
  use_foxglove:=true|false   Standard: false – foxglove_bridge Server (Port 8765)

Voraussetzung:
  sudo ip link set can0 up type can bitrate 1000000
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz with MoveIt plugin (set false for Foxglove mode)"
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="false",
        description="Launch foxglove_bridge server on port 8765 for Foxglove Studio"
    )

    r0192_description_share = get_package_share_directory("r0192_description")
    r0192_controller_share  = get_package_share_directory("r0192_controller")
    r0192_moveit_share      = get_package_share_directory("r0192_moveit")

    # URDF via xacro (is_sim=false → r0192_hardware Plugin)
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(r0192_description_share, "urdf", "r0192.urdf.xacro"),
            " is_sim:=false is_ignition:=false",
        ]),
        value_type=str,
    )

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )

    # 2. Controller Manager (lädt r0192_hardware Plugin via CAN)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            os.path.join(r0192_controller_share, "config", "r0192_controllers.yaml"),
        ],
    )

    # 3-5. Controller-Spawner (starten erst wenn controller_manager bereit ist)
    #      Die Spawner pollen automatisch auf den Controller Manager.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 5b. Robot State Manager – zentrale Zustandsmaschine (DISABLED/HOLD/JOG/
    #     MOVEIT/HOMING). Single source of truth; deaktiviert arm_controller beim
    #     Start (Arm steht drehmomentfrei in DISABLED). Tolerant gegen späte
    #     Services, daher kein TimerAction nötig.
    robot_state_manager = Node(
        package="r0192_hardware",
        executable="robot_state_manager",
        name="r0192_state_manager",
        output="screen",
    )

    # 6-7. MoveIt + RViz (mit leichter Verzögerung damit Controller erst stabil sind)
    moveit_and_rviz = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                os.path.join(r0192_moveit_share, "launch", "moveit.launch.py"),
                launch_arguments={
                    "is_sim": "False",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                }.items(),
            )
        ],
    )

    # 8. Foxglove Bridge – Native ROS 2 WebSocket-Bridge, Port 8765
    #    Verbindungstyp in Foxglove Studio: "Foxglove WebSocket" → ws://<ip>:8765
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "use_sim_time": False,
        }],
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_foxglove_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        robot_state_manager,
        moveit_and_rviz,
        foxglove_bridge,
    ])
