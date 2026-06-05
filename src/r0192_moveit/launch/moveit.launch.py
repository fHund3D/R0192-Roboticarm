import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    use_rviz = LaunchConfiguration("use_rviz")
    use_servo = LaunchConfiguration("use_servo")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz with MoveIt plugin"
    )

    use_servo_arg = DeclareLaunchArgument(
        "use_servo",
        default_value="true",
        description="Launch MoveIt Servo node for teach-pendant jogging (JogPanel)"
    )

    moveit_config = (
        MoveItConfigsBuilder("r0192", package_name="r0192_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("r0192_description"),
                "urdf",
                "r0192.urdf.xacro"
            ),
            mappings={"is_sim": "false", "is_ignition": "false"},
        )
        .robot_description_semantic(file_path="config/r0192.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config = os.path.join(
        get_package_share_directory("r0192_moveit"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    # MoveIt Servo node — teach-pendant jogging backend for r0192_rviz_plugins/
    # JogPanel. Starts idle (command type INVALID + paused by the panel) so it
    # does not interfere with move_group planning until the operator jogs.
    servo_params = {
        "moveit_servo": ParameterBuilder("r0192_moveit")
        .yaml("config/servo.yaml")
        .to_dict()
    }
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": is_sim},
        ],
        condition=IfCondition(use_servo),
    )

    return LaunchDescription([
        is_sim_arg,
        use_rviz_arg,
        use_servo_arg,
        move_group_node,
        rviz_node,
        servo_node,
    ])
