from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        "launch_rviz",
        default_value="False",
        description="Launch rviz2, by default is False",
    ),
    DeclareLaunchArgument(
        "launch_gzclient",
        default_value="True",
        description="Launch gzclient, by default is False, which means headless mode",
    ),
    DeclareLaunchArgument(
        "spawn_kobuki",
        default_value="True",
        description="Spawn kobuki, by default is True",
    ),
]


def generate_launch_description():
    # Launch Kobuki's description.
    launch_kobuki_description = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_description"),
                "launch",
                "robot_description.launch.py",
            ]
        ),
        condition=UnlessCondition(LaunchConfiguration("spawn_kobuki")),
    )

    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("kobuki_gazebo"), "launch", "aws_small_house.launch.py"],
        ),
        launch_arguments={
            "spawn_kobuki": LaunchConfiguration("spawn_kobuki"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_gzclient": LaunchConfiguration("launch_gzclient"),
        }.items(),
    )

    # Launch Gazebo RL env controller
    launch_gazebo_rl_env = Node(
        package="gazebo_rl_env",
        executable="main.py",
        name="gazebo_rl_env",
        output="screen",
    )

    # Launch command rqt_robot_steering
    launch_robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        name="rqt_robot_steering",
        output="screen",
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_kobuki_description)
    ld.add_action(launch_gazebo)
    ld.add_action(launch_robot_steering)
    # ld.add_action(launch_gazebo_rl_env)

    return ld
