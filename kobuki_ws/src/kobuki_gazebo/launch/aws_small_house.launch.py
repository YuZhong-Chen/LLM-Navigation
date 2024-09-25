from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        "launch_rviz",
        default_value="False",
        description="Launch rviz2, by default is False",
    ),
    DeclareLaunchArgument(
        "launch_gzclient",
        default_value="False",
        description="Launch gzclient, by default is False, which means headless mode",
    ),
    DeclareLaunchArgument(
        "spawn_kobuki",
        default_value="True",
        description="Spawn kobuki, by default is True",
    ),
]

def generate_launch_description():
    # Get gazebo world file path
    world_file = PathJoinSubstitution(
        [
            FindPackageShare("kobuki_gazebo"),
            "worlds/aws_small_house",
            "small_house.world",
        ],
    )

    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_gazebo"),
                "launch",
                "gazebo.launch.py",
            ],
        ),
        launch_arguments={
            "world_path": world_file,
            "GAZEBO_MODEL_PATH": get_package_share_directory("kobuki_gazebo") + "/models/aws_small_house",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_gzclient": LaunchConfiguration("launch_gzclient"),
            "spawn_kobuki": LaunchConfiguration("spawn_kobuki"),
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_gazebo)

    return ld
