from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Launch 1: Navigation stack ---
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tutorial_pkg"), "launch", "navigation.launch.py"]
            )
        )
    )

    # --- Launch 2: Gazebo simulation and Rviz ---
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbot_gazebo"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={"robot_model": "rosbot"}.items(),
    )


    return LaunchDescription([
        LogInfo(msg="🧭 Launching navigation stack..."),
        navigation_launch,
        LogInfo(msg="⏳ Starting simulation..."),
        simulation_launch,
       
    ])
