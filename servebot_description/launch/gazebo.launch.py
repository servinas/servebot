import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , SetEnvironmentVariable ,IncludeLaunchDescription
from launch.substitutions import Command , LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    servebot_description_dir=get_package_share_directory("servebot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    model_arg=DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(servebot_description_dir,"urdf","servebot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )
    
    robot_description=ParameterValue(Command(["xacro ",LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher_node=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    
    gazebo_resource_path=SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            str(Path(servebot_description_dir).parent.resolve())
            ]
    )
    
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("gazebo_ros"),"launch"
        ),"/gz_sim.launch.py"]),
        launch_arguments=[
            ("gz_args",[" -v 4"," -r"," empty.sdf"])
        ]
        )
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )
    gz_spawn_entity=Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic","robot_description","-entity","servebot"]
    )
    
    return LaunchDescription([
     model_arg,
     robot_state_publisher_node,
     gazebo_resource_path,
     start_gazebo_server,
     start_gazebo_client,
     gz_spawn_entity
     ])