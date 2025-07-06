import os
from launch import LaunchDescription
from launch.substitutions import Command 
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    servebot_description_dir=get_package_share_directory("servebot_description")
    robot_description=ParameterValue(Command([
        "xacro ",
        os.path.join(servebot_description_dir,"urdf","servebot.urdf.xacro"),
        " is_sim:=False",
        
        ]), value_type=str)
    
    robot_state_publisher_node=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description":robot_description,
             "use_sim_time":False},
             os.path.join(
                 get_package_share_directory("servebot_controller"),
                 "config",
                 "servebot_controller.yaml"
             )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager
        
    ])