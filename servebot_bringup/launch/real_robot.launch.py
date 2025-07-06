import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    
    hardware_interface=IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("servebot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        )
    )
    
    controller=IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("servebot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
             "use_simple_controller": "True", #change to false to use joystck
            "use_python": "True"
            }.items()
    )
    
    
    return LaunchDescription([
        hardware_interface,
        controller
    ])