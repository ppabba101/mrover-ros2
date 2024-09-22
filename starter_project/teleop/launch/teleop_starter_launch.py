from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    teleop_backend = Node(
            package="mrover",
            executable="gui_starter_backend.sh",
            name="teleop_backend"
    )

    teleop_frontend = Node(
            package="mrover",
            executable="gui_starter_frontend.sh",
            name="teleop_frontend"
    )

    return LaunchDescription([
        teleop_backend,
        teleop_frontend,

    ])