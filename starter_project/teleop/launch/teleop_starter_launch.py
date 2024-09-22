import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('mrover')
    scripts_directory = os.path.join(package_share_directory, 'starter_project/teleop')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[os.path.join(scripts_directory, 'gui_starter_frontend.sh')],
            cwd=scripts_directory,
            name='gui_frontend',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.join(scripts_directory, 'gui_starter_backend.sh')],
            cwd=scripts_directory,
            name='gui_backend',
            output='screen'
        ),
    ])