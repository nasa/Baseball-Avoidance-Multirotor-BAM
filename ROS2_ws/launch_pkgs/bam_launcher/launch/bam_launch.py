from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the drone plotter node
        Node(
            package='drone_plotter_py',
            executable='drone_plotter',
            name='drone_plotter',
            output='screen'
        ),
        
        # Launch the phase space warping node
        Node(
            package='phase_space_warping_py',
            executable='psw_node',
            name='phase_space_warping',
            output='screen'
        )
    ])
