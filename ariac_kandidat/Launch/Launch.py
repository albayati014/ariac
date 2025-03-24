import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='Ariac_kandidat',  # Your main package name
            executable='example_node.py',  # Your script name
            name='example_node',  # Name of the node
            output='screen'
        )
    ])

