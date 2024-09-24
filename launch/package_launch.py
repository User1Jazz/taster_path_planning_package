import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planning_package',
            executable='path_planning',
            name='pathPlannerControl',
            arguments=['--ros-args', '--log-level', os.environ['ROS_LOG_LEVEL']]
        ),
    ])