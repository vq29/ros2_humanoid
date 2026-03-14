import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch the object detection node."""

    perception_pkg = get_package_share_directory('humanoid_perception')
    config = os.path.join(perception_pkg, 'config', 'detection_params.yaml')

    object_detector = Node(
        package='humanoid_perception',
        executable='object_detector',
        name='object_detector',
        parameters=[config, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        object_detector,
    ])
