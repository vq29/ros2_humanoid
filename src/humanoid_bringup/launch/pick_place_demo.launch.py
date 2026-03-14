import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch the complete pick-and-place demo.
    
    This launches:
    1. Gazebo Harmonic simulation (world + robot + controllers)
    2. MoveIt2 (move_group + RViz)
    3. Object detection (perception)
    4. Pick-and-place state machine (bringup)
    """

    gazebo_pkg = get_package_share_directory('humanoid_gazebo')
    moveit_pkg = get_package_share_directory('humanoid_moveit_config')
    perception_pkg = get_package_share_directory('humanoid_perception')
    bringup_pkg = get_package_share_directory('humanoid_bringup')

    # 1. Launch MoveIt + Gazebo (includes simulation.launch.py internally)
    moveit_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg, 'launch', 'moveit_gazebo.launch.py')
        ]),
    )

    # 2. Launch perception
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(perception_pkg, 'launch', 'perception.launch.py')
        ]),
    )

    # 3. Pick-and-place node
    pick_place_config = os.path.join(
        bringup_pkg, 'config', 'pick_place_params.yaml'
    )
    pick_place = Node(
        package='humanoid_bringup',
        executable='pick_place_node',
        name='pick_place_node',
        parameters=[pick_place_config, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        moveit_gazebo,
        perception,
        pick_place,
    ])
