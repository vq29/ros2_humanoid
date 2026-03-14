import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a YAML file from a ROS2 package."""
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    """Launch MoveIt2 + Gazebo Harmonic simulation together."""

    description_pkg = get_package_share_directory('humanoid_description')
    gazebo_pkg = get_package_share_directory('humanoid_gazebo')
    moveit_pkg = get_package_share_directory('humanoid_moveit_config')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        os.path.join(description_pkg, 'urdf', 'humanoid.urdf.xacro'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # SRDF
    srdf_path = os.path.join(moveit_pkg, 'config', 'humanoid.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Load config files
    kinematics_yaml = load_yaml('humanoid_moveit_config', 'config/kinematics.yaml')
    ompl_planning_yaml = load_yaml('humanoid_moveit_config', 'config/ompl_planning.yaml')
    joint_limits_yaml = load_yaml('humanoid_moveit_config', 'config/joint_limits.yaml')
    moveit_controllers_yaml = load_yaml('humanoid_moveit_config', 'config/moveit_controllers.yaml')

    # Planning pipeline
    planning_pipeline = {
        'planning_pipelines': ['ompl'],
        'ompl': ompl_planning_yaml,
    }

    # MoveIt2 parameters
    moveit_config = {
        **robot_description,
        **robot_description_semantic,
        **kinematics_yaml,
        **planning_pipeline,
        **joint_limits_yaml,
        **moveit_controllers_yaml,
        'use_sim_time': True,
    }

    # Include the Gazebo simulation launch file
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_pkg, 'launch', 'simulation.launch.py')
        ]),
    )

    # Move Group Node
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config],
    )

    # RViz with MoveIt plugin
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[moveit_config],
        output='screen',
    )

    return LaunchDescription([
        gazebo_simulation,
        move_group,
        rviz,
    ])
