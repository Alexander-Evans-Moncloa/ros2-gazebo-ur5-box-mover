import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ur_simulation_gazebo_path = get_package_share_directory('ur_simulation_gazebo')

    ur_sim_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_simulation_gazebo_path, 'launch', 'ur_sim_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': 'true'
        }.items(),
    )

    pick_and_place_node = Node(
        package='ur_simulation_gazebo',
        executable='pick_and_place.py',
        name='pick_and_place',
        output='screen',
    )

    return LaunchDescription([
        ur_sim_moveit_launch,
        pick_and_place_node,
    ])