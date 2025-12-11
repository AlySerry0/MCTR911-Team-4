from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mujoco_ros2_dir = get_package_share_directory('mujoco_ros2')
    scene_path = os.path.join(mujoco_ros2_dir, 'mujoco_menagerie', 'universal_robots_ur5e', 'scene.xml')
    
    return LaunchDescription([
        Node(
            package='mujoco_ros2',
            executable='mujoco_node',
            name='mujoco_simulator',
            output='screen',
            arguments=[scene_path],
            parameters=[{
                'control_mode': 'POSITION',
                'simulation_frequency': 1000,
                'visualisation_frequency': 60
            }]
        ),
        
        Node(
            package='stacking_robot',
            executable='pick_and_place',
            name='pick_and_place_node',
            output='screen'
        )
    ])
