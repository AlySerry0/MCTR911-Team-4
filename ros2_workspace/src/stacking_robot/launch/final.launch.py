from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
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
            executable='final_pick_and_place',
            name='final_pick_and_place',
            output='screen'
        ),
        
        # Auto-start after 3 seconds
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '-1', '/pick_and_place_command', 
                         'std_msgs/msg/String', 'data: sequence'],
                    output='screen'
                )
            ]
        )
    ])
