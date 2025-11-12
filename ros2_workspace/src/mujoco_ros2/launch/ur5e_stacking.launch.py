import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_share_directory = get_package_share_directory('mujoco_ros2')
    xml_scene_path = os.path.join(package_share_directory, 'mujoco_menagerie/universal_robots_ur5e/scene.xml')

    if not os.path.exists(xml_scene_path):
        raise FileNotFoundError(f"Scene file does not exist: {xml_scene_path}.")

    mujoco_node = Node(
        package="mujoco_ros2",
        executable="mujoco_node",
        
        name="mujoco_ur5e", 
        output="screen",
        arguments=[xml_scene_path],
        parameters=[
            {"joint_state_topic_name": "ur5e/joint_states"},
            {"joint_command_topic_name": "/joint_commands"},
            {"control_mode": "POSITION"},
            {"simulation_frequency": 500},
            {"visualisation_frequency": 60},

            {"camera_focal_point": [0.0, 0.0, 0.25]},
            {"camera_distance": 2.5},
            {"camera_azimuth": 135.0},
            {"camera_elevation": -20.0},
            {"camera_orthographic": True}
        ]
    )

    return LaunchDescription([mujoco_node])
