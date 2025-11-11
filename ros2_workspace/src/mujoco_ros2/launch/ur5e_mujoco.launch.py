from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Adjust this path if your workspace is somewhere else
    xml_scene = "/home/omar/MCTR911-Team-4/ros2_workspace/src/mujoco_ros2/mujoco_menagerie/universal_robots_ur5e/scene.xml"


    return LaunchDescription([
        Node(
            package="mujoco_ros2",
            executable="mujoco_node",
            name="mujoco_ur5e",
            output="screen",
            arguments=[xml_scene],
            parameters=[
                {"joint_state_topic_name": "ur5e/joint_states"},
                {"joint_command_topic_name": "/joint_commands"},
                {"control_mode": "POSITION"},
                {"simulation_frequency": 500},
                {"visualisation_frequency": 60},
            ],
        ),
    ])
