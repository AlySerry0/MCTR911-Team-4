from setuptools import setup
import os
from glob import glob

package_name = 'stacking_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aly',
    maintainer_email='aly@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_teleop = stacking_robot.gui_teleop:main',
            'gui_trajectory = stacking_robot.gui_trajectory:main',
            'kinematics_solver = stacking_robot.kinematics_solver:main',
            'velocity_kinematics = stacking_robot.velocity_kinematics:main',
            'explore_chain = stacking_robot.explore_chain:main',
            'test_kinematics = stacking_robot.test_kinematics:main',
            'test_velocity_kinematics = stacking_robot.test_velocity_kinematics:main',
            'validate_velocity_kinematics = stacking_robot.validate_velocity_kinematics:main',
            'validate_trajectories = stacking_robot.validate_trajectories:main',
            'position_controller = stacking_robot.position_controller:main',
            'pick_and_place = stacking_robot.pick_and_place:main',
            'debug_coordinates = stacking_robot.debug_coordinates:main',
            'test_orientation = stacking_robot.test_orientation:main',
            'simple_test = stacking_robot.simple_test:main',
            'final_pick_and_place = stacking_robot.final_pick_and_place:main', 
        ],
    },
)
