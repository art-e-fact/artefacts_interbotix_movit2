from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'artefacts_demo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'world'), glob('world/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='decarabas',
    maintainer_email='decarabas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'artefacts_control = artefacts_demo_control.push_object:main',
            'artefacts_control_old = artefacts_demo_control.interbotix_moveit_control_node:main',
            'ex_pose_goal = artefacts_demo_control.pose_goal:main',
            "ex_joint_goal = artefacts_demo_control.joint_goal:main",
            "ex_gripper_command = artefacts_demo_control.gripper_control:main"
        ],
    },
)
