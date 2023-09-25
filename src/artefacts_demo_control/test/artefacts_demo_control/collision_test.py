import os
import unittest
import subprocess
import re
from time import sleep

import ament_index_python
import pytest
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


@pytest.mark.launch_test
def generate_test_description():
    artefacts_demo_world = os.path.join(
        ament_index_python.get_package_prefix("artefacts_demo_control"),
        "world",
        "artefacts_demo_world.sdf",
    )
    breakpoint()

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("interbotix_xsarm_moveit"),
                        "launch",
                        "xsarm_moveit.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_model": "wx200",
            # Must change this line on new machine
            "world_filepath": artefacts_demo_world,
            "hardware_type": "gz_classic",
            "use_gazebo": "true",
        }.items(),
    )

    demo = Node(
        package="artefacts_demo_control",
        executable="artefacts_control",
    )

    return LaunchDescription(
        [
            sim,
            demo,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"sim": sim, "demo": demo}


class TestCollision(unittest.TestCase):
    def test_collsion(self):
        """
        Collision Test case, if the values are no longer equal it means that the block has shifted position through a collison
        """

        sleep(10)

        command = ["gz", "model", "-m", "artefacts_box", "-i"]
        result = subprocess.run(command, check=True, text=True, stdout=subprocess.PIPE)
        result_output = result.stdout

        # Extract position data if it exists
        position_pattern = r"pose\s*{\s*position\s*{\s*x:\s*(-?\d+\.\d+)\s*y:\s*(-?\d+\.\d+)\s*z:\s*(-?\d+\.\d+)"

        # Search for the x, y, and z variables using the regular expression
        match = re.search(position_pattern, result_output, re.DOTALL)

        # Extract the x, y, and z variables if a match is found
        if match:
            x_variable = float(match.group(1))
            y_variable = float(match.group(2))
            z_variable = float(match.group(3))
            print(f"x: {x_variable}")
            print(f"y: {y_variable}")
            print(f"z: {z_variable}")
        else:
            print("No x, y, or z variables found in the text.")
            x_variable = 0.0
            y_variable = 0.0
            z_variable = 0.0

        sleep(30)

        command = ["gz", "model", "-m", "artefacts_box", "-i"]
        result = subprocess.run(command, check=True, text=True, stdout=subprocess.PIPE)
        result_output = result.stdout

        # Extract new position data if it exists
        position_pattern = r"pose\s*{\s*position\s*{\s*x:\s*(-?\d+\.\d+)\s*y:\s*(-?\d+\.\d+)\s*z:\s*(-?\d+\.\d+)"

        # Search for the new x, y, and z variables using the regular expression
        match = re.search(position_pattern, result_output, re.DOTALL)

        # Extract the new x, y, and z variables if a match is found
        if match:
            x_new_variable = float(match.group(1))
            y_new_variable = float(match.group(2))
            z_new_variable = float(match.group(3))
            print(f"x new variable: {x_new_variable}")
            print(f"y new variable: {y_new_variable}")
            print(f"z new variable: {z_new_variable}")
        else:
            print("No x, y, or z variables found in the text.")
            x_new_variable = 0.0
            y_new_variable = 0.0
            z_new_variable = 0.0

        sleep(10)

        decimal = 3
        self.assertAlmostEqual(x_variable, x_new_variable, decimal)
        self.assertAlmostEqual(y_variable, y_new_variable, decimal)
        self.assertAlmostEqual(z_variable, z_new_variable, decimal)
