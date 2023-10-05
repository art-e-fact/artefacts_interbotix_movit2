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
import launch_testing.markers
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    artefacts_demo_world = PathJoinSubstitution(
        [
            FindPackageShare("artefacts_demo_control"),
            "world",
            "artefacts_demo_world.sdf",
        ]
    )

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
            "world_filepath": artefacts_demo_world,
            "hardware_type": "gz_classic",
            "use_gazebo": "true",
            "use_rviz": "false",
            "use_moveit_rviz": "false",
            "use_gazebo_gui": "true",
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


def get_model_location(model_name):
    """
    Function to get the location of a box
    """

    command = ["gz", "model", "-m", model_name, "-i"]
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
        print(f"x box coordinate: {x_variable}")
        print(f"y box coordinate: {y_variable}")
        print(f"z box coordinate: {z_variable}")
    else:
        print("No x, y, or z variables found in the text.")
        x_variable = 0.0
        y_variable = 0.0
        z_variable = 0.0
    return x_variable, y_variable, z_variable


class TestCollision(unittest.TestCase):
    def test_box_moved(self, proc_output, demo):
        """
        Test case to see if node has finished executing and to gather 2 model locations
        """
        model_name = "artefacts_box"
        sleep(10)

        (
            TestCollision.x_variable,
            TestCollision.y_variable,
            TestCollision.z_variable,
        ) = get_model_location(model_name)

        proc_output.assertWaitFor("Block has been moved", timeout=180)

        (
            TestCollision.x_new_variable,
            TestCollision.y_new_variable,
            TestCollision.z_new_variable,
        ) = get_model_location(model_name)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_collsion(self):
        """
        Collision Test case, if the values are no longer equal it means that the block has shifted position through a collison
        """
        decimal = 3

        self.assertAlmostEqual(
            TestCollision.x_variable, TestCollision.x_new_variable, decimal
        )
        self.assertAlmostEqual(
            TestCollision.y_variable, TestCollision.y_new_variable, decimal
        )
        self.assertAlmostEqual(
            TestCollision.z_variable, TestCollision.z_new_variable, decimal
        )
