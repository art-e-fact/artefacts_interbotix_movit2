import unittest
import sys
import os 
import subprocess
import json
import re
from time import sleep

import pytest
import launch_testing
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def get_gazebo_pose():
    command = ["gz", "model", "-m", "artefacts_box", "-i"]
    result = subprocess.run(command, check = True, text = True, stdout=subprocess.PIPE)
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
        return x_variable, y_variable, z_variable
    else:
        print("No x, y, or z variables found in the text.")
        return None, None, None

z_variable, y_variable, z_variable = get_gazebo_pose()
# Command for running node as a subprocess
command_node = ["ros2", "run", "artefacts_demo_control", "artefacts_control"]
running_node = subprocess.run(command_node, check = True, text = True, stdout=subprocess.PIPE)

sleep(5)
x_new_variable, y_new_variable, z_new_variable = get_gazebo_pose()
# Enable test node, it will error out but this must exist for process to occur
@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(
            package="artefacts_demo_control",
            executable='artefacts_control',
            output="screen"
        ),

# Attempted commands to launch node, all unsucesful
##############################
        # ExecuteProcess(
        #         cmd=[sys.executable, "src/artefacts_demo_control/artefacts_demo_control/push_object.py"],
        #         output="log"),

        # ExecuteProcess(
        #         cmd=[sys.executable, "push_object.py"],
        #         output="log"),

        # ExecuteProcess(
        #         cmd=["python3", "src/artefacts_demo_control/artefacts_demo_control/push_object.py"],
        #         output="log"),

#################################
        launch_testing.actions.ReadyToTest()
    ]), {}


@launch_testing.post_shutdown_test()
class TestCollision(unittest.TestCase):
    def test_collsion(self):
        '''
        Collision Test case, if the values are no longer equal it means that the block has shifted position through a collison
        '''
        print("Testing")
        decimal = 3
        self.assertAlmostEqual(x_variable, x_new_variable, decimal)
        self.assertAlmostEqual(y_variable, y_new_variable, decimal)
        self.assertAlmostEqual(z_variable, z_new_variable, decimal)
