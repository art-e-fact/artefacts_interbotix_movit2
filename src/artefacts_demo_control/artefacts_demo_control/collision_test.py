import unittest
import sys
import os 
import subprocess
import json
import re

import pytest
import launch_testing
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

command = ["gz", "model", "-m", "artefacts_box", "-i"]
result = subprocess.run(command, check = True, text = True, stdout=subprocess.PIPE)
result_output = result.stdout
# print(result_output)

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



@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(
            package="artefacts_demo_control",
            executable='artefacts_control',
        ),
        launch_testing.actions.ReadyToTest()
    ])

@launch_testing.post_shutdown_test()
class TestCollision(unittest.TestCase):
    def test_collsion(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        # launch_testing.asserts.assertExitCodes(proc_info)
        print("hello")
