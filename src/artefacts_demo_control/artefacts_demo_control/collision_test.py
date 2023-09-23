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

ExecuteProcess(
        cmd=[sys.executable, "src/artefacts_demo_control/artefacts_demo_control/interbotix_moveit_control_node.py"],
        output="log")


sleep(10)

@launch_testing.post_shutdown_test()
class TestCollision(unittest.TestCase):
    def test_collsion(self):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        # launch_testing.asserts.assertExitCodes(proc_info)
        print("hello")
