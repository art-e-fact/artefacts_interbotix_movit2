"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2 import GripperCommand
from pymoveit2.robots import interbotix as panda
from time import sleep
import numpy as np
import math

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]



def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    # node.declare_parameter("position", [0.4, 0.0, 0.3])
    # gripper down
    node.declare_parameter("position", [0.25, 0.0, 0.18])
    # gripper front
    # node.declare_parameter("position", [0.35, 0.0, 0.3])

    # toward front, no rotation
    #node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    # toward front, 90 rotation
    #node.declare_parameter("quat_xyzw", [0.707, 0.0, 0.0, 0.707])
    # toward front down, 0 rotation
    #node.declare_parameter("quat_xyzw", [0.0, 0.707, 0.0, 0.707])
    # toward front down, 90 rotation
    node.declare_parameter("quat_xyzw", [-0.5, -0.5, 0.5, -0.5])

    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name=panda.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name = "/wx200/arm_controller/follow_joint_trajectory"
    )
    moveit2_gripper = GripperCommand(
        node=node,
        gripper_joint_names=panda.gripper_joint_names(),
        open_gripper_joint_positions=[0.037, -0.037],
        closed_gripper_joint_positions=[0.03, -0.03],
        max_effort=10.0,
        ignore_new_calls_while_executing=False,
        callback_group=callback_group,
        gripper_command_action_name="/wx200/gripper_controller/follow_joint_trajectory",
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    # Sleep a while in order to get the first joint state
    node.create_rate(10.0).sleep()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    orientations = [[0.0, 0.0, 0.0, 1.0], [0.707, 0.0, 0.0, 0.707], [0.0, 0.707, 0.0, 0.707],[-0.5, -0.5, 0.5, -0.5]]
    def get_end_effector_orientation(tray_tilt, karaage_orientation):
        return get_quaternion_from_euler(math.radians(karaage_orientation), math.radians(tray_tilt),0)

    #for quat_xyzw in orientations:
    x= 0.25
    y= 0.0
    moveit2_gripper.open()
    for tray_tilt in [45, 90]:
        for karaage_orientation in [0, 90]:
    
            for z in [0.1, -0.04]:
                quat_xyzw = get_end_effector_orientation(tray_tilt, karaage_orientation)
                # Move to pose
                position = [x,y,z]
                node.get_logger().info(
                    f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
                )
                moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian, target_link = "wx200/ee_gripper_link",
                        tolerance_orientation=0.005)
                moveit2.wait_until_executed()
                sleep(2.0)
            moveit2_gripper.close()
            #moveit2_gripper.wait_until_executed()
            sleep(2.0)

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
