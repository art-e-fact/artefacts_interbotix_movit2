"""
Example of push node
"""

from threading import Thread
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2 import MoveIt2Gripper
from pymoveit2.robots import interbotix as panda
from time import sleep


def main():
    rclpy.init()

    # Create node
    node = Node("artefacts_control")

    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 Interbotix wx200 Arm Interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name=panda.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name="/wx200/arm_controller/follow_joint_trajectory",
    )

    # Create MoveIt 2 Interbotix wx200 Gripper Interface
    moveit2_gripper = MoveIt2Gripper(
        node=node,
        gripper_joint_names=panda.gripper_joint_names(),
        open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=[0.02, -0.02],
        gripper_group_name=panda.MOVE_GROUP_GRIPPER,
        callback_group=callback_group,
        follow_joint_trajectory_action_name="/wx200/gripper_controller/follow_joint_trajectory",
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Joint Position of Interest
    joint_positions_init = [
        0.0,
        -1.35,
        1.5,
        0.8,
        0.0,
    ]
    joint_positions_ready = [
        0.2,
        0.558,
        0.855,
        -1.34,
        0.0,
    ]
    joint_positions_push = [
        0.2,
        0.820305,
        0.226893,
        -1.02974,
        0.0,
    ]

    sleep(5)

    node.get_logger().info("Node Is Ready")

    # Init

    moveit2.move_to_configuration(joint_positions_init)
    moveit2.wait_until_executed()

    # Close Gripper
    moveit2_gripper.close()
    moveit2_gripper.wait_until_executed()
    time.sleep(2)

    # Ready
    moveit2.move_to_configuration(joint_positions_ready)
    moveit2.wait_until_executed()

    # Push
    moveit2.move_to_configuration(joint_positions_push)
    moveit2.wait_until_executed()

    # Back 2 Init
    moveit2.move_to_configuration(joint_positions_ready)
    moveit2.wait_until_executed()
    moveit2.move_to_configuration(joint_positions_init)
    moveit2.wait_until_executed()

    node.get_logger().info("Trajectory Has Executed")

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
