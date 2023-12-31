#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
`ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import interbotix as panda


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            -0.191986,
            -0.10472,
            0.890118,
            -1.3439,
            -2.75762,
        ],
    )

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

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameter
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    node.declare_parameter(
        "joint_positions_2",
        [
            -0.0698132,
            0.191986,
            1.29154,
            -1.48353,
            0.0,
        ],
    )
    joint_positions = (
        node.get_parameter("joint_positions_2").get_parameter_value().double_array_value
    )
    

    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    node.declare_parameter(
        "joint_positions_3",
        [
            -0.0698132,
            0.191986,
            0.750492,
            -1.02974,
            0.0,
        ],
    )
    joint_positions = (
        node.get_parameter("joint_positions_3").get_parameter_value().double_array_value
    )

    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)





if __name__ == "__main__":
    main()