"""
Example of pick and place node
"""

from threading import Thread
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2 import MoveIt2Gripper
from pymoveit2.robots import interbotix as panda

def main():
    rclpy.init()

    # Create node for this example
    node = Node("artefacts_control")

    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 Arm Interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name=panda.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name = "/wx200/arm_controller/follow_joint_trajectory"
    )

    moveit2_gripper = MoveIt2Gripper(
        node=node,
        gripper_joint_names=panda.gripper_joint_names(),
        open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=[0.02, -0.02],
        gripper_group_name=panda.MOVE_GROUP_GRIPPER,
        callback_group=callback_group,
        follow_joint_trajectory_action_name = "/wx200/gripper_controller/follow_joint_trajectory"
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("cartesian", True)


    position_init = [0.2, 0.2, 0.3]
    position_travel = [0.3, 0.1, 0.15]
    position_pick = [0.416925, 0.0, 0.02]
    position_place = [0.3, 0.1, 0.02]
    joint_positions_init = [
                            0.0,
                            -1.35,
                            1.5,
                            0.8,
                            0.0,
                        ]
    joint_positions_pick = [
                            0.0,
                            0.820305,
                            0.226893,
                            -1.02974,
                            0.0,
                        ]
    joint_positions_travel = [
                            -0.73446,
                            0.191986,
                            1.29154,
                            -1.48353,
                            0.0,
                        ]
    joint_positions_place = [
                            -1.43446,
                            0.820305,
                            0.226893,
                            -1.02974,
                            0.0,
                        ]
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Init

    moveit2.move_to_configuration(joint_positions_init)
    moveit2.wait_until_executed()
    # Open Gripper
    moveit2_gripper.open()
    moveit2_gripper.wait_until_executed()
    time.sleep(2)

    # Pick
    moveit2.move_to_configuration(joint_positions_pick)
    moveit2.wait_until_executed()

    # Close grippper

    moveit2_gripper.close()
    moveit2_gripper.wait_until_executed()
    time.sleep(2)

    # travel

    # moveit2.move_to_pose(position=position_travel, quat_xyzw=quat_xyzw, cartesian=cartesian, target_link = "wx200/ee_gripper_link")
    # moveit2.wait_until_executed()

    moveit2.move_to_configuration(joint_positions_travel)
    moveit2.wait_until_executed()
    time.sleep(1)
    # Place

    # moveit2.move_to_pose(position=position_place, quat_xyzw=quat_xyzw, cartesian=cartesian, target_link = "wx200/ee_gripper_link")
    # moveit2.wait_until_executed()

    moveit2.move_to_configuration(joint_positions_place)
    moveit2.wait_until_executed()

    # Open Gripper
    moveit2_gripper.open()
    moveit2_gripper.wait_until_executed()
    time.sleep(1)

    # Back 2 Init 
    moveit2.move_to_configuration(joint_positions_travel)
    moveit2.wait_until_executed()

    moveit2.move_to_configuration(joint_positions_init)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)

# def move_arm_to_pose(self):
#     moveit2.move_to_pose(position=position_init, quat_xyzw=quat_xyzw, cartesian=cartesian, target_link = "wx200/ee_gripper_link")
#     moveit2.wait_until_executed()
# def move_arm_joints(self):
#     pass
# def place(self):
#     pass
# def transfer(self):
#     pass
# def arm_init(self):
#     pass
if __name__ == "__main__":
    main()