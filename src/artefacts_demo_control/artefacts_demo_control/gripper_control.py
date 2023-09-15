# #!/usr/bin/env python3
# """
# Example of interacting with the gripper.
# `ros2 run pymoveit2 ex_gripper_command.py --ros-args -p action:="toggle"`
# `ros2 run pymoveit2 ex_gripper_command.py --ros-args -p action:="open"`
# `ros2 run pymoveit2 ex_gripper_command.py --ros-args -p action:="close"`
# """

# from threading import Thread

# import rclpy
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.node import Node

# from pymoveit2 import GripperCommand
# from pymoveit2.robots import interbotix as panda


# def main():
#     rclpy.init()

#     # Create node for this example
#     node = Node("ex_gripper_command")

#     # Declare parameter for gripper action
#     node.declare_parameter(
#         "action",
#         "toggle",
#     )

#     # Create callback group that allows execution of callbacks in parallel without restrictions
#     callback_group = ReentrantCallbackGroup()

#     # Create MoveIt 2 gripper interface
#     moveit2_gripper = GripperCommand(
#         node=node,
#         gripper_joint_names=panda.gripper_joint_names(),
#         open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
#         closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
#         max_effort=10.0,
#         ignore_new_calls_while_executing=True,
#         callback_group=callback_group,
#         gripper_command_action_name="/wx200/gripper_controller/follow_joint_trajectory",
#     )

#     # Spin the node in background thread(s)
#     executor = rclpy.executors.MultiThreadedExecutor(2)
#     executor.add_node(node)
#     executor_thread = Thread(target=executor.spin, daemon=True, args=())
#     executor_thread.start()

#     # Sleep a while in order to get the first joint state
#     node.create_rate(10.0).sleep()

#     # Get parameter
#     action = node.get_parameter("action").get_parameter_value().string_value

#     # Perform gripper action
#     node.get_logger().info(f'Performing gripper action "{action}"')
#     if "open" == action:
#         moveit2_gripper.open()
#         moveit2_gripper.wait_until_executed()
#     elif "close" == action:
#         moveit2_gripper.close()
#         moveit2_gripper.wait_until_executed()
#     else:
#         period_s = 1.0
#         rate = node.create_rate(1 / period_s)
#         while rclpy.ok():
#             moveit2_gripper()
#             moveit2_gripper.wait_until_executed()
#             rate.sleep()

#     rclpy.shutdown()
#     exit(0)


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
"""
Example of interacting with the gripper.
`ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"`
`ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="open"`
`ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="close"`
"""
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2Gripper
from pymoveit2.robots import interbotix as panda


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_gripper")

    # Declare parameter for gripper action
    # node.declare_parameter(
    #     "action",
    #     "toggle",
    # )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 gripper interface
    moveit2_gripper = MoveIt2Gripper(
        node=node,
        gripper_joint_names=panda.gripper_joint_names(),
        open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=panda.MOVE_GROUP_GRIPPER,
        callback_group=callback_group,
        # skip_planning = True,
        follow_joint_trajectory_action_name = "/wx200/gripper_controller/follow_joint_trajectory"
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Sleep a while in order to get the first joint state
    node.create_rate(10.0).sleep()
    node.declare_parameter("action", ["open"])

    # Get parameter
    action = node.get_parameter("action").get_parameter_value().string_value

    # Perform gripper action
    node.get_logger().info(f'Performing gripper action "{action}"')
    print("close")
    # moveit2_gripper.toggle()
    moveit2_gripper.open()
    moveit2_gripper.wait_until_executed()
    time.sleep(10)
    # if "open" == action:
    #     print("open")
    #     moveit2_gripper.open()
    #     moveit2_gripper.wait_until_executed()
    #     time.sleep(10)
    # elif "close" == action:
    #     print("close")
    #     moveit2_gripper.close()
    #     moveit2_gripper.wait_until_executed()
    # else:
    #     print("else")
    #     period_s = 1.0
    #     rate = node.create_rate(1 / period_s)
    #     while rclpy.ok():
    #         moveit2_gripper()
    #         moveit2_gripper.wait_until_executed()
    #         rate.sleep()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()