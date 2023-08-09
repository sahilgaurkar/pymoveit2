#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import baxter_left
from pymoveit2.robots import baxter_right
from opencv_interfaces.srv import TargetPose, GripperCmd


class MoveitService(Node):
    def __init__(self):
        super().__init__("moveit_node")

        self.srv = self.create_service(TargetPose, "moveit_service", self.moveit_callback)
        self.srv = self.create_service(GripperCmd, "gripper_service", self.gripper_callback)

    def moveit_callback(self, request, response):

        position = [request.position.x, request.position.y, request.position.z]
        quat_xyzw = [request.orientation.x, request.orientation.y, request.orientation.z, request.orientation.w]
        cartesian = request.cartesian
        arm = request.arm
        object = request.object

        self.get_logger().info("Incoming Moveit request")

        callback_group = ReentrantCallbackGroup()


        if arm == 'left':
            moveit2 = MoveIt2(
                node=self,
                joint_names=baxter_left.joint_names(),
                base_link_name=baxter_left.base_link_name(),
                end_effector_name=baxter_left.end_effector_name(),
                group_name=baxter_left.MOVE_GROUP_ARM,
                callback_group=callback_group,
                execute_via_moveit=False,
            )
        elif arm == 'right':
            moveit2 = MoveIt2(
                node=self,
                joint_names=baxter_right.joint_names(),
                base_link_name=baxter_right.base_link_name(),
                end_effector_name=baxter_right.end_effector_name(),
                group_name=baxter_right.MOVE_GROUP_ARM,
                callback_group=callback_group,
                execute_via_moveit=False,
            )
        else:
            response.success = False
            response.message = "Invalid Arm"
            return response
            

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Move to pose
        self.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()

        success = True
        message = "Moveit Service Status"

        response.success = success
        response.message = message

        return response


    def gripper_callback(self, request, response):

        #arm = 'left'/'right'
        arm = request.arm
        #action = 'close'/'open'
        action = request.action

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        if arm == 'left':
            # Create MoveIt 2 gripper interface
            moveit2_gripper = MoveIt2Gripper(
                node=self,
                gripper_joint_names=baxter_left.gripper_joint_names(),
                open_gripper_joint_positions=baxter_left.OPEN_GRIPPER_JOINT_POSITIONS,
                closed_gripper_joint_positions=baxter_left.CLOSED_GRIPPER_JOINT_POSITIONS,
                gripper_group_name=baxter_left.MOVE_GROUP_GRIPPER,
                callback_group=callback_group,
            )
        elif arm == 'right':
            moveit2_gripper = MoveIt2Gripper(
                node=self,
                gripper_joint_names=baxter_right.gripper_joint_names(),
                open_gripper_joint_positions=baxter_right.OPEN_GRIPPER_JOINT_POSITIONS,
                closed_gripper_joint_positions=baxter_right.CLOSED_GRIPPER_JOINT_POSITIONS,
                gripper_group_name=baxter_right.MOVE_GROUP_GRIPPER,
                callback_group=callback_group,
            )
        else:
            response.success = False
            response.message = "Invalid Arm"
            return response

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Sleep a while in order to get the first joint state
        self.create_rate(10.0).sleep()

        # Perform gripper action
        self.get_logger().info(f'Performing gripper action "{action}"')
        if "open" == action:
            moveit2_gripper.open()
            moveit2_gripper.wait_until_executed()
        elif "close" == action:
            moveit2_gripper.close()
            moveit2_gripper.wait_until_executed()
        else:
            period_s = 1.0
            rate = self.create_rate(1 / period_s)
            while rclpy.ok():
                moveit2_gripper()
                moveit2_gripper.wait_until_executed()
                rate.sleep()


        success = True
        message = "Gripper Service Status"

        response.success = success
        response.message = message

        return response

def main():
    rclpy.init()
    moveit_service = MoveitService()
    rclpy.spin(moveit_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
