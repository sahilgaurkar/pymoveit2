#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import baxter_left
from pymoveit2.robots import baxter_right
from opencv_interfaces.srv import TargetPose


class MoveitService(Node):
    def __init__(self):
        super().__init__("moveit_node")

        self.srv = self.create_service(TargetPose, "moveit_callback", self.moveit_callback)

    def moveit_callback(self, request, response):

        position = [request.position.x, request.position.y, request.position.z]
        quat_xyzw = [request.orientation.x, request.orientation.y, request.orientation.z, request.orientation.w]
        cartesian = request.cartesian
        arm = request.arm
        object = request.object

        self.get_logger().info("Incoming request")

        callback_group = ReentrantCallbackGroup()


        if arm == 'left':
            moveit2 = MoveIt2(
                node=self,
                joint_names=baxter_left.joint_names(),
                base_link_name=baxter_left.base_link_name(),
                end_effector_name=baxter_left.end_effector_name(),
                group_name=baxter_left.MOVE_GROUP_ARM,
                callback_group=callback_group,
                execute_via_moveit=True,
            )
        elif arm == 'right':
            moveit2 = MoveIt2(
                node=self,
                joint_names=baxter_right.joint_names(),
                base_link_name=baxter_right.base_link_name(),
                end_effector_name=baxter_right.end_effector_name(),
                group_name=baxter_right.MOVE_GROUP_ARM,
                callback_group=callback_group,
                execute_via_moveit=True,
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


def main():
    rclpy.init()
    moveit_service = MoveitService()
    rclpy.spin(moveit_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
