#!/usr/bin/env python3
import rclpy
import threading
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


class Nav2Bridge(Node):
    def __init__(self, nav):
        super().__init__('nav2_bridge')
        self.nav = nav
        self.is_navigating = False
        self.init_sub = self.create_subscription(
            PoseStamped, '/nav2_initial_pose', self.init_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/nav2_goal', self.goal_callback, 10)
        self.cancel_sub = self.create_subscription(
            PoseStamped, '/nav2_cancel', self.cancel_callback, 10)
        self.get_logger().info("Nav2 Bridge started. Waiting for goals...")

    def cancel_callback(self, msg):
        self.get_logger().info("Canceling navigation")
        self.is_navigating = False
        self.nav.cancelTask()

    def init_callback(self, msg):
        self.get_logger().info(
            f"Setting initial pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.nav.setInitialPose(msg)

    def goal_callback(self, msg):
        self.get_logger().info(
            f"Received goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        threading.Thread(target=self._execute_goal, args=(msg,)).start()

    def _execute_goal(self, msg):
        self.is_navigating = True
        self.nav.goToPose(msg)
        while not self.nav.isTaskComplete() and self.is_navigating:
            feedback = self.nav.getFeedback()
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal reached!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal canceled!")


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    print("Nav2 is active! Waiting for goals...")
    bridge = Nav2Bridge(nav)
    rclpy.spin(bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()