#! /usr/bin/env python3 
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi

rclpy.init()
nav = BasicNavigator()
q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, pi/4)
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = nav.get_clock().now().to_msg()
goal_pose.pose.position.x = 1.0
goal_pose.pose.position.y = 0.0
goal_pose.pose.position.z = 0.0
goal_pose.pose.orientation.x = q_x
goal_pose.pose.orientation.y = q_y
goal_pose.pose.orientation.z = q_z
goal_pose.pose.orientation.w = q_w

nav.goToPose(goal_pose)
while not nav.isTaskComplete():
    print(nav.getFeedback())

rclpy.shutdown()