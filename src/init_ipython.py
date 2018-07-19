import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import random
import csv
import tf

import roslib
roslib.load_manifest('robotiq_s_model_control')
from robotiq_s_model_control.msg import _SModel_robot_output as outputMsg
from time import sleep

# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Instantiate CvBridge
bridge = CvBridge()

command = outputMsg.SModel_robot_output()
rarm = moveit_commander.MoveGroupCommander("manipulator")
robot = moveit_commander.RobotCommander()

z = 0.50
z_min = 0.12
sleeptime = 2.0

gripper_pose_virtical = tf.transformations.quaternion_from_euler(
    -pi / 2, pi / 2, pi / 2)
home_pose = geometry_msgs.msg.Pose()
home_pose.position.x = -0.304788039946
home_pose.position.y = 0.401533995393
home_pose.position.z = 0.497607185593
home_pose.orientation.x = gripper_pose_virtical[0]
home_pose.orientation.y = gripper_pose_virtical[1]
home_pose.orientation.z = gripper_pose_virtical[2]
home_pose.orientation.w = gripper_pose_virtical[3]
home_joints = [
    -0.7015657937617084,
    -1.5992801067858604,
    -1.3811384064202805,
    -1.731670452629782,
    1.5704884641121384,
    -0.7000814234168269]

rospy.init_node("moveit_command_sender")
rarm.set_max_velocity_scaling_factor(0.1)


def armdown():
    target_pose = rarm.get_current_pose().pose
    target_pose.position.z = z_min
    target_pose.orientation = home_pose.orientation
    rarm.set_pose_target(target_pose)
    rarm.go()


def armup():
    target_pose = rarm.get_current_pose().pose
    target_pose.position.z = z
    target_pose.orientation = home_pose.orientation
    rarm.set_pose_target(target_pose)
    rarm.go()


def adjust_z(adjz=0.2):
    target_pose = rarm.get_current_pose().pose
    target_pose.position.z = adjz
    target_pose.orientation = home_pose.orientation
    rarm.set_pose_target(target_pose)
    rarm.go()


def gohome():
    rarm.set_joint_value_target(home_joints)
    rarm.go()


print "gohome()"
print "armup()"
print "armdown()"
print "adjust_z(0.0to1.0)"
print "rarm.get_current_pose()"
