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


class CollectData(object):

    def __init__(self):
        self.c = 0
        self.command = outputMsg.SModel_robot_output()
        self.pub = rospy.Publisher('SModelRobotOutput',
                                   outputMsg.SModel_robot_output)
        self.rarm = moveit_commander.MoveGroupCommander("manipulator")
        self.robot = moveit_commander.RobotCommander()
        self.xlimit = [-0.60, -0.30]
        self.ylimit = [0.360, 0.60]
        self.z = 0.40
        self.z_min = 0.15
        self.sleeptime = 1.0
        self.gripper_close_ratio = 75

        gripper_pose_virtical = tf.transformations.quaternion_from_euler(
            -pi / 2, pi / 2, pi / 2)
        self.home_pose = geometry_msgs.msg.Pose()
        self.home_pose.position.x = -0.304788039946
        self.home_pose.position.y = 0.401533995393
        self.home_pose.position.z = 0.497607185593
        self.home_pose.orientation.x = gripper_pose_virtical[0]
        self.home_pose.orientation.y = gripper_pose_virtical[1]
        self.home_pose.orientation.z = gripper_pose_virtical[2]
        self.home_pose.orientation.w = gripper_pose_virtical[3]

    def image_callback(self, msg):

        print "#" * 20, self.c, "#" * 20
        self.c += 1

        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

        print "-" * 10, "current pose"
        print self.rarm.get_current_pose()
        print "-" * 10, "rpy"
        print self.rarm.get_current_rpy()
        print ""

        print "move to ...."
        target_pose = self.home_pose
        target_pose.position.x = random.uniform(self.xlimit[0], self.xlimit[1])
        target_pose.position.y = random.uniform(self.ylimit[0], self.ylimit[1])
        target_pose.position.z = self.z
        print target_pose
        self.rarm.set_pose_target(target_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

        print "arm down"
        target_pose.position.z = self.z_min
        self.rarm.set_pose_target(target_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

        print "release"
        self.robotiq_open()
        rospy.sleep(self.sleeptime)

        print "arm up"
        target_pose.position.z = self.z
        self.rarm.set_pose_target(target_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

        print "escape"
        self.rarm.set_pose_target(self.home_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

        print "save data"
        with open('test.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow(
                ["../data/image_{0:04d}.jpeg".format(self.c), str(target_pose.position.x), str(target_pose.position.y)])

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            print("Received an image!")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            cv2.imwrite('image_{0:04d}.jpeg'.format(self.c), cv2_img)
            print "saved"

        # print "ready"
        #
        rospy.sleep(self.sleeptime)
        rospy.sleep(self.sleeptime)
        rospy.sleep(self.sleeptime)

        print "reach"
        target_pose.position.z = self.z
        self.rarm.set_pose_target(target_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

        print "arm down"
        target_pose.position.z = self.z_min
        self.rarm.set_pose_target(target_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

        print "hold"
        self.robotiq_close()
        rospy.sleep(self.sleeptime)

        print "arm up"
        target_pose.position.z = self.z
        self.rarm.set_pose_target(target_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

        print "ready"
        self.rarm.set_pose_target(self.home_pose)
        self.rarm.go()
        rospy.sleep(self.sleeptime)

    def robotiq_open(self):
        self.command.rPRA = 0
        self.pub.publish(self.command)
        print "robotiq gripper close"
        # rospy.sleep(2.0)

    def robotiq_close(self):
        self.command.rPRA = self.gripper_close_ratio
        self.pub.publish(self.command)
        print "robotiq gripper close"
        # rospy.sleep(2.0)

    def robotiq_activate(self):
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSPA = 255
        self.command.rFRA = 150
        self.pub.publish(self.command)
        rospy.sleep(1)

    def main(self):
        rospy.init_node("moveit_command_sender")
        # robot = moveit_commander.RobotCommander()
        print "=" * 10, " Robot Groups:"
        print self.robot.get_group_names()
        print "=" * 10, " Printing robot state"
        print self.robot.get_current_state()
        print "=" * 10

        image_topic = "/camera/rgb/image_rect_color"

        print "robotiq gripper activating..."
        self.robotiq_activate()
        self.robotiq_open()

        # rarm = moveit_commander.MoveGroupCommander("manipulator")
        self.rarm.set_max_velocity_scaling_factor(0.1)
        print "=" * 15, " Right arm ", "=" * 15
        print "=" * 10, " Reference frame: %s" % self.rarm.get_planning_frame()
        print "=" * 10, " Reference frame: %s" % self.rarm.get_end_effector_link()

        rarm_initial_pose = self.rarm.get_current_pose().pose
        print "=" * 10, " Printing initial pose: "
        print rarm_initial_pose

        # target_pose_r = geometry_msgs.msg.Pose()
        # target_pose_r.position.x = 0.2035
        # target_pose_r.position.y = -0.5399
        # target_pose_r.position.z = 0.0709
        # target_pose_r.orientation.x = 0.000427
        # target_pose_r.orientation.y = 0.000317
        # target_pose_r.orientation.z = -0.000384
        # target_pose_r.orientation.w = 0.999999
        # self.rarm.set_pose_target(target_pose_r)
        home_joints = [-0.704154494998237, -1.544637275925154, -1.499685227873204,
                       -1.401863924370521, 1.5760672646165323, -0.6647137586379168]

        self.rarm.set_pose_target(self.home_pose)

        # print "=" * 10, " plan1..."
        print "=" * 10, " moving to home position..."
        print self.home_pose
        # self.rarm.go()
        rospy.sleep(self.sleeptime)

        #print "holding the object..."
        # rospy.sleep(self.sleeptime)
        # self.robotiq_close()

        # rarm.set_pose_target(rarm_initial_pose)
        # rarm.go()
        # rospy.sleep(2)

        # while not rospy.is_shutdown():
        # planning_frame = rarm.get_planning_frame()
        # print "============ Reference frame: %s" % planning_frame
        #
        # group_names = robot.get_group_names()
        # print "============ Robot Groups:", robot.get_group_names()
        # eef_link = rarm.get_end_effector_link()
        # print "============ End effector: %s" % eef_link
        #
        # print "============ Printing robot state"
        # print robot.get_current_state()
        # print ""
        #
        # print "robotiq open"
        # self.robotiq_open()
        #
        # print "robotiq close"
        # self.robotiq_close()
        #
        # # target_pose_r.position.x = random.uniform(0.0, 0.30)
        # # target_pose_r.position.y = random.uniform(0.0, 0.30)
        # # target_pose_r.position.z = random.uniform(0.0, 0.30)
        # # print target_pose_r
        # # rarm.set_pose_target(target_pose_r)
        # # c += 1
        # # print "=" * 10, " plan", c, " ..."
        # #
        # # rarm.go()
        #
        # print "#" * 10
        # print rarm.get_current_pose()
        # print
        # print rarm.get_current_rpy()

        rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.spin()


if __name__ == '__main__':
    collectdata = CollectData()
    try:
        collectdata.main()
    except rospy.ROSInterruptException:
        pass
