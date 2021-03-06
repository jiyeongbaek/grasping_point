#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
import franka_control.srv
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_matrix

import numpy as np
from MoveGroupPlanner import *
from whole_part import SceneObject
import yaml
import tf2_ros
import rospkg


class ContinuousGraspCandid():
    def __init__(self, package='grasping_point', path='STEFAN/grasping_point/assembly_line_renew.yaml'):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path(package)
        self.file_path = self.package_path + '/' + path
        with open(self.file_path, 'r') as stream:
            self.yaml = yaml.safe_load(stream)

    def get_grasp(self, index, ratio):
        lb = np.array(self.yaml['grasp_points'][index]['lower_bound'])
        ub = np.array(self.yaml['grasp_points'][index]['upper_bound'])
        ori = np.array(self.yaml['grasp_points'][index]['orientation'])
        offset = np.dot(quaternion_matrix(
            ori)[:3, :3], np.array([0.0, 0.0, -0.103]))
        return ((ub-lb) * ratio + lb + offset, ori)

    def get_grasp_pose_msg(self, index, ratio):
        g = self.get_grasp(index, ratio)
        # pose_msg = geometry_msgs.msg.Pose()
        pose_msg = geometry_msgs.msg.PoseStamped()
        pose_msg.header.frame_id = "assembly_frame"
        pose_msg.header.stamp = rospy.Time(0)
        pose_msg.pose.position.x = g[0][0]
        pose_msg.pose.position.y = g[0][1]
        pose_msg.pose.position.z = g[0][2]
        pose_msg.pose.orientation.x = g[1][0]
        pose_msg.pose.orientation.y = g[1][1]
        pose_msg.pose.orientation.z = g[1][2]
        pose_msg.pose.orientation.w = g[1][3]
        return pose_msg


if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')

    mdp = MoveGroupPlanner()
    mdp.initial_pose()
    mdp.gripper_open()

    listener = tf.TransformListener()
    listener.waitForTransform(
        "assembly_frame", mdp.planning_frame, rospy.Time(0), rospy.Duration(1.0))

    grp = ContinuousGraspCandid('grasping_point')
    for i in range(0, 10):
        print(i)
        grasp_point = listener.transformPose(mdp.planning_frame, grp.get_grasp_pose_msg(0, 0.1*i))  # transfrom msg to "base" frame
        try:
            mdp.group_1st.plan(grasp_point)
            mdp.group_1st.go()
            break
        except : 
            print("panda 1st fail ")
        try:
            mdp.group_2nd.plan(grasp_point)
            mdp.group_2nd.go()
            break
        except : 
            print("panda 2nd fail")
        try:
            mdp.group_3rd.plan(grasp_point)
            mdp.group_3rd.go()
            # break
        except : 
            print("panda 3rd fail")

    for key, value in mdp.stefan.list.items():
        mdp.scene.add_mesh(
                key, value, mdp.stefan.stefan_dir + key + ".stl")
