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
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

from part import SceneObject

import yaml
import tf2_ros

class MoveGroupPlanner():
    def __init__(self):
        ### MoveIt! 
        moveit_commander.roscpp_initialize(sys.argv)
        br = tf.TransformBroadcaster()
    

        #rospy.init_node('move_group_planner',
        #                anonymous=True)
 
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_left = moveit_commander.MoveGroupCommander("panda_left")
        self.group_right = moveit_commander.MoveGroupCommander("panda_right")
        self.hand_left = moveit_commander.MoveGroupCommander("hand_left")
        self.hand_right = moveit_commander.MoveGroupCommander("hand_right")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_left.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_left.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_right.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_right.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        rospy.sleep(1)
        stefan_dir = "/home/jiyeong/STEFAN/stl/"
        self.stefan = SceneObject()

        self.scene.remove_attached_object(self.group_left.get_end_effector_link())
        self.scene.remove_attached_object(self.group_right.get_end_effector_link())
        self.scene.remove_world_object()
        for key, value in self.stefan.list.items():
            self.scene.add_mesh(key, value, stefan_dir + key + ".stl")
            

        print ("============ Add Mesh:")
        ### Franka Collision
        self.set_collision_behavior = rospy.ServiceProxy(
            'franka_control/set_force_torque_collision_behavior',
            franka_control.srv.SetForceTorqueCollisionBehavior)
        #self.set_collision_behavior.wait_for_service()

        self.active_controllers = []

        self.listener = tf.TransformListener()
        self.tr = TransformerROS()


    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()
    def plan(self, goal, arm_name):
        if (arm_name == 'panda_left'):
            self.group_left.set_max_velocity_scaling_factor = 0.6
            self.group_left.set_max_acceleration_scaling_factor = 0.4
            self.group_left.set_start_state_to_current_state()
            trajectory = self.group_left.plan(goal)
        if (arm_name == 'panda_right'):
            self.group_right.set_max_velocity_scaling_factor = 0.6
            self.group_right.set_max_acceleration_scaling_factor = 0.4
            self.group_right.set_start_state_to_current_state()
            trajectory = self.group_right.plan(goal)
        return trajectory
    
    def plan_cartesian_target(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(math.radians(90), math.radians(90), math.radians(0)))
        pose_goal.position.x =  0.5429
        pose_goal.position.y = 0.05
        pose_goal.position.z = 0.6 + 0.50

        trajectory = self.group_left.plan(pose_goal)
        return trajectory

    def initial_pose(self):
        joint_goal = self.group_left.get_current_joint_values()
        print(joint_goal)
        joint_goal[6] = pi/4
        trajectory = self.group_left.plan(joint_goal)
        return trajectory

    def plan_joint_target(self):
        joint_goal = self.group_left.get_current_joint_values()
        print(joint_goal)
        joint_goal = [-0.386443059 ,0.954913403 ,   2.8973 ,-1.40610046 ,-0.336642852 ,0.527090123 ,2.8973]
        
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        self.group_right.plan(joint_goal)
        self.group_right.go()

    def plan_joint_left(self):
        joint_goal = [1.7098470003689021, 1.2583736025832721, -2.5043530156759464, -2.7632484238650843, 2.1677462567329893, 3.501025268485752, 2.3616581326737625]
        self.group_left.plan(joint_goal)
        self.group_left.go()

    def gripper_open(self, arm):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.04
        joint_goal[1] = 0.04
        if (arm == "left"):
            plan = self.hand_left.plan(joint_goal)
            if plan.joint_trajectory.points:
                self.hand_left.go()
                print("OPEN LEFT GRIPPER")
        
        else :
            self.hand_right.plan(joint_goal)
            print("OPEN RIGHT GRIPPER")
            self.hand_right.go()
        

    def gripper_close(self, arm):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.02
        joint_goal[1] = 0.02
        if (arm == "left"):
            plan = self.hand_left.plan(joint_goal)
            if plan.joint_trajectory.points:
                self.hand_left.go()
                print("CLOSE LEFT GRIPPER")
        else :
            self.hand_right.plan(joint_goal)
            self.hand_right.go()
            print("CLOSE RIGHT GRIPPER")
        
    def display_trajectory(self, plan):
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def check_grasp(self, part_name, arm):
        file_name = self.stefan.stefan_dir + "../grasping_point/" + part_name + ".stl.yaml"
        with open(file_name, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
        print(file_name)
        for pose in data_loaded['grasp_points']:
            if not rospy.is_shutdown():
                grasp_point = geometry_msgs.msg.PoseStamped()
                grasp_point.header.frame_id = part_name + "_frame"
                grasp_point.header.stamp = rospy.Time.now()
                
                # -Z axis offset
                offset = np.dot(quaternion_matrix(pose['orientation']) , np.array([0.0, 0.0, -0.109, 1]))
                grasp_point.pose.position.x = pose['position'][0] + offset[0]
                grasp_point.pose.position.y = pose['position'][1] + offset[1]
                grasp_point.pose.position.z = pose['position'][2] + offset[2]
                grasp_point.pose.orientation.x = pose['orientation'][0]
                grasp_point.pose.orientation.y = pose['orientation'][1]
                grasp_point.pose.orientation.z = pose['orientation'][2]
                grasp_point.pose.orientation.w = pose['orientation'][3]
                
                rospy.sleep(1)
                grasp_point = listener.transformPose(self.planning_frame, grasp_point) #transfrom msg to "base" frame
                   
                if (arm == "left"):
                    self.group_left.set_pose_target(grasp_point)
                    plan = self.group_left.plan()
                    if plan.joint_trajectory.points:
                        print(pose['position'][0], pose['position'][1], pose['position'][2])
                        self.group_left.go()
                        break
                    else:
                        rospy.logerr("FAILED")
                else :
                    self.group_right.set_pose_target(grasp_point)
                    plan = self.group_right.plan()
                    if plan.joint_trajectory.points:
                        print(pose['position'][0], pose['position'][1], pose['position'][2])
                        self.group_right.go()
                        break                
                    else:
                        rospy.logerr("FAILED")        

if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')
    
    mdp = MoveGroupPlanner()
    mdp.initial_pose()
    mdp.group_left.go()
    mdp.gripper_open("left")
    rospy.sleep(1)
    listener = tf.TransformListener()
    listener.waitForTransform("short_part_frame", mdp.planning_frame, rospy.Time(0), rospy.Duration(1.0))
    
    mdp.check_grasp("long_part", "left")
    touch_links = mdp.robot.get_link_names(group = 'hand_left')
    mdp.scene.attach_mesh(mdp.group_left.get_end_effector_link(), "long_part", touch_links=touch_links)
    # mdp.scene.attach_mesh('panda_left_link8', "long_part")
   
    rospy.sleep(1)
    mdp.gripper_close("left")

    mdp.plan_joint_target()

    mdp.plan_joint_left()

    # mdp.group_right.stop()
    # rospy.spin()

    
    

