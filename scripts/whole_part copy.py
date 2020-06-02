import geometry_msgs.msg
from tf.listener import TransformerROS
import tf
import rospy
import moveit_msgs.msg
from math import radians
class SceneObject():
    def __init__(self):
        self.stefan_dir = "/home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/"
        # self.stefan_dir = "package://STEFAN/stl/"
        self.assembly = "assembly"
        self.assembly_pose = geometry_msgs.msg.PoseStamped()
        self.assembly_pose.header.frame_id="base"
        # self.assembly_pose.pose.orientation.w = 1.0

        # self.assembly_pose.pose.position.x = 0.95
        # self.assembly_pose.pose.position.y = 0.0 # -0.15
        # self.assembly_pose.pose.position.z = 0.601 #0.601
        # q = tf.transformations.quaternion_from_euler(radians(0), radians(0), radians(30))
        # self.assembly_pose.pose.orientation.x = q[0]
        # self.assembly_pose.pose.orientation.y = q[1]
        # self.assembly_pose.pose.orientation.z = q[2]
        # self.assembly_pose.pose.orientation.w = q[3]

        # self.assembly_pose.pose.position.x = 0.9
        # self.assembly_pose.pose.position.y = 0.10 # -0.15
        # self.assembly_pose.pose.position.z = 0.65#0.601
        # q = tf.transformations.quaternion_from_euler(radians(20), radians(0), radians(0))
        # self.assembly_pose.pose.orientation.x = q[0]
        # self.assembly_pose.pose.orientation.y = q[1]
        # self.assembly_pose.pose.orientation.z = q[2]
        # self.assembly_pose.pose.orientation.w = q[3]

        # self.assembly_pose.pose.position.x = 1.0
        # self.assembly_pose.pose.position.y = 0.08 # -0.15
        # self.assembly_pose.pose.position.z = 0.65#0.601
        # q = tf.transformations.quaternion_from_euler(radians(40), radians(0), radians(0))
        # self.assembly_pose.pose.orientation.x = q[0]
        # self.assembly_pose.pose.orientation.y = q[1]
        # self.assembly_pose.pose.orientation.z = q[2]
        # self.assembly_pose.pose.orientation.w = q[3]

        # self.assembly_pose.pose.position.x = 1.0
        # self.assembly_pose.pose.position.y = 0.08 # -0.15
        # self.assembly_pose.pose.position.z = 0.65#0.601
        # q = tf.transformations.quaternion_from_euler(radians(60), radians(3), radians(0))
        # self.assembly_pose.pose.orientation.x = q[0]
        # self.assembly_pose.pose.orientation.y = q[1]
        # self.assembly_pose.pose.orientation.z = q[2]
        # self.assembly_pose.pose.orientation.w = q[3]

        # self.assembly_pose.pose.position.x = 1.0
        # self.assembly_pose.pose.position.y = 0.15 # -0.15
        # self.assembly_pose.pose.position.z = 0.63#0.601
        # q = tf.transformations.quaternion_from_euler(radians(75), radians(3), radians(0))
        # self.assembly_pose.pose.orientation.x = q[0]
        # self.assembly_pose.pose.orientation.y = q[1]
        # self.assembly_pose.pose.orientation.z = q[2]
        # self.assembly_pose.pose.orientation.w = q[3]

        self.assembly_pose.pose.position.x = 1.0
        self.assembly_pose.pose.position.y = 0.18 # -0.15
        self.assembly_pose.pose.position.z = 0.63#0.601
        q = tf.transformations.quaternion_from_euler(radians(85), radians(3), radians(0))
        self.assembly_pose.pose.orientation.x = q[0]
        self.assembly_pose.pose.orientation.y = q[1]
        self.assembly_pose.pose.orientation.z = q[2]
        self.assembly_pose.pose.orientation.w = q[3]
       

        # self.assembly_pose.pose.position.x = 1.1
        # self.assembly_pose.pose.position.y = 0.15 # -0.15
        # self.assembly_pose.pose.position.z = 1.0#0.601
        # q = tf.transformations.quaternion_from_euler(radians(175), radians(0), radians(0))
        # self.assembly_pose.pose.orientation.x = q[0]
        # self.assembly_pose.pose.orientation.y = q[1]
        # self.assembly_pose.pose.orientation.z = q[2]
        # self.assembly_pose.pose.orientation.w = q[3]

        self.list = {self.assembly : self.assembly_pose}
     