import geometry_msgs.msg
from tf.listener import TransformerROS
import tf
import rospy
import moveit_msgs.msg
from math import radians
from geometry_msgs.msg import Quaternion
import tf_conversions
class SceneObject():
    def __init__(self):
        self.stefan_dir = "/home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/"
        # self.stefan_dir = "package://STEFAN/stl/"
        self.assembly = "assembly"
        self.assembly_pose = geometry_msgs.msg.PoseStamped()
        self.assembly_pose.header.frame_id="base"

        ## rotate chair start position
        self.assembly_pose.pose.position.x = 1.15
        self.assembly_pose.pose.position.y = -0.1
        self.assembly_pose.pose.position.z = 0.601 #0.601
        self.assembly_pose.pose.orientation.w = 1.0

        # self.assembly_pose.pose.position.x = 1.15
        # self.assembly_pose.pose.position.y = -0.1
        # self.assembly_pose.pose.position.z = 0.85 #0.601
        # self.assembly_pose.pose.orientation.w = 1.0

        ## rotate chair goal position
        self.assembly_pose.pose.position.x = 1.15 #1.15
        self.assembly_pose.pose.position.y = 0.05
        self.assembly_pose.pose.position.z = 0.85
        self.assembly_pose.pose.orientation.x = 0.7071068
        self.assembly_pose.pose.orientation.y = 0.0
        self.assembly_pose.pose.orientation.z = 0.0
        self.assembly_pose.pose.orientation.w = 0.7071068
        
        # floor start position
        # self.assembly_pose.pose.position.x = 1.20 #1.15
        # self.assembly_pose.pose.position.y = -0.3
        # self.assembly_pose.pose.position.z = 0.751 #0.601
        # self.assembly_pose.pose.orientation.w = 1.0

        # floor goal position
        # self.assembly_pose.pose.position.x = 1.15 #1.15
        # self.assembly_pose.pose.position.y = -0.1
        # self.assembly_pose.pose.position.z = 1.0 #0.601
        # self.assembly_pose.pose.orientation.w = 1.0

        # renew test
        # self.assembly_pose.pose.position.x = 1.10 #1.15
        # self.assembly_pose.pose.position.y = -0.1
        # self.assembly_pose.pose.position.z = 1.0 #0.601
        # self.assembly_pose.pose.orientation.w = 1.0

        # self.assembly_pose.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(radians(90), radians(90), radians(0), 'rxyz') )

        # TEST MODE
        self.list = {self.assembly : self.assembly_pose}
     