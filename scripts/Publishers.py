#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import tf
from Utils import *

class Publishers():
    def __init__(self):
        # ======== ROS communicators declaration ========z
        self.pub_cmd_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self.pub_NN = rospy.Publisher("/NN", String, queue_size=5)
        self.pub_dummy_goal_MBD_GSIM = rospy.Publisher("/goal_dummy_MBD_GSIM", PoseStamped, queue_size=5)
        self.pub_dummy_goal_FSE = rospy.Publisher("/goal_dummy_FSE", PoseStamped, queue_size=5)
        self.pub_dummy_goal_hector = rospy.Publisher("/goal_dummy_hector", PoseStamped, queue_size=5)

    def assign_cmd_goal(self, P, theta=0.0):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        # pose.header.seq = 4
        msg.pose.position.x = P[0]
        msg.pose.position.y = P[1]
        msg.pose.position.z = 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        return msg

    def assign_NN(self, NN, attr_list):
        G_json_str = get_jsonstr_graph(NN, attr_list)
        msg_NN_jsonstr = String()
        msg_NN_jsonstr.data = G_json_str

        return msg_NN_jsonstr