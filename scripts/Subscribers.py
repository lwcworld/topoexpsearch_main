#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from tuw_multi_robot_msgs.msg import Graph, Vertex
from actionlib_msgs.msg import GoalStatusArray
import copy

class Subscribers():
    def __init__(self, q_t, q_m, q_r, q_h, q_f, q_p, q_NN, p):
        self.q_t = q_t
        self.q_m = q_m
        self.q_r = q_r
        self.q_h = q_h
        self.q_f = q_f
        self.q_p = q_p
        self.q_NN = q_NN
        self.p = p
        rospy.Subscriber('/hector_exploration_node/exploration_transform', PointCloud, self.cb_et, queue_size=5)
        rospy.Subscriber('/hector_exploration_node/obstacle_transform', PointCloud, self.cb_ot, queue_size=5)
        rospy.Subscriber('/exploration_path', Path, self.cb_p, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self.cb_o, queue_size=5)
        rospy.Subscriber('/map', OccupancyGrid, self.cb_m, queue_size=5)
        rospy.Subscriber('/segments', Graph, self.cb_v, queue_size=10)  # for voronoi graph
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.cb_s, queue_size=5)

        self.print_subscribe = False

    def cb_v(self, msg):
        if self.print_subscribe == True:
            print('[subscribed] : ' + 'voronoi')

        self.q_t['NN']      = msg.header.stamp.to_sec()
        self.q_NN['NN_msg'] = msg
        self.q_NN['origin_x'] = msg.origin.position.x
        self.q_NN['origin_y'] = msg.origin.position.y
        self.q_NN['vertices'] = msg.vertices

    def cb_m(self, msg):
        if self.print_subscribe == True:
            print('[subscribed] : ' + 'map')
        self.q_t['map'] = msg.header.stamp.to_sec()
        self.q_m['map_1d'] = msg.data
        self.q_m['res'] = msg.info.resolution  # [m]
        self.q_m['width'] = msg.info.width  # [pix]
        self.q_m['height'] = msg.info.height  # [pix]
        self.q_m['origin'] = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.q_m['map_2d'] = np.array(self.q_m['map_1d']).reshape(self.q_m['height'], self.q_m['width'])
        self.q_f['new_map'] = True

    def cb_ot(self, msg):
        # callback : /exploration_transform
        self.q_t['ot'] = msg.header.stamp.to_sec()
        self.q_f['OT'] = False

        X = [p.x for p in msg.points]
        Y = [p.y for p in msg.points]
        Z = [p.z for p in msg.points]

        self.q_h['g_ot_pcl'][0] = X
        self.q_h['g_ot_pcl'][1] = Y
        self.q_h['g_ot_pcl'][2] = Z

        self.q_f['OT'] = True
        print('cb : obstacle_transform')

    def cb_et(self, msg):
        # callback : /exploration_transform
        self.q_t['et'] = msg.header.stamp.to_sec()
        self.q_f['ET'] = False

        X = [p.x for p in msg.points]
        Y = [p.y for p in msg.points]
        Z = [p.z for p in msg.points]

        # # get frontier data
        i_zero_list = [i for i, v in enumerate(Z) if v == 10.0]
        X_f = [X[i] for i in i_zero_list]
        Y_f = [Y[i] for i in i_zero_list]

        self.q_h['g_et_pcl'][0] = X
        self.q_h['g_et_pcl'][1] = Y
        self.q_h['g_et_pcl'][2] = Z

        self.q_h['f_pcl_local'][0] = X_f
        self.q_h['f_pcl_local'][1] = Y_f
        self.q_f['ET'] = True
        print('cb : exploration_transform')

    def cb_p(self, msg):
        self.q_t['path_hector'] = msg.header.stamp.to_sec()
        X_p = [pose.pose.position.x for pose in msg.poses]
        Y_p = [pose.pose.position.y for pose in msg.poses]
        Z_p = [pose.pose.position.z for pose in msg.poses]

        self.q_h['path'][0] = X_p
        self.q_h['path'][1] = Y_p
        self.q_h['path'][2] = Z_p

        print('cb : exploration_path')

    def cb_o(self, msg):
        if self.print_subscribe == True:
            print('[subscribed] : ' + 'odometry')
        # callback : /odom
        self.q_t['odom'] = msg.header.stamp.to_sec()
        self.q_r['pos']  = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.q_f['odom'] = True

    def cb_s(self, msg):
        if self.print_subscribe == True:
            print('[subscribed] : ' + 'status')
        if len(msg.status_list) > 0:
            self.q_t['mission_status'] = msg.header.stamp.to_sec()
            prev_status = copy.deepcopy(self.q_p['status'])
            self.q_p['status'] = msg.status_list[0].status
            if prev_status ==1 and self.q_p['status'] != 1:
                self.q_t['robot_stop'] = msg.header.stamp.to_sec()
                # self.q_f['new_map'] = True