#!/usr/bin/env python
import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import networkx as nx
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def rviz_draw_floorplan(img, res):
    pub_floorplan = rospy.Publisher('~'+'test', Marker, queue_size=1)

    marker = Marker()

    marker.header.frame_id = "/map"
    marker.type = marker.CUBE_LIST
    marker.action = marker.ADD

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    dim = np.shape(img)
    x_dim = dim[0]
    y_dim = dim[1]

    bias_x = y_dim*res/2.
    bias_y = x_dim*res/2.

    for x in range(0,x_dim):
        for y in range(0,y_dim):
            p   = Point()
            c   = ColorRGBA()

            p.z = -0.3
            p.x = y*res - bias_x
            p.y = x*res - bias_y

            c.r = img[x,y,2]/255.
            c.g = img[x,y,1]/255.
            c.b = img[x,y,0]/255.
            c.a = 1.0

            marker.points.append(p)
            marker.colors.append(c)

    pub_floorplan.publish(marker)


def rviz_draw_topology(network, sp):
    nodeatt_pos = nx.get_node_attributes(network, 'pos')
    nodeidxs = network.nodes()
    poses = nodeatt_pos.values()

    # get node indexs of ('to_go', 'isrobot', etc) properties
    nodeatt_to_go = nx.get_node_attributes(network, 'to_go')
    nodeidxs_to_go = [i for i, x in enumerate(list(nodeatt_to_go.values())) if x == True]
    nodeatt_isrobot = nx.get_node_attributes(network, 'isrobot')
    nodeidxs_isrobot = [i for i, x in enumerate(list(nodeatt_isrobot.values())) if x == True]
    nodeidxs_etc = [i for i, x in enumerate(list(nodeidxs)) if i not in nodeidxs_to_go + nodeidxs_isrobot]
    poses_to_go = [x for i, x in enumerate(poses) if i in nodeidxs_to_go]
    poses_isrobot = [x for i, x in enumerate(poses) if i in nodeidxs_isrobot]
    poses_etc = [x for i, x in enumerate(poses) if i in nodeidxs_etc]

    # get node indexs of 'type' property
    N_class = max(sp['cmap_rviz'].keys())
    nodeatt_type = nx.get_node_attributes(network, 'type')
    poses_types = {}
    for i_c in range(N_class):
        poses_types[i_c] = [x for i, x in enumerate(poses) if nodeatt_type[i]==i_c]

    # extract start & end points to draw lines
    edges = list(network.edges())
    line_starts = []
    line_ends = []
    for i, e in enumerate(edges):
        line_starts.append(nodeatt_pos[edges[i][0]])
        line_ends.append(nodeatt_pos[edges[i][1]])

    # draw NN with ('to_go', 'isrobot', etc) property
    rviz_draw_points(poses_to_go, 2.5, (1., 1., 1.), 'to_go', size=0.7)
    rviz_draw_points(poses_isrobot, 2.5, (1., 0., 0.), 'isrobot', size=0.7)
    rviz_draw_points(poses_etc, 2.5, (0., 0., 0.), 'etc', size=0.7)
    rviz_draw_lines(line_starts, 0.5, line_ends, (0., 1., 0.), topic_name='topo_lines', width=0.3)

    # draw NN with 'type' property
    for i_c in range(N_class):
        cmap = (sp.cmap_rviz[i_c][2]/255.0, sp.cmap_rviz[i_c][1]/255.0, sp.cmap_rviz[i_c][0]/255.0)
        rviz_draw_points(poses_types[i_c], 1.5, cmap, 'type_'+'i_c', size=0.7)

def rviz_draw_points(points, z_pos, color, type, size=0.5):
    # points = [(x1,y1), (x2,y2), ...]
    # colors = (r,g,b)
    pub_point = rospy.Publisher('~topo_points'+type, Marker, queue_size=1)
    # rospy.loginfo('Publishing points')

    marker = Marker()
    marker.header.frame_id = "/map"
    if type == 'robot_pos':
        marker.type = marker.CUBE_LIST
    elif type == 'goal_pos' or type == 'goal_pos_compare_greedy' or type == 'goal_pos_compare_FE':
        marker.type = marker.CUBE_LIST
    else:
        marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD

    # set x and y scale for width/height
    marker.scale.x = size
    marker.scale.y = size

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker color
    marker.color.a = 0.7
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    marker.points = []
    for i_p in range(0, len(points)):
        # marker line points
        point = Point()
        point.x = points[i_p][0]
        point.y = points[i_p][1]
        point.z = z_pos
        marker.points.append(point)

    # publish points
    pub_point.publish(marker)

def rviz_draw_lines(starts, ends, color, topic_name, width=1):
    # starts = [(xs1,ys1), (xs2,ys2), (xs3,ys3), ...]
    # ends = [(xe1,ye1), (xe2,y2), (xe3,ye3), ...]
    pub_line = rospy.Publisher('~'+topic_name, Marker, queue_size=1)
    # rospy.loginfo('Publishing lines')

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD

    # set x and y scale for width/height
    marker.scale.x = width
    marker.scale.y = 0.0

    # marker color
    marker.color.a = 0.7
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # # marker position
    # marker.pose.position.x = 0.0
    # marker.pose.position.y = 0.0
    # marker.pose.position.z = 0.0

    marker.points = []
    for i_l in range(0, len(starts)):
        # marker line points
        first_line_point = Point()
        first_line_point.x = starts[i_l][0]
        first_line_point.y = starts[i_l][1]
        first_line_point.z = 0.0
        marker.points.append(first_line_point)

        second_line_point = Point()
        second_line_point.x = ends[i_l][0]
        second_line_point.y = ends[i_l][1]
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

    # Publish the Marker
    pub_line.publish(marker)


def trans_posatt_array2cart(network, res=0.5, origin=(-10,-10)):
    # transfrom pos attribute from array to cartesian coordinate
    network_c = copy.deepcopy(network)
    o_x = origin[0]
    o_y = origin[1]
    for i_n in network_c.nodes():
        x_p = network_c.nodes[i_n]['pos'][0]
        y_p = network_c.nodes[i_n]['pos'][1]
        x_c = x_p*res + o_x
        y_c = y_p*res + o_y
        network_c.nodes[i_n]['pos'] = (x_c, y_c)
    return network_c

def rviz_draw_robot_pos(robot_pos):
    rviz_draw_points(robot_pos, (1., 0., 0.), 'robot_pos', size=2.0)

def rviz_draw_goal_pos(goal_pos):
    rviz_draw_points(goal_pos, (0., 0., 1.), 'robot_pos', size=2.0)

def rviz_draw_boundary(ll, lr, ul, ur, topic_name):
    line_starts = [ll, lr, ur, ul]
    line_ends = [lr, ur, ul, ll]
    rviz_draw_lines(line_starts, line_ends, (1., 1., 0.), topic_name=topic_name)

def rviz_write_textinfo_query(query, position, color):
    pub_textinfo = rospy.Publisher('~textinfo_query', Marker, queue_size=1)
    # rospy.loginfo('Publishing text information')

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD

    marker.scale.x = 2.0
    marker.scale.y = 2.0
    marker.scale.z = 2.0

    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    marker.text = query
    pub_textinfo.publish(marker)

def rviz_write_textinfo_answer(answer, position, color):
    pub_textinfo = rospy.Publisher('~textinfo_answer', Marker, queue_size=1)

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD

    marker.scale.x = 2.0
    marker.scale.y = 2.0
    marker.scale.z = 2.0

    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    marker.text = answer
    pub_textinfo.publish(marker)

if __name__ == '__main__':
    rospy.init_node('rviz_topo_draw')
    glo_net = nx.read_gpickle("glo_network_origin_1.gpickle")
    vis_map = cv2.imread('vis_map_1.png', cv2.IMREAD_COLOR)
    lvlENN_map = cv2.imread('level_ENN_map_1.png',  cv2.IMREAD_COLOR)
    bridge = CvBridge()
    img_msg_vismap = bridge.cv2_to_imgmsg(vis_map)
    img_msg_lvlENN = bridge.cv2_to_imgmsg(lvlENN_map)
    while not rospy.is_shutdown() and 1:
        rviz_draw_topology(glo_net)

        # rviz_write_textinfo_query(  ' Travel distance : \n'
        #                           + '    Covered area : \n'
        #                           + '  Robot Position : \n'
        #                           + '   Goal Position :', position=(0.,-10.,0.))
        # rviz_write_textinfo_answer( '123 \n'
        #                           + '32 \n'
        #                           + '(10, 10) \n'
        #                           + '(20, 20)', position=(2.5,-10.,0.))
        # rviz_draw_boundary(ll=[-60,-10], lr=[54,-10], ur=[54,16], ul=[-60,16], topic_name='map_boundary')

        rviz_draw_points([(-50.0, 3.0)], [0.,1.,0.], 'robot_pos', size=1.0)

        rospy.sleep(1)