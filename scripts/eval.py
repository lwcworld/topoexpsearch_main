#!/usr/bin/env python
import numpy as np
import networkx as nx
from Publishers import *
from Utils import *
from Params import *
import rospy
from topoexpsearch_MBM.srv import MBD_GSIM
from topoexpsearch_MBM.srv import FSE
from topoexpsearch_FVE.srv import pred_FVE
from topoexpsearch_planner.srv import get_path_GEST
from topoexpsearch_planner.srv import get_path_hector
import ast
from geometry_msgs.msg import Point32


p = param()

# ros node init
rospy.init_node('eval')

# ===== load snapshot data =====
i_d = 0
# map_name = '50052751'
# map_name = '50056459'
map_name = '50045232'
map   = np.load('/home/lwcubuntu/data/snapshot_eval_data/' + map_name + '_map_msg_' + str(i_d)+'.npy', allow_pickle='TRUE').item()
NN    = nx.read_gpickle('/home/lwcubuntu/data/snapshot_eval_data/'+ map_name +'_NN_'+str(i_d)+'.gpickle')
start = np.load('/home/lwcubuntu/data/snapshot_eval_data/'+ map_name +'_robot_pos_'+str(i_d) +'.npy')

# convert "features" to "type"
features = nx.get_node_attributes(NN, 'features')
nx.set_node_attributes(NN, features, "type")

NN.remove_node(0)
# NN.remove_node(4)
NN, _ = relabel_network(NN)

# ===== call publisher =====
pub = Publishers()

# ===== plt NN with to_go =====
pos = nx.get_node_attributes(NN, 'pos')
labels = nx.get_node_attributes(NN, 'to_go')
nx.draw(NN, pos, labels=labels)
plt.show()

# ===== plot NN with value =====
pos = nx.get_node_attributes(NN, 'pos')
labels = nx.get_node_attributes(NN, 'value')
nx.draw(NN, pos, with_labels=True)
plt.show()

# ===== publish NN for visulization =====
msg_NN_jsonstr = pub.assign_NN(NN, ['value', 'pos', 'isrobot', 'to_go', 'type'])
pub.pub_NN.publish(msg_NN_jsonstr)

# ===== call srvs =====
print('===== start calling servicies =====')
rospy.wait_for_service('MBD_GSIM')
print('service MBD-GSIM is loaded')
rospy.wait_for_service('FSE')
print('service FSE is loaded')
rospy.wait_for_service('FVE_MBD_GSIM')
print('service FVE_MBD_GSIM is loaded')
rospy.wait_for_service('FVE_FSE')
print('service FVE_FSE is loaded')
rospy.wait_for_service('get_GEST')
print('service GEST is loaded')
rospy.wait_for_service('get_hector')
print('service hector planner is loaded')

srv_MBD_GSIM = rospy.ServiceProxy('MBD_GSIM', MBD_GSIM)
srv_FSE = rospy.ServiceProxy('FSE', FSE)
srv_FVE_MBD_GSIM = rospy.ServiceProxy('FVE_MBD_GSIM', pred_FVE)
srv_FVE_FSE = rospy.ServiceProxy('FVE_FSE', pred_FVE)
srv_get_path_GEST = rospy.ServiceProxy('get_GEST', get_path_GEST)
srv_get_path_hector = rospy.ServiceProxy('get_hector', get_path_hector)

# ==== get jsonstr NN =====
dict_NN = {}
E = [[v1, v2] for (v1, v2) in list(nx.edges(NN))]
C = {str(n): str(c) for n, c in nx.get_node_attributes(NN, 'type').items()}
T = {str(n): str(t) for n, t in nx.get_node_attributes(NN, 'to_go').items()}
dict_NN["edges"] = E
dict_NN["features"] = C
dict_NN["to_go"] = T
msg_NN_jsonstr = String()
msg_NN_jsonstr.data = str(dict_NN)

# ===== MBD-GSIM based MP & FVE prediction =====
print('============ MBD-GSIM ============')
MP_list = []
for node in list(NN.nodes()):
    output = srv_MBD_GSIM(msg_NN_jsonstr, node)
    MP = ast.literal_eval(output.marginal_probs.data)
    MP_list.append(MP)
    print(node, MP)

# FVE
FVE = get_nodes_FVE(NN, p, srv_FVE_MBD_GSIM)
print(FVE)

# MBD-GSIM goal
nx.set_node_attributes(NN, FVE, "value")
path = get_path(map, NN, srv_get_path_GEST)
goal_MBD_GSIM = path[-1]
msg_goal = pub.assign_cmd_goal(goal_MBD_GSIM, 1.0)
pub.pub_dummy_goal_MBD_GSIM.publish(msg_goal)
print('MBD-GSIM goal generation done')

# ===== FSE based MP & FVE prediction
print('=========== FSE ============')
for node in list(NN.nodes()):
    output = srv_FSE(msg_NN_jsonstr, node)
    MP = ast.literal_eval(output.marginal_probs.data)
    print(node, MP)

# FVE
FVE = get_nodes_FVE(NN, p, srv_FVE_FSE)
print(FVE)

# FSE goal
nx.set_node_attributes(NN, FVE, "value")
path = get_path(map, NN, srv_get_path_GEST)
goal_FSE = path[-1]
msg_goal = pub.assign_cmd_goal(goal_FSE, 1.0)
pub.pub_dummy_goal_FSE.publish(msg_goal)
print('FSE goal generation done')

# hector goal
print('============ hector =============')
msg_start = Point32()
msg_start.x = start[0]
msg_start.y = start[1]
output = srv_get_path_hector(map, msg_start)
path_msg = output.path
path = [(point.pose.position.x, point.pose.position.y) for point in path_msg.poses]
goal_hector = path[-1]
msg_goal = pub.assign_cmd_goal(goal_hector, 1.0)
pub.pub_dummy_goal_hector.publish(msg_goal)
print('hector goal generation done')




