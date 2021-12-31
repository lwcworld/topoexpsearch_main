#!/usr/bin/env python
import rospy
from Params import *
from SimParams import *
from Variables import *
from Subscribers import *
from Publishers import *
from environment import *
from Utils import *
from Rviz import *
from topoexpsearch_FVE.srv import pred_FVE
from topoexpsearch_planner.srv import get_path_GEST

if __name__ == '__main__':
    rospy.init_node('topoexpsearch_main')

    # declare classes
    sp    = simparam()
    p     = param()
    q_t   = time_stamp()
    q_r   = robot()
    q_m   = map()
    q_f   = flag()
    q_p   = plan()
    q_h   = hector()
    q_NN  = NN()
    q_env = environment()

    # wait for MBD_GSIM
    # rospy.wait_for_service('FVE_MBD_GSIM')
    # srv_pred_FVE = rospy.ServiceProxy('FVE_MBD_GSIM', pred_FVE)
    # print('service FVE is loaded')

    # # wait for FVE_GSIM
    rospy.wait_for_service('FVE_FSE')
    srv_pred_FVE = rospy.ServiceProxy('FVE_FSE', pred_FVE)
    print('service FVE is loaded')

    # wait for GEST
    rospy.wait_for_service('get_GEST')
    srv_get_path_GEST = rospy.ServiceProxy('get_GEST', get_path_GEST)
    print('service GEST is loaded')

    # declare subscriber & publisher
    sub = Subscribers(q_t=q_t, q_m=q_m, q_r=q_r, q_h=q_h, q_f=q_f, q_p=q_p, q_NN=q_NN, p=p)
    pub = Publishers()

    # set iterator frequency
    freq = 1.0
    rate = rospy.Rate(freq)

    while q_f['map'] == False and q_f['odom'] == False:
        rospy.sleep(5)

    count = 1
    while not rospy.is_shutdown():
        q_t['now'] = rospy.get_rostime().to_time()

        if q_f['processing_FVE_plan'] == False and q_p['has_plan'] == False:
            q_f['processing_FVE_plan'] = True

            # condition to update NN (navigation network)
            cond_NN_update = [q_t['now'] - q_t['map'] <= sp.threshold_time * q_env.sim_speed,
                              q_t['now'] - q_t['NN'] <= sp.threshold_time * q_env.sim_speed,
                              q_t['now'] - q_t['odom'] <= sp.threshold_time * q_env.sim_speed,
                              q_t['now'] - q_t['robot_stop'] > sp.threshold_time * q_env.sim_speed,
                              q_f['new_map'] == True]

            print(cond_NN_update)
            if all(cond_NN_update):
                print('updating NN')
                q_f['new_map'] = False
                q_NN, q_m, q_r, p, q_f = get_voronoi(q_NN, q_m, q_r, q_f, q_env, p, sp)

                # publish NN for visualization
                msg_NN_jsonstr = pub.assign_NN(q_NN['global_network'], ['value', 'pos', 'isrobot', 'to_go', 'type'])
                pub.pub_NN.publish(msg_NN_jsonstr)
                q_f['new_NN'] = True

            if q_f['new_NN'] == True:
                q_f['new_NN'] = False

                # ===== FVE (call topoexpsearch_FVE) =====
                print('calculating FVE')
                FVE = get_nodes_FVE(q_NN['global_network'], p, srv_pred_FVE)
                nx.set_node_attributes(q_NN['global_network'], FVE, "value")

                # ===== plan (call topoexpsearch_planner) ====
                print('calculating path')
                path = get_path(q_m['map_msg'], q_NN['global_network'], srv_get_path_GEST)
                goal = path[-1]

                print('===== goal =====')
                print(goal)

                # ===== send goal =====
                msg_goal = pub.assign_cmd_goal(goal, 1.0)
                pub.pub_cmd_goal.publish(msg_goal)

                q_f['new_goal'] = True

            if q_f['new_goal'] == True:
                q_f['new_goal'] = False
                q_p['has_plan'] = True

            q_f['processing_FVE_plan'] = False

        if count >= freq:
            count = 1
        else:
            count = count + 1

        rate.sleep()

