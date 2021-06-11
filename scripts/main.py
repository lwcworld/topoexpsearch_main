#!/usr/bin/env python
import rospy
from Params import *
from SimParams import *
from Variables import *
from Subscribers import *
from environment import *
from Utils import *
from topoexpsearch_FVE.srv import pred_FVE
from topoexpsearch_planner.srv import get_path_GEST

if __name__ == '__main__':
    rospy.init_node('topoexpsearch')

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

    # # wait for FVE predict server
    # rospy.wait_for_service('pred_FVE')
    # srv_pred_FVE = rospy.ServiceProxy('pred_FVE', pred_FVE)
    #
    # # wait for GEST path server
    # rospy.wait_for_service('get_path_GEST')
    # srv_get_path_GEST = rospy.ServiceProxy('get_path_GEST', get_path_GEST)

    # declare subscriber & publisher
    sub = Subscribers(q_t=q_t, q_m=q_m, q_r=q_r, q_h=q_h, q_f=q_f, q_p=q_p, q_NN=q_NN, p=p)

    # set iterator frequency
    freq_est = sp['freq_est']
    freq_ctrl = sp['freq_ctrl']
    freq_rviz = sp['freq_rviz']
    freq = lcm(lcm(freq_est, freq_ctrl), freq_rviz)
    rate = rospy.Rate(freq)

    while q_f['map']==False and q_f['odom']==False:
        rospy.sleep(5)

    count = 1
    while not rospy.is_shutdown():
        q_t['now'] = rospy.get_rostime().to_time()

        conditions_NN_update = [count % (freq / freq_est)==0,
                                q_t['now']-q_t['map']<=sp.threshold_time*q_env.sim_speed,
                                q_t['now']-q_t['NN']<=sp.threshold_time*q_env.sim_speed,
                                q_t['now']-q_t['odom']<=sp.threshold_time*q_env.sim_speed,
                                q_t['now']-q_t['robot_stop']>sp.threshold_time*q_env.sim_speed,
                                q_f['map_used']==False]
        print(conditions_NN_update)

        if all(conditions_NN_update):
            q_f['map_used'] = True
            q_NN, q_m, q_r, p, q_f = get_voronoi(q_NN, q_m, q_r, q_f, q_env, p, sp)

        if count >= freq:
            count = 1
        else:
            count = count + 1

        rate.sleep()


