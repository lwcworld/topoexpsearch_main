#!/usr/bin/env python
import rospy
from Params import *
from SimParams import *
from Variables import *
from Subscribers import *
from Publishers import *
from environment import *
from Rviz import *
from hector_nav_msgs.srv import GetRobotTrajectory

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

    # wait for hector
    rospy.wait_for_service('get_exploration_path')
    get_exploration_path = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)

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
                              q_t['now'] - q_t['odom'] <= sp.threshold_time * q_env.sim_speed,
                              q_t['now'] - q_t['robot_stop'] > sp.threshold_time * q_env.sim_speed,
                              q_f['new_map'] == True]

            print(cond_NN_update)
            if all(cond_NN_update):
                flag = False
                while flag == False:
                    path = get_exploration_path()
                    if len(path.trajectory.poses)>0:
                        flag = True
                goal = (path.trajectory.poses[-1].pose.position.x, path.trajectory.poses[-1].pose.position.y)

                print('goal : ' + str(goal))
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

