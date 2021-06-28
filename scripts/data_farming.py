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

    # declare subscriber & publisher
    sub = Subscribers(q_t=q_t, q_m=q_m, q_r=q_r, q_h=q_h, q_f=q_f, q_p=q_p, q_NN=q_NN, p=p)
    pub = Publishers()

    # set iterator frequency
    freq = 1.0
    rate = rospy.Rate(freq)

    while q_f['map'] == False and q_f['odom'] == False:
        rospy.sleep(5)

    while not rospy.is_shutdown():
        q_t['now'] = rospy.get_rostime().to_time()

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
            msg_NN_jsonstr = pub.assign_NN(q_NN['global_network'], ['value', 'pos', 'isrobot', 'to_go', 'type'])
            pub.pub_NN.publish(msg_NN_jsonstr)
            q_f['new_NN'] = True

        if q_f['new_NN'] == True:
            # let's save and plot
            i_d = 0

            # draw map
            h = q_m['map_msg'].info.height
            w = q_m['map_msg'].info.width
            map = np.reshape(q_m['map_msg'].data, (h, w))
            m = np.zeros(np.shape(map))
            idx_free = np.where(map == 0)
            idx_occ = np.where(map == 100)
            idx_unknown = np.where(map == -1)
            m[idx_free] = 1
            m[idx_occ] = 2
            m[idx_unknown] = 3

            # plt.imshow(m)
            # plt.gca().invert_yaxis()
            # plt.show()

            # draw NN
            pos = nx.get_node_attributes(q_NN['global_network'], 'pos')
            labels = nx.get_node_attributes(q_NN['global_network'], 'to_go')
            nx.draw(q_NN['global_network'], pos, labels=labels)
            plt.show()

            # draw NN
            pos = nx.get_node_attributes(q_NN['global_network'], 'pos')
            labels = nx.get_node_attributes(q_NN['global_network'], 'type')
            nx.draw(q_NN['global_network'], pos, labels=labels)
            plt.show()

            # starting point of robot
            print('start point : ' + str(q_r['pos']))

            np.save('/home/lwcubuntu/data/snapshot_eval_data/'+q_env['map_name']+'_map_msg_'+str(i_d), q_m['map_msg'])
            nx.write_gpickle(q_NN['global_network'], '/home/lwcubuntu/data/snapshot_eval_data/'+q_env['map_name']+'_NN_'+str(i_d)+'.gpickle')
            np.save('/home/lwcubuntu/data/snapshot_eval_data/'+q_env['map_name']+'_robot_pos_'+str(i_d), q_r['pos'])

            print('done')
            break

        rate.sleep()

