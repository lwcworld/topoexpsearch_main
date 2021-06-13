#!/usr/bin/env python
from environment import environment
from Subscribers import *
import glob

savedir = '/home/lwcubuntu/workspaces/topoexpsearch/src/topoexpsearch/dataset/maplog/'
env = environment()

if __name__ == '__main__':
    rospy.init_node('map_robot_generation')

    # map_list = [dI for dI in os.listdir(dir_model) if os.path.isdir(os.path.join(dir_model, dI))]
    dir_model = '/home/lwcubuntu/data/KTH_dataset/kth_maps_preprocessing/models'
    dir_categorized_floorplans = '/home/lwcubuntu/data/KTH_dataset/categorized_maps/'
    categorized_floorplans = glob.glob(dir_categorized_floorplans + '*.png')
    map_name_list = []
    for i_m, map_png in enumerate(categorized_floorplans):
        map_name = map_png.split('/')[-1].split('.png')[0]
        map_name_list.append(map_name)


    # spawn map
    print('== spawn map start ..')
    env.getinfo_env()
    print('simulation speed : ' + str(env.sim_speed))
    if env.map_exist == False:
        env.spawn_map(map_name_list[22], dir_map = dir_model)
        print('spawned map : ' + map_name_list[22])
    else:
        print('map already exist : ' + env.map_name)
    print('== spawn map done .. ')

    # set agent
    print('== set robot state start ..')
    env.set_model_state("turtlebot3_burger", 0, 0, 0.0)
    print('== set robot state done ..')
    rospy.sleep(1*3)