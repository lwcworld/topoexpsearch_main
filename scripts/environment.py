#!/usr/bin/env python
import subprocess

import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty

from gazebo_msgs.srv import SpawnModel, GetModelState, SetModelState, DeleteModel, GetWorldProperties, GetPhysicsProperties, SetPhysicsProperties
from gazebo_msgs.msg import ModelState

import cv2


# <environment>
# - open map (o)
# - get map configuration
# - map change (o)
# - accelerate simulation (o)
# - obtain local map (o)

class environment():
    def __init__(self):
        self.process_frontier_exploration = 0

        self.map_exist = False  # 0: empty, 1: map called
        self.map_name = []
        self.sim_speed = 1.
        self.center_floorplan = (0, 0)
        self.GT_map = []

        # set groundtruth map
        self.getinfo_env()
        if self.map_name != []:
            img = cv2.imread('/home/lwcubuntu/data/KTH_dataset/categorized_maps/' + self.map_name + '.png')
            img_flipped = cv2.flip(img, 0)
            self.GT_map = img_flipped

        # self.spawn_agent()
        # self.delete_agent()
        # self.set_model_state('turtlebot3_waffle_pi', 0, 0, 0)
        # self.spawn_map(self.map_name)
        # self.delete_map()
        # self.set_simulation_speed(2)
        # self.getinfo_env()
        # self.notify_map_status()
        # self.reset_simulation()
        # self.start_frontier_exploration()
        # self.stop_frontier_exploration()
        # self.map_save()
        # self.map_save_terminate()
        # self.get_local_costmap()
        # self.pause_physics()
        # self.unpause_physics()

    # pause physics
    def pause_physics(self):
        pause_physics = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        pause_physics()

    # unpause physics
    def unpause_physics(self):
        unpause_physics = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        unpause_physics()

    # save slam map
    def map_save(self):
        self.precess_map_saver = subprocess.Popen(["rosrun", "map_server", "map_saver"])

    # kill slam map save subprocess
    def map_save_terminate(self):
        self.precess_map_saver.terminate()

    # start local path planner
    def start_slam(self, method):
        # self.process_frontier_exploration = subprocess.Popen(
        #     ["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch", "slam_methods:=frontier_exploration_explore_lite"])
        self.process_slam = subprocess.Popen(
            ["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch",
             "slam_methods:="+method])

    # kill local path planner
    def stop_frontier_exploration(self):
        # self.process_frontier_exploration.kill()
        self.process_slam.terminate()

    # reset simulation
    def reset_simulation(self):
        reset_simulation = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        reset_simulation()

    # notify map status
    def notify_map_existence(self):
        if self.map_exist == False:
            print('map : empty')
        elif self.map_exist == True:
            print('map : ' + self.map_name)

    # set model state
    def set_model_state(self, name, pos_x, pos_y, pos_z):
        set_modelstate_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        modelstate = ModelState()
        modelstate.model_name = name
        modelstate.pose.position.x = pos_x
        modelstate.pose.position.y = pos_y
        modelstate.pose.position.z = pos_z
        modelstate.pose.orientation.x = 0.
        modelstate.pose.orientation.y = 0.
        modelstate.pose.orientation.z = 0.
        modelstate.pose.orientation.w =  1.0

        try:
            ret = set_modelstate_prox(modelstate)
            print(ret.status_message)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    # get information about env
    def getinfo_env(self):
        # get map information
        # boundary (min/max)
        # world properties
        get_worldproperty_prox = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
        worldproperty = get_worldproperty_prox()

        for model_name in worldproperty.model_names:
            if 'floorplan_' in model_name:
                self.map_name = model_name.replace('floorplan_', '')
                self.map_exist = True
            else:
                self.map_name = []
                self.map_exist = False

        get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
        physicsproperty = get_physics_properties()

        self.sim_speed = physicsproperty.time_step*physicsproperty.max_update_rate

    # simulation speed setting
    def set_simulation_speed(self, sim_speed):
        get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
        physicsproperty = get_physics_properties()

        origin_update_rate = physicsproperty.max_update_rate
        goal_update_rate = origin_update_rate * sim_speed

        origin_time_step = physicsproperty.time_step
        goal_time_step = origin_time_step / sim_speed

        set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
        # set_physics_properties_prox(time_step=goal_time_step)
        set_physics_properties_prox(max_update_rate=goal_update_rate)


    # spawn agent
    def spawn_agent(self):
        # spawn turtlebot
        f = open(
            '/home/lwcubuntu/workspaces/exploration/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf.xacro',
            'r')
        sdff = f.read()

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0

        ret = spawn_model_prox(model_name='turtlebot3_waffle_pi', model_xml=sdff,
                               robot_namespace='turtlebot3_waffle_pi', initial_pose=initial_pose,
                               reference_frame="world")
        success = ret.success

        return success

    # delete agent
    def delete_agent(self):
        rospy.wait_for_service('gazebo/delete_model')
        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        ret = del_model_prox('agent')
        success = ret.success
        return success

    # spawn map
    def spawn_map(self, map_name, dir_map):
        # spawn map
        f = open(dir_map + map_name + '/model.sdf', 'r')
        sdff = f.read()

        center_str = sdff.split('<pose frame="">')[-1].split(' ')[0:2]
        center = [float(center_str[0]), float(center_str[1])]


        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        initial_pose = Pose()

        initial_pose.position.x = center[0]
        initial_pose.position.y = center[1]

        ret = spawn_model_prox(model_name='floorplan_'+map_name, model_xml=sdff, robot_namespace='floorplan_'+map_name,
                               initial_pose=initial_pose, reference_frame="world")

        success = ret.success

        return success

    # delete map
    def delete_map(self):
        rospy.wait_for_service('gazebo/delete_model')
        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        ret = del_model_prox('floorplan_' + self.map_name)
        success = ret.success
        return success

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

if __name__=='__main__':
    env = environment()