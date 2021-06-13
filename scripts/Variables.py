class Node_voronoi:
    def __init__(self, index, pos, to_go, isrobot, value, type):
        self.index = index
        self.pos = pos
        self.to_go = to_go
        self.isrobot = isrobot
        self.value = value
        self.type  = type

class NN:
    def __init__(self):
        self.NN_msg = []
        self.origin_x = []
        self.origin_y = []
        self.vertices = []
        self.global_network = []

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class map:
    def __init__(self):
        self.res = []
        self.map_msg = []
        self.map_1d         = [] # (unknown:255, free:0, occ:100)
        self.map_2d         = [] # (unknown:255, free:0, occ:100)
        self.width      = 0
        self.height      = 0
        self.origin       = (0,0)

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class hector:
    def __init__(self):
        self.g_et_pcl = [[], [], []] # grid cell exploration_transform pointclouds list
        self.g_ot_pcl = [[], [], []]
        self.g_et_pcl_local = [[], [], []] # local grid cell exploration_transform pointclouds list
        self.f_pcl_local = [[], [], []]  # frontier pointclouds of local grid cell et list
        self.path = [[], [], []]  # path from hector


    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class navigation_network:
    def __init__(self):
        self.NN = [] # navigation network

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class plan:
    def __init__(self):
        self.path = [[], [], []] # exploration_transform pointclouds list
        self.status = 0 # 1: ongoing, 0 : finished

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class robot:
    def __init__(self):
        self.pos = (0,0)

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class time_stamp:
    def __init__(self):
        self.now = 0  # simulation time
        self.map = 0 # map topic
        self.ET = 0   # exploration transform from hector
        self.path_hector = 0 # path from hector
        self.odom = 0 # robot odom
        self.NN = 0 # navigation network
        self.robot_stop = 0 # robot stop time
        self.mission_status = 0 # status of mission

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class flag:
    def __init__(self):
        self.map = False
        self.odom = False
        self.ET = False # processing of exploration transform
        self.OT = False


        self.new_map = False
        self.new_NN = False

        self.processing_FVE_plan = False

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')