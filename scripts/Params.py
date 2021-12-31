import numpy as np

class param:
    def __init__(self):
        # sensor spec
        self.r            = 10.0 # [m]
        self.r_pix_scaled = 0

        #
        self.def_occ     = 0.0
        self.def_unknown = 0.5
        self.def_free    = 1.0

        #
        self.scale_per   = 20.0

        #
        self.N_c = 5

        self.remove_small_area    = True
        self.small_area_threshold = 4.0
        self.gate                 = 5.0

        # ===== parameters for FVE =====
        self.p_E = [0.6, 0.1, 0.1, 0.1, 0.1] # [1xN_c] target existence probability vector
        # self.p_E = [0.1, 0.1, 0.1, 0.1, 0.6] # [1xN_c] target existence probability vector
        # self.p_E = [0.75, 0.75, 0.75, 0.75, 0.7]
        # self.p_E = [0.1, 0.1, 0.6, 0.1, 0.1]  # [1xN_c] target existence probability vector

        self.K = 2 # rollout length (K)
        self.M = 1 # high member cut (M)
        self.gamma = 0.9 # discount factor (gamma)

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')