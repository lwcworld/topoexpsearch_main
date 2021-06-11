import numpy as np

class param:
    def __init__(self):
        # sensor spec
        self.r            = 7.0 # [m]
        self.r_pix_scaled = 0

        #
        self.def_occ     = 0.0
        self.def_unknown = 0.5
        self.def_free    = 1.0

        #
        self.scale_per   = 10.0

        #
        self.class_level = 5

        self.remove_small_area    = True
        self.small_area_threshold = 4.0
        self.gate                 = 5.0


    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')