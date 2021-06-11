import numpy as np

class simparam:
    def __init__(self):
        #

        # threshold condition time
        self.threshold_time = 5.

        # frequency in simulation time
        self.freq_est  = 1.
        self.freq_ctrl = 1.
        self.freq_rviz = 1.

        self.cmap_category = {'lab': [(0, 0, 255)],
                             'corridor': [(0, 150, 255), (255, 255, 0)],
                             'share': [(0, 255, 255)],
                             'storage': [(0, 255, 0)],
                             'toilet': [(255, 0, 0)],
                             'maintenence': [(0, 255, 0)],
                             'food': [(0, 255, 255)],
                             'pass': [(255,255,255)]}

        # self.imap_category = {'lab':0, 'corridor':1, 'share':2, 'storage':3, 'toilet':4, 'maintenence':5, 'food':6}
        self.imap_category = {'lab': 1, 'corridor': 2, 'share': 3, 'storage': 4, 'toilet': 5, 'maintenence': 4, 'food': 3, 'pass':0}

        # directories
        self.dir_log = '../dataset/log/'

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')