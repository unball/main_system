class World(object):
    def __init__(self):
        pass

    def update(self):
        self.past_info = self.instant_info
        #self.instant_info = publisher
        pass

    def get_inst_info(self):
        return self.instant_info

    def calc_velocities(self):
        for obj in self.instant_info:
            if self.instant_info[obj][pos] != None and self.past_info[obj][pos]
