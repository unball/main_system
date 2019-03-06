from world import World

class TargetSelector:
    def __init__(self):
        pass

    def update(self, formation: list, world: World, centralZone=None):
        self.formation = formation
        self.world = world
        self.centralZone = centralZone
