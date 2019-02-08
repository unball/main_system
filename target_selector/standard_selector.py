from target_selector.target_selector import TargetSelector

class StandardSelector(TargetSelector):
    def __init__(self):
        pass

    def calcTargets(self, formation = None, world = None, centralZone = None):
        if formation == None:
            _formation = list(player.target for player in self.formation)
            return _formation
        elif isinstance(formation, list):
            _formation = list(player.target for player in formation)
        else:
            raise TypeError("Expected a formation with type 'list'")
