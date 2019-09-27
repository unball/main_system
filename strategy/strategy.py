"""Strategy system module."""

from strategy.movimentsDecider import Attacker, Defender, Goalkeeper, Midfielder, MovimentsDecider
from statics.static_classes import world

import strategy.frameRenderer

def error():
    """Print the standard error message for STRATEGY scope."""
    print("\nSTRATEGY ERROR:")


class Strategy(object):
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.coach = None
        self.tactic = None
        self.decider = MovimentsDecider()
        self.targets = []
        self.spin = [0,0,0]
        self.step = 0.005
        
        self._frameRenderers = {
            "parametrosEstrategia": strategy.frameRenderer.parametrosEstrategia(self),
            "gerenciadorEntidades": strategy.frameRenderer.gerenciadorEntidades(self)
        }

    def plan(self):
        """Toplevel planner which contains all the deciders of the system."""
        # TODO: VERIFICATION TEST FOR THE WORLD STATE

        # # Fuzzy
        # self.tactic = self.firstLvlDec.plan(self.world)
        # self.decider.setParams(world)
        # self.decider.setFormation(world)
        # self.targets = self.decider.updateTargets()
        
        #self.decider.setFormation()
        
        #if self.dynamicPossession:
        self.decider.updadeHost()
#        else:
#            self.decider.calcPath()

        self.targets = []
        for robot in world.robots:
            robot.discretize(self.step)
            self.targets.append(robot.nextStep())
            
    def setTurningRadius(self, radius):
        self.decider.turning_radius = radius
            
    def setDynamicPossession(self, value):
        self.decider.dynamicPossession = value
    
    def get_targets(self):
        """Getter of each robot target planned."""
        return self.targets, self.spin
