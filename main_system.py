"""Main module system. World -> Strategy -> Control."""

from world import World
from world_standards import *
from strategy import Strategy
from control import Control


def start_system():
    """Start the system in the 'main function'."""
    world_state = World(STANDARD3)
    strategy_system = Strategy()
    control_system = Control()
    # while True:
    world_state.dummy_update()
    strategy_system.plan(world_state)
    targets = strategy_system.get_targets()
    control_system.actuate(targets)
    # Publish to communication node with ROS


if __name__ == '__main__':
        start_system()
