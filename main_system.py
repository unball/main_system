"""Main module system. World -> Strategy -> Control."""

from world import World
import world_standards
from strategy import Strategy
from controllers.controller import Controller


def start_system():
    """Start the system in the 'main function'."""
    world_state = World(world_standards.STANDARD3)
    strategy_system = Strategy()
    control_system = Controller()
    # while True:
    world_state.dummy_update()
    strategy_system.plan(world_state)
    targets = strategy_system.get_targets()
    print(targets)
    control_system.actuate(targets)
    # Publish to communication node with ROS


if __name__ == '__main__':
        start_system()
