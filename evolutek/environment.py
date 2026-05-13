from src.evo_robot.robot.robot import Robot
from evo_robot.perception.environment import Environment, EnvironmentManager

from evo_lib.interfaces.obstacles_provider import ObstacleProvider
from evo_lib.path_finding import PathFindingMap


class MyEnvironment(Environment):
    __slots__ = ["map"]

    def __init__(self, robot: Robot):
        self.map = PathFindingMap()


class MyEnvironmentManager(EnvironmentManager):
    def __init__(self, robot: Robot):
        self._robot = robot
        self._map: PathFindingMap

    def on_obstacles(self, obstacles: list[Obstacle]) -> None:
        for obstacle in obstacles:
            self._map.add_shape()

    def init(self) -> None:
        self._robot.get_perception().get_map()
        obstacles = self._robot.get_peripherals_manager().get_peripheral("aa", ObstacleProvider)
        obstacles.on_obstacles().register()

    def close(self) -> None:
        pass
