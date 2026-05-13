from evo_lib.types.transform import RigidTransform2D
from src.evo_robot.robot.robot import Robot
from evo_robot.perception.environment import Environment, EnvironmentManager

from evo_lib.interfaces.obstacles_provider import Obstacle, ObstacleProvider
from evo_lib.path_finding import PathFindingMap, PolygoneShape


class MyEnvironment(Environment):
    __slots__ = ["map"]

    def __init__(self, robot: Robot):
        self.map = PathFindingMap()


class MyEnvironmentManager(EnvironmentManager):
    def __init__(self, robot: Robot, env: MyEnvironment):
        self._robot = robot
        self._env = env
        self._obstacles: list[Obstacle] = []

    def _update_map(self) -> None:
        obstacle_radius = 160
        self._env.map.lock()
        self._env.map.clear_shapes()
        for obstacle in self._obstacles:
            obstacle_shape = PolygoneShape.create_regular(radius=obstacle_radius, vertexes=6) # Create hexagone
            obstacle_shape.transform(RigidTransform2D.create_translate(obstacle.position))
            self._env.map.add_shape(obstacle_shape)
        self._env.map.unlock()

    def _on_obstacles(self, obstacles: list[Obstacle]) -> None:
        self._obstacles = obstacles
        self._update_map()

    def init(self) -> None:
        obstacles = self._robot.get_peripherals_manager().get_peripheral("aa", ObstacleProvider)
        obstacles.on_obstacles().register(self._on_obstacles)

    def close(self) -> None:
        pass
