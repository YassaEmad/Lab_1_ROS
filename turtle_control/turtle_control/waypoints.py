import math
from dataclasses import dataclass

from turtle_control import constants as C


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


class PathFactory:
    registry = {}

    @classmethod
    def register(cls, kind):
        def wrap(path_cls):
            cls.registry[kind] = path_cls
            return path_cls
        return wrap

    @classmethod
    def create(cls, kind, **kwargs):
        if kind not in cls.registry:
            raise ValueError(f'unknown path kind: {kind}')
        return cls.registry[kind](**kwargs)


@PathFactory.register('square')
class SquarePath:
    def __init__(self, x0, y0, side):
        if not math.isfinite(side) or side <= 0.0:
            raise ValueError('side length must be > 0')
        self.x0 = x0
        self.y0 = y0
        self.side = side
        low = C.WORLD_MIN + C.SAFE_MARGIN
        high = C.WORLD_MAX - C.SAFE_MARGIN
        for wp in self.waypoints():
            if not (low <= wp.x <= high and low <= wp.y <= high):
                raise ValueError(
                    f'square with side {side:.2f} starting at ({x0:.2f}, {y0:.2f}) '
                    f'goes outside [{low:.1f}, {high:.1f}], max side is {high - max(x0, y0):.2f}')

    def waypoints(self):
        x0, y0, s = self.x0, self.y0, self.side
        return [
            Waypoint(x0 + s, y0, math.pi / 2),
            Waypoint(x0 + s, y0 + s, math.pi),
            Waypoint(x0, y0 + s, -math.pi / 2),
            Waypoint(x0, y0, 0.0),
        ]
