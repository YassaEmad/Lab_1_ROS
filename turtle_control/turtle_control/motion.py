import math
from abc import ABC, abstractmethod
from dataclasses import dataclass


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Limits:
    max_linear: float
    max_angular: float


@dataclass(frozen=True)
class Tolerance:
    position: float = 0.05
    yaw: float = 0.03
    heading: float = 0.05


@dataclass(frozen=True)
class Cmd:
    linear: float = 0.0
    angular: float = 0.0


class PController:
    def __init__(self, gain):
        self.gain = gain

    def __call__(self, error, limit):
        return clamp(self.gain * error, -limit, limit)


def distance_to(pose, goal):
    return math.hypot(goal.x - pose.x, goal.y - pose.y)


def heading_error(pose, goal):
    return normalize_angle(math.atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta)


def get_errors(pose, goal):
    return distance_to(pose, goal), normalize_angle(goal.theta - pose.theta)


class State(ABC):
    name = 'state'
    finished = False

    @abstractmethod
    def step(self, pose, goal, limits, tol):
        pass


class RotateToHeading(State):
    name = 'rotate_to_heading'

    def __init__(self, turn=None):
        self.turn = turn or PController(4.0)

    def step(self, pose, goal, limits, tol):
        if distance_to(pose, goal) < tol.position:
            return Cmd(), RotateToYaw(self.turn)
        err = heading_error(pose, goal)
        if abs(err) < tol.heading:
            return Cmd(), DriveToPoint(self.turn)
        return Cmd(angular=self.turn(err, limits.max_angular)), self


class DriveToPoint(State):
    name = 'drive_to_point'
    CLOSE = 0.2

    def __init__(self, turn=None, speed=None):
        self.turn = turn or PController(4.0)
        self.speed = speed or PController(1.5)

    def step(self, pose, goal, limits, tol):
        dist = distance_to(pose, goal)
        if dist < tol.position:
            return Cmd(), RotateToYaw(self.turn)
        err = heading_error(pose, goal)
        if dist > self.CLOSE and abs(err) > math.pi / 4:
            return Cmd(), RotateToHeading(self.turn)
        v = self.speed(dist, limits.max_linear)
        w = self.turn(err, limits.max_angular) if dist > self.CLOSE else 0.0
        return Cmd(linear=v, angular=w), self


class RotateToYaw(State):
    name = 'rotate_to_yaw'

    def __init__(self, turn=None):
        self.turn = turn or PController(4.0)

    def step(self, pose, goal, limits, tol):
        err = normalize_angle(goal.theta - pose.theta)
        if abs(err) < tol.yaw:
            return Cmd(), Done()
        return Cmd(angular=self.turn(err, limits.max_angular)), self


class Done(State):
    name = 'done'
    finished = True

    def step(self, pose, goal, limits, tol):
        return Cmd(), self


class PoseController:
    def __init__(self):
        self.state = RotateToHeading()

    @property
    def phase(self):
        return self.state.name

    @property
    def finished(self):
        return self.state.finished

    def update(self, pose, goal, limits, tol):
        for _ in range(4):
            cmd, nxt = self.state.step(pose, goal, limits, tol)
            if nxt is self.state:
                return cmd
            self.state = nxt
        return Cmd()
