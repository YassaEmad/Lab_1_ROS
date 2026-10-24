#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose as TurtlePose

from turtle_control import constants as C
from turtle_control.motion import (Cmd, Limits, Pose, PoseController, Tolerance,
                                   get_errors, normalize_angle)
from turtle_interfaces.action import GoToPose
from turtle_interfaces.srv import SetSpeedLimits

SPEED_PARAMS = ('max_linear_speed', 'max_angular_speed')


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self.group = ReentrantCallbackGroup()
        self.pose = None
        self.pose_lock = threading.Lock()
        self.busy = False
        self.busy_lock = threading.Lock()

        self.add_on_set_parameters_callback(self.check_params)
        self.declare_parameter('max_linear_speed', C.DEFAULT_MAX_LINEAR)
        self.declare_parameter('max_angular_speed', C.DEFAULT_MAX_ANGULAR)
        self.declare_parameter('position_tolerance', Tolerance.position)
        self.declare_parameter('yaw_tolerance', Tolerance.yaw)

        self.cmd_pub = self.create_publisher(Twist, C.CMD_VEL_TOPIC, 10)
        self.create_subscription(TurtlePose, C.POSE_TOPIC, self.pose_callback, 10,
                                 callback_group=self.group)
        self.create_service(SetSpeedLimits, 'set_speed_limits', self.set_speed_callback,
                            callback_group=self.group)
        self.server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group)
        self.get_logger().info('go_to_pose server is up')

    def check_params(self, params):
        for p in params:
            if p.name not in SPEED_PARAMS:
                continue
            if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                return SetParametersResult(successful=False,
                                           reason=f'{p.name} must be a number')
            value = float(p.value)
            if not math.isfinite(value) or value <= 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{p.name} must be > 0 (got {value})')
            if value > C.SPEED_UPPER_BOUND:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must be <= {C.SPEED_UPPER_BOUND} (got {value})')
        return SetParametersResult(successful=True)

    def set_speed_callback(self, request, response):
        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, request.max_linear_speed),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, request.max_angular_speed),
        ])
        response.success = result.successful
        if result.successful:
            response.message = (f'new limits: linear={request.max_linear_speed:.2f}, '
                                f'angular={request.max_angular_speed:.2f}')
            self.get_logger().info(response.message)
        else:
            response.message = f'rejected, limits unchanged: {result.reason}'
            self.get_logger().warn(response.message)
        return response

    def get_limits(self):
        return Limits(float(self.get_parameter('max_linear_speed').value),
                      float(self.get_parameter('max_angular_speed').value))

    def get_tolerance(self):
        return Tolerance(position=float(self.get_parameter('position_tolerance').value),
                         yaw=float(self.get_parameter('yaw_tolerance').value))

    def pose_callback(self, msg):
        with self.pose_lock:
            self.pose = Pose(msg.x, msg.y, msg.theta)

    def get_pose(self):
        with self.pose_lock:
            return self.pose

    def wait_for_pose(self, timeout=2.0):
        t0 = time.monotonic()
        while self.get_pose() is None and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        return self.get_pose()

    def send_cmd(self, cmd):
        msg = Twist()
        msg.linear.x = float(cmd.linear)
        msg.angular.z = float(cmd.angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_cmd(Cmd())

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request):
        t = goal_request.target
        if not all(math.isfinite(v) for v in (t.x, t.y, t.yaw)):
            self.get_logger().warn('goal rejected, value is not finite')
            return GoalResponse.REJECT
        if not (C.WORLD_MIN <= t.x <= C.WORLD_MAX and C.WORLD_MIN <= t.y <= C.WORLD_MAX):
            self.get_logger().warn(
                f'goal rejected, ({t.x:.2f}, {t.y:.2f}) is outside the window')
            return GoalResponse.REJECT
        with self.busy_lock:
            if self.busy:
                self.get_logger().warn('goal rejected, another goal is running')
                return GoalResponse.REJECT
            self.busy = True
        self.get_logger().info(f'goal accepted: x={t.x:.2f} y={t.y:.2f} yaw={t.yaw:.2f}')
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        try:
            return self.run(goal_handle)
        finally:
            self.stop()
            with self.busy_lock:
                self.busy = False

    def run(self, goal_handle):
        target = goal_handle.request.target
        goal = Pose(float(target.x), float(target.y), normalize_angle(float(target.yaw)))
        result = GoToPose.Result()
        feedback = GoToPose.Feedback()
        controller = PoseController()

        def finish(ok, message, pose):
            if pose is not None:
                dist, yaw_err = get_errors(pose, goal)
                result.final_distance_error = float(dist)
                result.final_yaw_error = float(yaw_err)
            result.success = ok
            result.message = message
            return result

        pose = self.wait_for_pose()
        if pose is None:
            goal_handle.abort()
            return finish(False, 'no pose received, is turtlesim running?', None)

        period = 1.0 / C.CONTROL_RATE_HZ
        start = last_fb = time.monotonic()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                return finish(False, 'canceled', self.get_pose())

            pose = self.get_pose()
            cmd = controller.update(pose, goal, self.get_limits(), self.get_tolerance())
            if controller.finished:
                self.stop()
                goal_handle.succeed()
                self.get_logger().info('pose reached')
                return finish(True, 'pose reached', pose)

            self.send_cmd(cmd)
            now = time.monotonic()
            if now - last_fb >= C.FEEDBACK_PERIOD:
                dist, yaw_err = get_errors(pose, goal)
                feedback.distance_remaining = float(dist)
                feedback.yaw_remaining = float(yaw_err)
                feedback.phase = controller.phase
                goal_handle.publish_feedback(feedback)
                last_fb = now
            if now - start > C.GOAL_TIMEOUT:
                self.stop()
                goal_handle.abort()
                return finish(False, 'timeout', pose)
            time.sleep(period)

        goal_handle.abort()
        return finish(False, 'shutdown', self.get_pose())


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
