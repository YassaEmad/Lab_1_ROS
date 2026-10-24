#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from turtle_control import constants as C
from turtle_control.waypoints import PathFactory
from turtle_interfaces.action import DrawSquare, GoToPose
from turtle_interfaces.msg import TargetPose


class Cancelled(Exception):
    pass


class Failed(Exception):
    pass


class DrawSquareServer(Node):
    def __init__(self):
        super().__init__('draw_square_server')
        self.group = ReentrantCallbackGroup()
        self.busy = False
        self.busy_lock = threading.Lock()

        self.declare_parameter('default_side_length', 3.0)
        self.declare_parameter('start_x', C.START_X)
        self.declare_parameter('start_y', C.START_Y)

        self.reset_client = self.create_client(Trigger, 'reset_turtle',
                                               callback_group=self.group)
        self.goto_client = ActionClient(self, GoToPose, 'go_to_pose',
                                        callback_group=self.group)
        self.server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group)
        self.get_logger().info('draw_square server is up')

    def get_side(self, requested):
        if requested > 0.0:
            return float(requested)
        return float(self.get_parameter('default_side_length').value)

    def make_path(self, side):
        return PathFactory.create('square',
                                  x0=float(self.get_parameter('start_x').value),
                                  y0=float(self.get_parameter('start_y').value),
                                  side=side)

    def wait_for(self, future, goal_handle, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout
        while not done.wait(0.05):
            if goal_handle.is_cancel_requested:
                raise Cancelled()
            if not rclpy.ok():
                raise Failed('shutdown')
            if time.monotonic() > deadline:
                raise Failed('timed out waiting for a server')
        return future.result()

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request):
        side = goal_request.side_length
        if not math.isfinite(side) or side < 0.0:
            self.get_logger().warn('goal rejected, bad side length')
            return GoalResponse.REJECT
        try:
            self.make_path(self.get_side(side))
        except ValueError as e:
            self.get_logger().warn(f'goal rejected: {e}')
            return GoalResponse.REJECT
        with self.busy_lock:
            if self.busy:
                self.get_logger().warn('goal rejected, a square is already running')
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        try:
            return self.run(goal_handle)
        finally:
            with self.busy_lock:
                self.busy = False

    def run(self, goal_handle):
        result = DrawSquare.Result()
        result.sides_completed = 0
        feedback = DrawSquare.Feedback()
        side = self.get_side(goal_handle.request.side_length)
        waypoints = self.make_path(side).waypoints()
        inner = None

        def publish(current, status):
            feedback.current_side = current
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        try:
            publish(0, 'resetting turtle')
            if not self.reset_client.wait_for_service(timeout_sec=5.0):
                raise Failed('reset_turtle service not available')
            reset = self.wait_for(self.reset_client.call_async(Trigger.Request()),
                                  goal_handle, 15.0)
            if not reset.success:
                raise Failed(f'reset failed: {reset.message}')

            if not self.goto_client.wait_for_server(timeout_sec=5.0):
                raise Failed('go_to_pose action server not available')

            for i, wp in enumerate(waypoints, start=1):
                publish(i, f'side {i}/4, going to ({wp.x:.2f}, {wp.y:.2f})')
                goal = GoToPose.Goal()
                goal.target = TargetPose(x=float(wp.x), y=float(wp.y), yaw=float(wp.yaw))
                inner = self.wait_for(self.goto_client.send_goal_async(goal),
                                      goal_handle, 5.0)
                if not inner.accepted:
                    inner = None
                    raise Failed(f'go_to_pose rejected side {i}')
                wrapped = self.wait_for(inner.get_result_async(), goal_handle,
                                        C.GOAL_TIMEOUT + 10.0)
                inner = None
                if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
                    raise Failed(f'side {i} failed: {wrapped.result.message}')
                result.sides_completed = i
                self.get_logger().info(f'side {i}/4 done')

            goal_handle.succeed()
            result.success = True
            result.message = 'square done with 4 go_to_pose goals'
            return result

        except Cancelled:
            if inner is not None:
                inner.cancel_goal_async()
            goal_handle.canceled()
            result.success = False
            result.message = 'canceled'
            return result
        except Failed as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            return result


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareServer()
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
