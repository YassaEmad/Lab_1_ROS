#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute

from turtle_control import constants as C


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.clear_client = self.create_client(Empty, C.CLEAR_SERVICE,
                                               callback_group=self.group)
        self.teleport_client = self.create_client(TeleportAbsolute, C.TELEPORT_SERVICE,
                                                  callback_group=self.group)
        self.pen_client = self.create_client(SetPen, C.SET_PEN_SERVICE,
                                             callback_group=self.group)
        self.create_service(Trigger, 'reset_turtle', self.reset_callback,
                            callback_group=self.group)
        self.get_logger().info('reset_turtle service is up')

    def call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise RuntimeError(f'service {client.srv_name} timed out')
        return future.result()

    def set_pen(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b = C.PEN_R, C.PEN_G, C.PEN_B
        req.width = C.PEN_WIDTH
        req.off = 1 if off else 0
        self.call(self.pen_client, req)

    def reset_callback(self, request, response):
        with self.lock:
            pen_off = False
            try:
                self.set_pen(True)
                pen_off = True
                tp = TeleportAbsolute.Request()
                tp.x, tp.y, tp.theta = C.START_X, C.START_Y, C.START_THETA
                self.call(self.teleport_client, tp)
                time.sleep(C.TELEPORT_WAIT)
                self.call(self.clear_client, Empty.Request())
                response.success = True
                response.message = 'screen cleared, turtle is back at the start'
            except Exception as e:
                response.success = False
                response.message = f'reset failed: {e}'
                self.get_logger().error(response.message)
            finally:
                if pen_off:
                    try:
                        self.set_pen(False)
                    except Exception as e:
                        self.get_logger().error(f'could not turn the pen back on: {e}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
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
