"""
Serves the static web controller UI on http://0.0.0.0:8080.
Open http://<workstation-ip>:8080 from any device on the same network.
"""
import atexit
import os
import socket
import threading
import http.server
import functools

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class _ReusableHTTPServer(http.server.HTTPServer):
    # Allow re-binding the port immediately on restart (TIME_WAIT bypass)
    allow_reuse_address = True


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        static_dir = os.path.join(
            get_package_share_directory('web_pkg'), 'static')

        handler = functools.partial(
            http.server.SimpleHTTPRequestHandler,
            directory=static_dir)

        self._server = _ReusableHTTPServer(('0.0.0.0', port), handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=False)
        self._thread.start()
        self._shutdown_done = False

        self.get_logger().info(
            f'Web controller available at http://0.0.0.0:{port}')

    def shutdown_http(self):
        """Idempotent HTTP-server teardown. Safe to call from signal
        handlers, atexit, and destroy_node."""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        try:
            self._server.shutdown()
        except Exception:
            pass
        try:
            self._server.server_close()
        except Exception:
            pass
        try:
            self._thread.join(timeout=0.5)
        except Exception:
            pass

    def destroy_node(self):
        self.shutdown_http()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()

    # Port release on exit: the finally clause below, plus atexit as a
    # fallback. No custom SIGINT/SIGTERM handler: on Jazzy replacing rclpy's
    # handler leaves rclpy.spin() blocked in its wait set, so the process
    # never exits and keeps port 8080 bound. rclpy's own handler makes spin()
    # raise KeyboardInterrupt, which the try below already handles.
    atexit.register(node.shutdown_http)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_http()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
