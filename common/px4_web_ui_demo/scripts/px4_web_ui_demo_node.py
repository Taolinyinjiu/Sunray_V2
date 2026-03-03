#!/usr/bin/env python3
"""Serve a simple HTML page for PX4 web API demo."""

import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rospy


class UiHandler(BaseHTTPRequestHandler):
    html_data = ""
    api_base_url = ""

    def log_message(self, fmt, *args):
        rospy.loginfo("UI HTTP " + fmt, *args)

    def do_GET(self):  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            body = self.html_data.replace("__API_BASE_URL__", self.api_base_url)
            data = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        self.send_response(404)
        self.end_headers()


def main():
    rospy.init_node("px4_web_ui_demo")

    host = str(rospy.get_param("~host", "0.0.0.0"))
    port = int(rospy.get_param("~port", 8090))
    api_base_url = str(rospy.get_param("~api_base_url", "http://127.0.0.1:8080"))

    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    html_path = os.path.join(pkg_dir, "web", "index.html")
    with open(html_path, "r", encoding="utf-8") as f:
        UiHandler.html_data = f.read()
    UiHandler.api_base_url = api_base_url

    server = ThreadingHTTPServer((host, port), UiHandler)
    rospy.loginfo("PX4 web UI demo listening on http://%s:%d", host, port)
    rospy.loginfo("Using API base URL: %s", api_base_url)

    rospy.on_shutdown(lambda: server.shutdown())
    server.serve_forever()


if __name__ == "__main__":
    main()

