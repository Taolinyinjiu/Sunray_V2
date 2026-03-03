#!/usr/bin/env python3
"""PX4 Web API demo node.

Exposes simple HTTP endpoints and forwards read/write requests to:
  /<uav_name><uav_id>/mavros/param/get
  /<uav_name><uav_id>/mavros/param/set
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Tuple
from urllib.parse import parse_qs, urlparse

import rospy
from mavros_msgs.srv import ParamGet, ParamGetRequest, ParamSet, ParamSetRequest


class Px4ParamApiBridge:
    """ROS service wrapper for MAVROS PX4 parameters."""

    def __init__(self) -> None:
        uav_id = int(rospy.get_param("~uav_id", rospy.get_param("uav_id", 1)))
        uav_name = str(rospy.get_param("~uav_name", rospy.get_param("uav_name", "uav")))
        self.host = str(rospy.get_param("~host", "0.0.0.0"))
        self.port = int(rospy.get_param("~port", 8080))

        mavros_ns = "/{}{}".format(uav_name, uav_id) + "/mavros"
        self.param_get_name = mavros_ns + "/param/get"
        self.param_set_name = mavros_ns + "/param/set"

        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service(self.param_get_name, timeout=5.0)
        rospy.wait_for_service(self.param_set_name, timeout=5.0)
        self.param_get = rospy.ServiceProxy(self.param_get_name, ParamGet)
        self.param_set = rospy.ServiceProxy(self.param_set_name, ParamSet)
        rospy.loginfo("PX4 Web API bound to MAVROS namespace: %s", mavros_ns)

    def get_param(self, name: str) -> Dict[str, Any]:
        req = ParamGetRequest()
        req.param_id = name
        rsp = self.param_get(req)
        if not rsp.success:
            raise RuntimeError("ParamGet failed for '{}'".format(name))
        return {
            "name": name,
            "integer": int(rsp.value.integer),
            "real": float(rsp.value.real),
        }

    def set_param(self, name: str, value: Any, value_type: str) -> Dict[str, Any]:
        req = ParamSetRequest()
        req.param_id = name

        if value_type == "int":
            req.value.integer = int(value)
            req.value.real = 0.0
        elif value_type == "float":
            req.value.real = float(value)
            req.value.integer = 0
        else:
            raise ValueError("value_type must be 'int' or 'float'")

        rsp = self.param_set(req)
        if not rsp.success:
            raise RuntimeError("ParamSet failed for '{}'".format(name))
        return {
            "name": name,
            "integer": int(rsp.value.integer),
            "real": float(rsp.value.real),
            "type": value_type,
        }


def _json(handler: BaseHTTPRequestHandler, code: int, payload: Dict[str, Any]) -> None:
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
    handler.send_header("Access-Control-Allow-Headers", "Content-Type")
    handler.send_header("Content-Length", str(len(data)))
    handler.end_headers()
    handler.wfile.write(data)


def _read_json(handler: BaseHTTPRequestHandler) -> Tuple[Dict[str, Any], str]:
    length = int(handler.headers.get("Content-Length", "0"))
    if length <= 0:
        return {}, "empty body"
    raw = handler.rfile.read(length).decode("utf-8")
    try:
        return json.loads(raw), ""
    except json.JSONDecodeError as exc:
        return {}, "invalid json: {}".format(exc)


def make_handler(bridge: Px4ParamApiBridge):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            rospy.loginfo("HTTP " + fmt, *args)

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Max-Age", "600")
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/health":
                _json(self, 200, {"ok": True})
                return

            if parsed.path == "/param":
                qs = parse_qs(parsed.query)
                name = (qs.get("name") or [""])[0]
                if not name:
                    _json(self, 400, {"ok": False, "error": "missing query param 'name'"})
                    return
                try:
                    value = bridge.get_param(name)
                    _json(self, 200, {"ok": True, "data": value})
                except Exception as exc:  # broad by design for demo endpoint
                    _json(self, 500, {"ok": False, "error": str(exc)})
                return

            _json(self, 404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/param":
                body, err = _read_json(self)
                if err:
                    _json(self, 400, {"ok": False, "error": err})
                    return

                name = str(body.get("name", ""))
                if not name:
                    _json(self, 400, {"ok": False, "error": "missing field 'name'"})
                    return

                if "value" not in body:
                    _json(self, 400, {"ok": False, "error": "missing field 'value'"})
                    return

                value = body["value"]
                value_type = str(body.get("type", ""))
                if not value_type:
                    value_type = "int" if isinstance(value, int) else "float"

                try:
                    result = bridge.set_param(name, value, value_type)
                    _json(self, 200, {"ok": True, "data": result})
                except Exception as exc:  # broad by design for demo endpoint
                    _json(self, 500, {"ok": False, "error": str(exc)})
                return

            _json(self, 404, {"ok": False, "error": "not found"})

    return Handler


def main() -> None:
    rospy.init_node("px4_web_api_demo")
    bridge = Px4ParamApiBridge()
    server = ThreadingHTTPServer((bridge.host, bridge.port), make_handler(bridge))
    rospy.loginfo("PX4 web API demo listening on http://%s:%d", bridge.host, bridge.port)

    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()

    def _shutdown() -> None:
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass

    rospy.on_shutdown(_shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
