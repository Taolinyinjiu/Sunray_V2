#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import lzma
import os
from typing import Any, Dict, List, Optional
import xml.etree.ElementTree as ET

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def _safe_get(obj: Dict[str, Any], key: str, default: Any = None) -> Any:
    value = obj.get(key, default)
    return default if value is None else value


def _normalize_type(px4_type: str) -> str:
    t = (px4_type or "").lower()
    if "float" in t or "double" in t:
        return "float"
    if "bool" in t:
        return "bool"
    if "int" in t:
        return "int"
    return "unknown"


def _normalize_param(entry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    name = str(_safe_get(entry, "name", "")).strip()
    if not name:
        return None

    enum_values = _safe_get(entry, "values", [])
    bitmask_values = _safe_get(entry, "bitmask", [])

    enum_items: List[Dict[str, Any]] = []
    for item in enum_values:
        if not isinstance(item, dict):
            continue
        enum_items.append(
            {
                "value": item.get("value"),
                "label": item.get("description", ""),
            }
        )

    bitmask_items: List[Dict[str, Any]] = []
    for item in bitmask_values:
        if not isinstance(item, dict):
            continue
        bitmask_items.append(
            {
                "index": item.get("index"),
                "label": item.get("description", ""),
            }
        )

    kind = "scalar"
    if bitmask_items:
        kind = "bitmask"
    elif enum_items:
        kind = "enum"

    return {
        "name": name,
        "kind": kind,
        "type": _normalize_type(str(_safe_get(entry, "type", ""))),
        "raw_type": _safe_get(entry, "type", ""),
        "default": _safe_get(entry, "default"),
        "min": _safe_get(entry, "min"),
        "max": _safe_get(entry, "max"),
        "increment": _safe_get(entry, "increment"),
        "decimal_places": _safe_get(entry, "decimalPlaces"),
        "unit": _safe_get(entry, "units", ""),
        "category": _safe_get(entry, "category", ""),
        "group": _safe_get(entry, "group", ""),
        "short_desc": _safe_get(entry, "shortDesc", ""),
        "long_desc": _safe_get(entry, "longDesc", ""),
        "reboot_required": bool(_safe_get(entry, "rebootRequired", False)),
        "enum": enum_items,
        "bitmask": bitmask_items,
    }


def _read_json_or_xz(path: str) -> Dict[str, Any]:
    if path.endswith(".xz"):
        with lzma.open(path, "rt", encoding="utf-8") as f:
            return json.load(f)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _parse_xml(path: str) -> Dict[str, Any]:
    root = ET.parse(path).getroot()
    params = []
    for p in root.findall(".//parameter"):
        item: Dict[str, Any] = {
            "name": p.get("name", ""),
            "type": p.get("type", ""),
            "units": p.get("unit", ""),
            "min": p.get("min"),
            "max": p.get("max"),
            "default": p.get("default"),
            "shortDesc": p.get("short_desc", ""),
            "longDesc": p.get("long_desc", ""),
            "group": p.get("group", ""),
            "category": p.get("category", ""),
        }
        values = []
        for v in p.findall("./values/value"):
            values.append({"value": v.get("code"), "description": (v.text or "").strip()})
        if values:
            item["values"] = values
        bitmask = []
        for b in p.findall("./bitmask/bit"):
            bitmask.append({"index": b.get("index"), "description": (b.text or "").strip()})
        if bitmask:
            item["bitmask"] = bitmask
        params.append(item)
    return {"parameters": params}


def load_metadata(path: str) -> Dict[str, Dict[str, Any]]:
    if not os.path.isfile(path):
        raise FileNotFoundError("metadata file not found: {}".format(path))

    lower = path.lower()
    if lower.endswith(".json") or lower.endswith(".json.xz"):
        raw = _read_json_or_xz(path)
    elif lower.endswith(".xml"):
        raw = _parse_xml(path)
    else:
        raise ValueError("unsupported metadata suffix: {}".format(path))

    params = raw.get("parameters", [])
    out: Dict[str, Dict[str, Any]] = {}
    for entry in params:
        if not isinstance(entry, dict):
            continue
        p = _normalize_param(entry)
        if p is None:
            continue
        out[p["name"]] = p
    return out


class Px4ParamMetaServer:
    def __init__(self, metadata_path: str):
        self._metadata_path = metadata_path
        self._param_map: Dict[str, Dict[str, Any]] = {}

        self._pub = rospy.Publisher("~result", String, queue_size=20)
        self._sub = rospy.Subscriber("~query", String, self._on_query, queue_size=20)
        self._reload_srv = rospy.Service("~reload", Trigger, self._on_reload)

        self.reload()

    def reload(self) -> int:
        self._param_map = load_metadata(self._metadata_path)
        rospy.loginfo("px4_param_meta_server: loaded %d parameters from %s", len(self._param_map), self._metadata_path)
        return len(self._param_map)

    def _on_reload(self, _req: Trigger) -> TriggerResponse:
        try:
            count = self.reload()
            return TriggerResponse(success=True, message="reloaded {} parameters".format(count))
        except Exception as e:  # pylint: disable=broad-except
            return TriggerResponse(success=False, message=str(e))

    def _search(self, keyword: str, limit: int) -> List[Dict[str, Any]]:
        kw = keyword.lower().strip()
        if not kw:
            return []
        matched = []
        for p in self._param_map.values():
            name = str(p.get("name", ""))
            short_desc = str(p.get("short_desc", ""))
            long_desc = str(p.get("long_desc", ""))
            group = str(p.get("group", ""))
            if (
                kw in name.lower()
                or kw in short_desc.lower()
                or kw in long_desc.lower()
                or kw in group.lower()
            ):
                matched.append(
                    {
                        "name": name,
                        "kind": p.get("kind"),
                        "type": p.get("type"),
                        "unit": p.get("unit"),
                        "short_desc": short_desc,
                    }
                )
        matched.sort(key=lambda x: x["name"])
        return matched[:limit]

    def _on_query(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        payload: Dict[str, Any] = {"ok": True, "query": raw}
        try:
            if raw.startswith("name:"):
                key = raw.split(":", 1)[1].strip()
                payload["mode"] = "name"
                payload["name"] = key
                payload["meta"] = self._param_map.get(key)
                payload["ok"] = payload["meta"] is not None
            elif raw.startswith("search:"):
                expr = raw.split(":", 1)[1].strip()
                limit = 30
                if "|" in expr:
                    expr, lim = expr.rsplit("|", 1)
                    expr = expr.strip()
                    lim = lim.strip()
                    if lim.isdigit():
                        limit = max(1, min(200, int(lim)))
                payload["mode"] = "search"
                payload["keyword"] = expr
                payload["items"] = self._search(expr, limit)
                payload["count"] = len(payload["items"])
            elif raw == "count":
                payload["mode"] = "count"
                payload["count"] = len(self._param_map)
            else:
                key = raw
                payload["mode"] = "name"
                payload["name"] = key
                payload["meta"] = self._param_map.get(key)
                payload["ok"] = payload["meta"] is not None
        except Exception as e:  # pylint: disable=broad-except
            payload = {"ok": False, "query": raw, "error": str(e)}

        self._pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def run_cli(metadata_path: str, query: str, limit: int) -> int:
    param_map = load_metadata(metadata_path)
    if query == "count":
        print(len(param_map))
        return 0

    if query.startswith("search:"):
        keyword = query.split(":", 1)[1].strip()
        rows = []
        kw = keyword.lower()
        for p in param_map.values():
            if kw in p["name"].lower() or kw in str(p.get("short_desc", "")).lower():
                rows.append(
                    {
                        "name": p["name"],
                        "kind": p["kind"],
                        "type": p["type"],
                        "unit": p["unit"],
                        "short_desc": p["short_desc"],
                    }
                )
        rows.sort(key=lambda x: x["name"])
        print(json.dumps(rows[:limit], ensure_ascii=False, indent=2))
        return 0

    key = query
    if query.startswith("name:"):
        key = query.split(":", 1)[1].strip()
    data = param_map.get(key)
    if data is None:
        print(json.dumps({"ok": False, "name": key}, ensure_ascii=False))
        return 1
    print(json.dumps({"ok": True, "meta": data}, ensure_ascii=False, indent=2))
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="PX4 parameter metadata loader for test/px4_param_tui")
    parser.add_argument("--metadata", required=True, help="path to parameters.json.xz/json/xml")
    parser.add_argument("--query", default="count", help="count | name:PARAM_NAME | search:KEYWORD")
    parser.add_argument("--limit", type=int, default=30, help="search result limit")
    parser.add_argument("--ros", action="store_true", help="run as ROS topic server")
    parser.add_argument("--node-name", default="px4_param_meta_server", help="ROS node name")
    args = parser.parse_args()

    if args.ros:
        rospy.init_node(args.node_name, anonymous=False)
        server = Px4ParamMetaServer(args.metadata)
        rospy.loginfo(
            "px4_param_meta_server ready: query topic=~query, result topic=~result, reload service=~reload, count=%d",
            len(server._param_map),
        )
        rospy.spin()
        return

    rc = run_cli(args.metadata, args.query, max(1, args.limit))
    raise SystemExit(rc)


if __name__ == "__main__":
    main()

