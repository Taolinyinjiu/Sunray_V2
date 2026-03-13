#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

import yaml


def parse_args():
    parser = argparse.ArgumentParser(
        description="Load control YAML into /<uav_name><uav_id> namespace."
    )
    parser.add_argument(
        "--input-yaml",
        required=True,
        help="Path to control config YAML.",
    )
    parser.add_argument(
        "--uav-name",
        default="__AUTO__",
        help="Override uav_name. Use __AUTO__ to read from YAML.",
    )
    parser.add_argument(
        "--uav-id",
        default="__AUTO__",
        help="Override uav_id. Use __AUTO__ to read from YAML.",
    )
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    input_yaml = os.path.abspath(args.input_yaml)
    if not os.path.exists(input_yaml):
        print("[load_param] input YAML not found: {}".format(input_yaml), file=sys.stderr)
        return 1

    with open(input_yaml, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        data = {}
    if not isinstance(data, dict):
        print("[load_param] YAML root must be a mapping/dict", file=sys.stderr)
        return 1

    yaml_uav_name = data.get("uav_name", "uav")
    yaml_uav_id = data.get("uav_id", 1)

    uav_name = yaml_uav_name if args.uav_name == "__AUTO__" else args.uav_name
    uav_id_raw = yaml_uav_id if args.uav_id == "__AUTO__" else args.uav_id
    try:
        uav_id = int(uav_id_raw)
    except ValueError:
        print("[load_param] invalid uav_id: {}".format(uav_id_raw), file=sys.stderr)
        return 1

    uav_ns = "{}{}".format(uav_name, uav_id)
    target_ns = "/{}".format(uav_ns)

    print("[load_param] input YAML: {}".format(input_yaml))
    print("[load_param] target namespace: {}".format(target_ns))

    try:
        subprocess.check_call(["rosparam", "set", "/uav_name", uav_name])
        subprocess.check_call(["rosparam", "set", "/uav_id", str(uav_id)])
        subprocess.check_call(["rosparam", "set", "/uav_ns", uav_ns])
        subprocess.check_call(["rosparam", "load", input_yaml, target_ns])
    except subprocess.CalledProcessError as e:
        print("[load_param] rosparam command failed: {}".format(e), file=sys.stderr)
        return e.returncode

    print("[load_param] rosparam load succeeded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
