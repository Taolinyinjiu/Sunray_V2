#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Export mocap pose topic to timestamped CSV (avoid overwrite).

Usage:
  export_mocap_pose_csv.sh --bag /path/to/data.bag [options]
  export_mocap_pose_csv.sh --live [options]

Options:
  --bag <path>          Input rosbag path (offline export mode)
  --live                Export from live topic stream (press Ctrl+C to stop)
  --topic <name>        Topic to export (default: /vrpn_client_node/uav1/pose)
  --out-dir <dir>       Output directory (default: current directory)
  --prefix <name>       Output file prefix (default: mocap_pose)
  --time-format <fmt>   date format used in file name
                        (default: %Y%m%d_%H%M%S, example with ms: %Y%m%d_%H%M%S_%3N)
  -h, --help            Show this help

Examples:
  ./export_mocap_pose_csv.sh \
    --bag /data/flight_01.bag \
    --topic /vrpn_client_node/uav1/pose \
    --out-dir /home/taolin/Desktop/Sunray_OpticalFlow_TestDir/MTF01/Mocap_Logs

  ./export_mocap_pose_csv.sh --live --topic /vrpn_client_node/uav1/pose --out-dir /tmp
EOF
}

err() {
  echo "[export_mocap_pose_csv] ERROR: $*" >&2
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    err "command not found: $1"
    exit 127
  fi
}

BAG=""
TOPIC="/vrpn_client_node/uav1/pose"
OUT_DIR="$(pwd)"
PREFIX="mocap_pose"
TIME_FORMAT="%Y%m%d_%H%M%S"
LIVE_MODE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || { err "--bag requires a value"; exit 2; }
      BAG="$2"
      shift 2
      ;;
    --live)
      LIVE_MODE=1
      shift
      ;;
    --topic)
      [[ $# -ge 2 ]] || { err "--topic requires a value"; exit 2; }
      TOPIC="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || { err "--out-dir requires a value"; exit 2; }
      OUT_DIR="$2"
      shift 2
      ;;
    --prefix)
      [[ $# -ge 2 ]] || { err "--prefix requires a value"; exit 2; }
      PREFIX="$2"
      shift 2
      ;;
    --time-format)
      [[ $# -ge 2 ]] || { err "--time-format requires a value"; exit 2; }
      TIME_FORMAT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      err "unknown option: $1"
      usage
      exit 2
      ;;
  esac
done

require_cmd rostopic
require_cmd date

if [[ "$LIVE_MODE" -eq 1 && -n "$BAG" ]]; then
  err "cannot use --live and --bag together"
  exit 2
fi

if [[ "$LIVE_MODE" -eq 0 ]]; then
  if [[ -z "$BAG" ]]; then
    err "missing --bag (or use --live for online export)"
    exit 2
  fi
  if [[ ! -f "$BAG" ]]; then
    err "bag file not found: $BAG"
    exit 2
  fi
fi

mkdir -p "$OUT_DIR"
TIMESTAMP="$(date +"$TIME_FORMAT")"
OUT_CSV="$OUT_DIR/${PREFIX}_${TIMESTAMP}.csv"

echo "[export_mocap_pose_csv] topic: $TOPIC"
echo "[export_mocap_pose_csv] output: $OUT_CSV"

if [[ "$LIVE_MODE" -eq 1 ]]; then
  echo "[export_mocap_pose_csv] mode: live stream (Ctrl+C to stop)"
  rostopic echo -p "$TOPIC" > "$OUT_CSV"
else
  if ! rostopic list -b "$BAG" | grep -Fxq "$TOPIC"; then
    err "topic '$TOPIC' not found in bag: $BAG"
    echo "[export_mocap_pose_csv] available topics in bag:"
    rostopic list -b "$BAG"
    exit 3
  fi

  echo "[export_mocap_pose_csv] mode: offline bag export"
  rostopic echo -b "$BAG" -p "$TOPIC" > "$OUT_CSV"
fi

if [[ -s "$OUT_CSV" ]]; then
  echo "[export_mocap_pose_csv] done: $OUT_CSV"
else
  err "export completed but output file is empty: $OUT_CSV"
  exit 4
fi
