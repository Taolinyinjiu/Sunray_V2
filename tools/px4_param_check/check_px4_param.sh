#!/usr/bin/env bash

set -o pipefail

START_TIME="$(date +%s.%N)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/config"
CONFIG_FILE="px4_params_default.yaml"
FLOAT_TOLERANCE="${FLOAT_TOLERANCE:-0.001}"
SERVICE_TIMEOUT_SEC="${SERVICE_TIMEOUT_SEC:-3}"
MAVROS_START_TIMEOUT_SEC="${MAVROS_START_TIMEOUT_SEC:-15}"
MAVROS_CONNECT_TIMEOUT_SEC="${MAVROS_CONNECT_TIMEOUT_SEC:-20}"
AUTO_START_MAVROS="${AUTO_START_MAVROS:-1}"
WAIT_MAVROS_CONNECTED="${WAIT_MAVROS_CONNECTED:-1}"
MAVROS_FCU_URL="${MAVROS_FCU_URL:-}"
MAVROS_LAUNCH_PID=""
MAVROS_LOG_FILE=""
ROS_HOME="${ROS_HOME:-/tmp/ros_home_sunray_px4_param_check}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs_sunray_px4_param_check}"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-}"
WORKSPACE_SETUP_FILE="${WORKSPACE_SETUP_FILE:-}"
AGENT_NAME="${AGENT_NAME:-}"
AGENT_ID="${AGENT_ID:-}"
MAVROS_NODE="${MAVROS_NODE:-}"
MAVROS_NS="${MAVROS_NS:-}"
MAVROS_NS_EXPLICIT=0
GROUP_FILTER=""
COLOR_RED=$'\033[31m'
COLOR_GREEN=$'\033[32m'
COLOR_YELLOW=$'\033[33m'
COLOR_BLUE=$'\033[34m'
COLOR_BOLD=$'\033[1m'
COLOR_RESET=$'\033[0m'

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --config FILE        PX4 parameter config YAML file name or path. Default: ${CONFIG_FILE}
  --yaml FILE          Deprecated alias of --config.
  --tolerance VALUE    Float comparison tolerance. Default: ${FLOAT_TOLERANCE}
  --timeout SEC        Timeout for each rosservice call. Default: ${SERVICE_TIMEOUT_SEC}
  --agent-name NAME    Agent name used by mavros.launch. Default: config/env
  --agent-id ID        Agent id used by mavros.launch. Default: config/env
  --mavros-node NAME   MAVROS node name. Default: config/env
  --mavros-ns NS       Full MAVROS namespace. Overrides agent config.
  --no-start-mavros    Do not auto-start MAVROS when the parameter service is missing.
  --fcu-url URL        FCU URL passed to sunray_mavros/mavros.launch when auto-starting.
  --ros-setup FILE     ROS setup.bash to source. Default: /opt/ros/\${ROS_DISTRO:-noetic}/setup.bash
  --workspace-setup FILE
                       Workspace setup.bash to source. Default: <workspace>/devel/setup.bash
  --mavros-start-timeout SEC
                       Timeout for waiting MAVROS after auto-start. Default: ${MAVROS_START_TIMEOUT_SEC}
  --mavros-connect-timeout SEC
                       Timeout for waiting MAVROS connected=true. Default: ${MAVROS_CONNECT_TIMEOUT_SEC}
  --no-wait-connected Do not wait for MAVROS state connected=true.
  --group GROUP       Only check one parameter group name from config.
  -h, --help          Show this help.

Example:
  $0
  $0 --agent-id 2 --group Commander
  $0 --mavros-ns /uav1/mavros --group "Multicopter Position Control"
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --yaml)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --agent-name)
      AGENT_NAME="$2"
      shift 2
      ;;
    --agent-id)
      AGENT_ID="$2"
      shift 2
      ;;
    --mavros-node)
      MAVROS_NODE="$2"
      shift 2
      ;;
    --mavros-ns)
      MAVROS_NS="$2"
      MAVROS_NS_EXPLICIT=1
      shift 2
      ;;
    --no-start-mavros)
      AUTO_START_MAVROS=0
      shift
      ;;
    --fcu-url)
      MAVROS_FCU_URL="$2"
      shift 2
      ;;
    --ros-setup)
      ROS_SETUP_FILE="$2"
      shift 2
      ;;
    --workspace-setup)
      WORKSPACE_SETUP_FILE="$2"
      shift 2
      ;;
    --mavros-start-timeout)
      MAVROS_START_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --mavros-connect-timeout)
      MAVROS_CONNECT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --no-wait-connected)
      WAIT_MAVROS_CONNECTED=0
      shift
      ;;
    --group)
      GROUP_FILTER="$2"
      shift 2
      ;;
    --tolerance)
      FLOAT_TOLERANCE="$2"
      shift 2
      ;;
    --timeout)
      SERVICE_TIMEOUT_SEC="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

yaml_scalar() {
  local key="$1"
  local file="$2"
  awk -v key="${key}" '
    function trim(s) {
      gsub(/^[ \t]+|[ \t]+$/, "", s)
      gsub(/^"|"$/, "", s)
      gsub(/^'\''|'\''$/, "", s)
      return s
    }
    $0 ~ "^[ \t]*" key "[ \t]*:" {
      sub(/^[^:]+:[ \t]*/, "", $0)
      print trim($0)
      exit
    }
  ' "${file}"
}

normalize_ns() {
  local ns="$1"
  ns="/${ns#/}"
  ns="${ns%/}"
  echo "${ns}"
}

elapsed_seconds() {
  awk -v start="${START_TIME}" -v end="$(date +%s.%N)" 'BEGIN { printf "%.3f", end - start }'
}

print_title() {
  printf "%s%s%s\n" "${COLOR_BOLD}" "$1" "${COLOR_RESET}"
}

print_field() {
  printf "%s%s:%s %s\n" "${COLOR_BLUE}" "$1" "${COLOR_RESET}" "$2"
}

resolve_config_path() {
  local file="$1"

  if [[ "${file}" = /* || "${file}" == */* ]]; then
    echo "${file}"
  else
    echo "${CONFIG_DIR}/${file}"
  fi
}

source_if_exists() {
  local setup_file="$1"
  local label="$2"

  if [[ -f "${setup_file}" ]]; then
    # shellcheck source=/dev/null
    source "${setup_file}"
    echo "Sourced ${label}: ${setup_file}"
    return 0
  fi

  return 1
}

source_ros_environment() {
  local ros_distro="${ROS_DISTRO:-noetic}"
  local default_ros_setup="/opt/ros/${ros_distro}/setup.bash"
  local default_workspace_setup="${WORKSPACE_DIR}/devel/setup.bash"
  local fallback_workspace_setup="${WORKSPACE_DIR}/install/setup.bash"

  if [[ -n "${ROS_SETUP_FILE}" ]]; then
    if ! source_if_exists "${ROS_SETUP_FILE}" "ROS"; then
      echo "ROS setup file not found: ${ROS_SETUP_FILE}" >&2
      return 1
    fi
  else
    source_if_exists "${default_ros_setup}" "ROS" >/dev/null || true
  fi

  if [[ -n "${WORKSPACE_SETUP_FILE}" ]]; then
    if ! source_if_exists "${WORKSPACE_SETUP_FILE}" "workspace"; then
      echo "Workspace setup file not found: ${WORKSPACE_SETUP_FILE}" >&2
      return 1
    fi
  elif source_if_exists "${default_workspace_setup}" "workspace" >/dev/null; then
    return 0
  else
    source_if_exists "${fallback_workspace_setup}" "workspace" >/dev/null || true
  fi
}

prepare_ros_runtime_dirs() {
  mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" 2>/dev/null || true
  export ROS_HOME ROS_LOG_DIR
}

print_mavros_log_tail() {
  if [[ -z "${MAVROS_LOG_FILE}" ]]; then
    return 0
  fi

  echo "MAVROS log: ${MAVROS_LOG_FILE}" >&2
  if [[ -f "${MAVROS_LOG_FILE}" ]]; then
    echo "Last MAVROS log lines:" >&2
    tail -n 30 "${MAVROS_LOG_FILE}" >&2
  fi
}

load_target_config() {
  if [[ -f "${CONFIG_FILE}" ]]; then
    [[ -z "${AGENT_NAME}" ]] && AGENT_NAME="$(yaml_scalar agent_name "${CONFIG_FILE}")"
    [[ -z "${AGENT_ID}" ]] && AGENT_ID="$(yaml_scalar agent_id "${CONFIG_FILE}")"
    [[ -z "${MAVROS_NODE}" ]] && MAVROS_NODE="$(yaml_scalar mavros_node "${CONFIG_FILE}")"
  fi

  AGENT_NAME="${AGENT_NAME:-uav}"
  AGENT_ID="${AGENT_ID:-1}"
  MAVROS_NODE="${MAVROS_NODE:-mavros}"

  if [[ -z "${MAVROS_NS}" ]]; then
    MAVROS_NS="/${AGENT_NAME}${AGENT_ID}/${MAVROS_NODE}"
  fi

  MAVROS_NS="$(normalize_ns "${MAVROS_NS}")"
}

service_available() {
  local service="${MAVROS_NS}/param/get"

  timeout "${SERVICE_TIMEOUT_SEC}" rosservice info "${service}" >/dev/null 2>&1
}

state_topic_available() {
  timeout "${SERVICE_TIMEOUT_SEC}" rostopic info "${MAVROS_NS}/state" >/dev/null 2>&1
}

mavros_connected() {
  local output

  output="$(timeout "${SERVICE_TIMEOUT_SEC}" rostopic echo -n 1 "${MAVROS_NS}/state" 2>/dev/null)" || return 1
  grep -q "connected: True" <<<"${output}"
}

start_mavros_if_needed() {
  local service="${MAVROS_NS}/param/get"

  if service_available; then
    return 0
  fi

  if [[ "${AUTO_START_MAVROS}" != "1" ]]; then
    return 1
  fi

  if [[ "${MAVROS_NS_EXPLICIT}" -eq 1 ]]; then
    echo "MAVROS parameter service not available: ${service}" >&2
    echo "Skip auto-start because --mavros-ns was explicitly specified." >&2
    return 1
  fi

  if ! command -v roslaunch >/dev/null 2>&1; then
    echo "roslaunch not found. Cannot auto-start MAVROS." >&2
    return 1
  fi

  MAVROS_LOG_FILE="/tmp/sunray_mavros_${AGENT_NAME}${AGENT_ID}_$(date +%Y%m%d_%H%M%S).log"
  echo "MAVROS parameter service not available: ${service}"
  echo "Auto-start MAVROS: roslaunch sunray_mavros mavros.launch agent_name:=${AGENT_NAME} agent_id:=${AGENT_ID}${MAVROS_FCU_URL:+ fcu_url:=${MAVROS_FCU_URL}}"
  echo "MAVROS log: ${MAVROS_LOG_FILE}"

  prepare_ros_runtime_dirs

  if [[ -n "${MAVROS_FCU_URL}" ]]; then
    nohup roslaunch sunray_mavros mavros.launch \
      agent_name:="${AGENT_NAME}" \
      agent_id:="${AGENT_ID}" \
      fcu_url:="${MAVROS_FCU_URL}" \
      >"${MAVROS_LOG_FILE}" 2>&1 &
  else
    nohup roslaunch sunray_mavros mavros.launch \
      agent_name:="${AGENT_NAME}" \
      agent_id:="${AGENT_ID}" \
      >"${MAVROS_LOG_FILE}" 2>&1 &
  fi
  MAVROS_LAUNCH_PID=$!

  printf "%sWaiting for MAVROS parameter service%s" "${COLOR_BLUE}" "${COLOR_RESET}"
  local deadline=$((SECONDS + MAVROS_START_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if service_available; then
      echo
      return 0
    fi
    if [[ -n "${MAVROS_LAUNCH_PID}" ]] && ! kill -0 "${MAVROS_LAUNCH_PID}" 2>/dev/null; then
      echo
      echo "MAVROS launch process exited before the parameter service became available." >&2
      print_mavros_log_tail
      return 1
    fi
    echo -n "."
    sleep 1
  done
  echo
  echo "Timed out waiting for MAVROS parameter service: ${service}" >&2
  print_mavros_log_tail
  return 1
}

wait_mavros_connected() {
  local deadline

  if [[ "${WAIT_MAVROS_CONNECTED}" != "1" ]]; then
    return 0
  fi

  printf "%sWaiting for MAVROS FCU connection%s (%s connected=true)" "${COLOR_BLUE}" "${COLOR_RESET}" "${MAVROS_NS}/state"
  deadline=$((SECONDS + MAVROS_CONNECT_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if state_topic_available && mavros_connected; then
      echo
      return 0
    fi
    echo -n "."
    sleep 1
  done
  echo

  echo "MAVROS started but FCU is not connected: ${MAVROS_NS}/state" >&2
  echo "Please check fcu_url=${MAVROS_FCU_URL:-/dev/ttyACM0:921600(default)}, serial device, permissions, baudrate, and PX4 power." >&2
  echo "If this is fake/simulated MAVROS without connected=true, use --no-wait-connected." >&2
  print_mavros_log_tail
  return 1
}

require_mavros_param_service() {
  local service="${MAVROS_NS}/param/get"

  start_mavros_if_needed || {
    echo "MAVROS parameter service not available: ${service}" >&2
    echo "Please check whether mavros.launch is running with agent_name=${AGENT_NAME}, agent_id=${AGENT_ID}." >&2
    echo "Expected namespace: ${MAVROS_NS}" >&2
    echo "You can override it with: --mavros-ns /uav1/mavros" >&2
    echo "Or disable auto-start with: --no-start-mavros" >&2
    return 1
  }

  wait_mavros_connected
}

CONFIG_FILE="$(resolve_config_path "${CONFIG_FILE}")"

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Config file not found: ${CONFIG_FILE}" >&2
  exit 1
fi

source_ros_environment || exit 1

if ! command -v rosservice >/dev/null 2>&1 || ! command -v rostopic >/dev/null 2>&1; then
  echo "rosservice/rostopic not found after sourcing ROS/workspace setup files." >&2
  echo "Use --ros-setup FILE or --workspace-setup FILE to specify setup.bash explicitly." >&2
  exit 1
fi

load_target_config
require_mavros_param_service || exit 1

param_rows() {
  awk '
    function trim(s) {
      gsub(/^[ \t]+|[ \t]+$/, "", s)
      gsub(/^"|"$/, "", s)
      return s
    }
    function indent(line) {
      match(line, /^[ ]*/)
      return RLENGTH
    }
    function field_value(line) {
      sub(/^[^:]+:[ \t]*/, "", line)
      return trim(line)
    }
    function emit() {
      if (param_name != "") {
        print param_name "|" type "|" recommended_value "|" group_name "|" desc "|" group_desc
      }
    }
    /^  - name:/ {
      emit()
      group_name=field_value($0)
      group_desc=""
      param_name=""
      type=""
      recommended_value=""
      desc=""
      next
    }
    /^      - name:/ {
      emit()
      param_name=field_value($0)
      type=""
      recommended_value=""
      desc=""
      next
    }
    /^[ \t]*type:/ { if (param_name != "") type=field_value($0); next }
    /^[ \t]*recommended_value:/ { if (param_name != "") recommended_value=field_value($0); next }
    /^[ \t]*description:/ {
      if (param_name != "") {
        desc=field_value($0)
      } else if (group_name != "") {
        group_desc=field_value($0)
      }
      next
    }
    END { emit() }
  ' "${CONFIG_FILE}"
}

read_current_value() {
  local name="$1"
  local type="$2"
  local output status integer real

  output="$(timeout "${SERVICE_TIMEOUT_SEC}" rosservice call "${MAVROS_NS}/param/get" "{param_id: '${name}'}" 2>&1)"
  status=$?
  if [[ ${status} -ne 0 ]]; then
    if [[ ${status} -eq 124 ]]; then
      echo "TIMEOUT"
      return 1
    fi
    echo "CALL_FAILED"
    return 1
  fi

  if ! grep -q "success: True" <<<"${output}"; then
    echo "GET_FAILED"
    return 1
  fi

  integer="$(awk '/integer:/ {print $2; exit}' <<<"${output}")"
  real="$(awk '/real:/ {print $2; exit}' <<<"${output}")"

  if [[ "${type}" == "integer" ]]; then
    echo "${integer}"
  else
    echo "${real}"
  fi
}

format_value() {
  local value="$1"
  local type="$2"

  if [[ "${type}" == "integer" ]]; then
    echo "${value}"
    return
  fi

  awk -v value="${value}" '
    function abs(x) { return x < 0 ? -x : x }
    BEGIN {
      if (value !~ /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/) {
        print value
        exit
      }
      number = value + 0
      if (number != 0 && (abs(number) < 0.001 || abs(number) >= 10000)) {
        printf "%.3e\n", number
      } else {
        printf "%.3f\n", number
      }
    }
  '
}

values_equal() {
  local current="$1"
  local recommended="$2"
  local type="$3"

  if [[ "${type}" == "integer" ]]; then
    [[ "${current}" == "${recommended}" ]]
    return
  fi

  awk -v current="${current}" -v recommended="${recommended}" -v tolerance="${FLOAT_TOLERANCE}" '
    function abs(x) { return x < 0 ? -x : x }
    BEGIN {
      if (current !~ /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/ ||
          recommended !~ /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/) {
        exit 1
      }
      exit(abs((current + 0) - (recommended + 0)) <= (tolerance + 0) ? 0 : 1)
    }
  '
}

status_text() {
  local status="$1"

  case "${status}" in
    OK)
      printf "%s%-5s%s" "${COLOR_GREEN}" "OK" "${COLOR_RESET}"
      ;;
    diff)
      printf "%s%-5s%s" "${COLOR_YELLOW}" "${status}" "${COLOR_RESET}"
      ;;
    failed)
      printf "%s%-5s%s" "${COLOR_RED}" "${status}" "${COLOR_RESET}"
      ;;
    *)
      printf "%-5s" "${status}"
      ;;
  esac
}

print_title "PX4 parameter check"
print_field "Config" "${CONFIG_FILE}"
print_field "Agent" "${AGENT_NAME}${AGENT_ID}"
print_field "MAVROS namespace" "${MAVROS_NS}"
[[ -n "${GROUP_FILTER}" ]] && print_field "Group" "${GROUP_FILTER}"
print_field "Float tolerance" "${FLOAT_TOLERANCE}"
print_field "Service timeout" "${SERVICE_TIMEOUT_SEC}s"
echo

total=0
failed=0
ok_count=0
diff_count=0
failed_count=0
current_group=""
while IFS='|' read -r name type recommended group desc group_desc; do
  [[ -z "${name}" ]] && continue
  [[ -n "${GROUP_FILTER}" && "${group}" != "${GROUP_FILTER}" ]] && continue

  if [[ "${group}" != "${current_group}" ]]; then
    current_group="${group}"
    echo
    if [[ -n "${group_desc}" ]]; then
      echo "[${current_group}] - ${group_desc}"
    else
      echo "[${current_group}]"
    fi
    printf "%s\n" "参数                当前值     推荐值     状态   描述"
    printf "%-18s  %-9s  %-9s  %-5s  %s\n" "------------------" "---------" "---------" "-----" "-----------"
  fi

  total=$((total + 1))
  current="$(read_current_value "${name}" "${type}")"
  if [[ $? -ne 0 ]]; then
    failed=$((failed + 1))
    failed_count=$((failed_count + 1))
    printf "%-18s  %-9s  %-9s  %s  %s\n" "${name}" "${current}" "$(format_value "${recommended}" "${type}")" "$(status_text failed)" "${desc}"
  elif values_equal "${current}" "${recommended}" "${type}"; then
    ok_count=$((ok_count + 1))
    printf "%-18s  %-9s  %-9s  %s  %s\n" "${name}" "$(format_value "${current}" "${type}")" "$(format_value "${recommended}" "${type}")" "$(status_text OK)" "${desc}"
  else
    diff_count=$((diff_count + 1))
    printf "%-18s  %-9s  %-9s  %s  %s\n" "${name}" "$(format_value "${current}" "${type}")" "$(format_value "${recommended}" "${type}")" "$(status_text diff)" "${desc}"
  fi
done < <(param_rows)

echo
echo "Summary: total=${total}, OK=${ok_count}, diff=${diff_count}, failed=${failed_count}, elapsed=$(elapsed_seconds)s."
[[ "${failed}" -eq 0 ]]
