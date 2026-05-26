#!/usr/bin/env bash

set -o pipefail

START_TIME="$(date +%s.%N)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/config"
CONFIG_FILE="px4_params_default.yaml"
FLOAT_TOLERANCE="${FLOAT_TOLERANCE:-0.001}"
SERVICE_TIMEOUT_SEC="${SERVICE_TIMEOUT_SEC:-3}"
AGENT_NAME="${AGENT_NAME:-}"
AGENT_ID="${AGENT_ID:-}"
MAVROS_NODE="${MAVROS_NODE:-}"
MAVROS_NS="${MAVROS_NS:-}"
GROUP_FILTER=""
DRY_RUN=0
REBOOT=0
COLOR_RED=$'\033[31m'
COLOR_GREEN=$'\033[32m'
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
  --group GROUP       Only set one parameter group name from config.
  --dry-run           Print commands without changing PX4 parameters.
  --reboot            Reboot flight controller after setting parameters.
  -h, --help          Show this help.

Example:
  $0 --group Commander
  $0 --agent-id 2 --dry-run
  $0 --mavros-ns /uav1/mavros --dry-run
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
      shift 2
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
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --reboot)
      REBOOT=1
      shift
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

resolve_config_path() {
  local file="$1"

  if [[ "${file}" = /* || "${file}" == */* ]]; then
    echo "${file}"
  else
    echo "${CONFIG_DIR}/${file}"
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

require_mavros_param_service() {
  local service="${MAVROS_NS}/param/get"

  if timeout "${SERVICE_TIMEOUT_SEC}" rosservice info "${service}" >/dev/null 2>&1; then
    return 0
  fi

  echo "MAVROS parameter service not available: ${service}" >&2
  echo "Please check whether mavros.launch is running with agent_name=${AGENT_NAME}, agent_id=${AGENT_ID}." >&2
  echo "Expected namespace: ${MAVROS_NS}" >&2
  echo "You can override it with: --mavros-ns /uav1/mavros" >&2
  return 1
}

CONFIG_FILE="$(resolve_config_path "${CONFIG_FILE}")"

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Config file not found: ${CONFIG_FILE}" >&2
  exit 1
fi

if ! command -v rosservice >/dev/null 2>&1; then
  echo "rosservice not found. Please source ROS and MAVROS workspace first." >&2
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
  local expected="$2"
  local type="$3"

  if [[ "${type}" == "integer" ]]; then
    [[ "${current}" == "${expected}" ]]
    return
  fi

  awk -v current="${current}" -v expected="${expected}" -v tolerance="${FLOAT_TOLERANCE}" '
    function abs(x) { return x < 0 ? -x : x }
    BEGIN {
      if (current !~ /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/ ||
          expected !~ /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/) {
        exit 1
      }
      exit(abs((current + 0) - (expected + 0)) <= (tolerance + 0) ? 0 : 1)
    }
  '
}

status_text() {
  local status="$1"

  case "${status}" in
    不需要修改*|修改成功*)
      printf "%s%s%s" "${COLOR_GREEN}" "${status}" "${COLOR_RESET}"
      ;;
    读取失败*|修改失败*)
      printf "%s%s%s" "${COLOR_RED}" "${status}" "${COLOR_RESET}"
      ;;
    *)
      printf "%s" "${status}"
      ;;
  esac
}

display_width() {
  local text="$1"

  TEXT="${text}" python3 - <<'PY'
import os
import unicodedata

text = os.environ.get("TEXT", "")
width = 0
for char in text:
    if unicodedata.east_asian_width(char) in ("F", "W"):
        width += 2
    else:
        width += 1
print(width)
PY
}

pad_display() {
  local text="$1"
  local target_width="$2"
  local width padding

  width="$(display_width "${text}")"
  padding=$((target_width - width))
  if (( padding < 0 )); then
    padding=0
  fi

  printf "%s%*s" "${text}" "${padding}" ""
}

print_set_row() {
  local name="$1"
  local status="$2"
  local desc="$3"

  printf "%-18s  %s  %s\n" "${name}" "$(status_text "$(pad_display "${status}" 20)")" "${desc}"
}

set_param() {
  local name="$1"
  local type="$2"
  local value="$3"
  local integer_value=0
  local real_value=0.0
  local request

  if [[ "${type}" == "integer" ]]; then
    integer_value="${value}"
  else
    real_value="${value}"
  fi

  request="{param_id: '${name}', value: {integer: ${integer_value}, real: ${real_value}}}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    return 0
  fi

  timeout "${SERVICE_TIMEOUT_SEC}" rosservice call "${MAVROS_NS}/param/set" "${request}" >/dev/null
}

reboot_fcu() {
  local request
  request="{command: 246, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "DRY-RUN rosservice call ${MAVROS_NS}/cmd/command \"${request}\""
    return 0
  fi

  rosservice call "${MAVROS_NS}/cmd/command" "${request}" >/dev/null
}

echo "PX4 parameter set"
echo "Config: ${CONFIG_FILE}"
echo "Agent: ${AGENT_NAME}${AGENT_ID}"
echo "MAVROS namespace: ${MAVROS_NS}"
[[ -n "${GROUP_FILTER}" ]] && echo "Group: ${GROUP_FILTER}"
echo "Float tolerance: ${FLOAT_TOLERANCE}"
echo "Service timeout: ${SERVICE_TIMEOUT_SEC}s"
[[ "${DRY_RUN}" -eq 1 ]] && echo "Mode: dry-run"
echo

total=0
failed=0
skipped=0
changed=0
success_count=0
failed_count=0
dry_run_count=0
current_group=""
while IFS='|' read -r name type value group desc group_desc; do
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
    printf "%s\n" "参数                状态                  描述"
    printf "%-18s  %-20s  %s\n" "------------------" "--------------------" "-----------------"
  fi

  total=$((total + 1))

  current="$(read_current_value "${name}" "${type}")"
  if [[ $? -ne 0 ]]; then
    failed=$((failed + 1))
    failed_count=$((failed_count + 1))
    status="读取失败: ${current}"
    print_set_row "${name}" "${status}" "${desc}"
    continue
  fi

  current_display="$(format_value "${current}" "${type}")"
  value_display="$(format_value "${value}" "${type}")"

  if values_equal "${current}" "${value}" "${type}"; then
    skipped=$((skipped + 1))
    status="不需要修改: ${current_display}"
    print_set_row "${name}" "${status}" "${desc}"
    continue
  fi

  changed=$((changed + 1))
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    dry_run_count=$((dry_run_count + 1))
    status="需要修改: ${current_display} -> ${value_display}, dry-run"
    print_set_row "${name}" "${status}" "${desc}"
    continue
  fi

  if set_param "${name}" "${type}" "${value}"; then
    success_count=$((success_count + 1))
    status="修改成功: ${current_display} -> ${value_display}"
    print_set_row "${name}" "${status}" "${desc}"
  else
    failed=$((failed + 1))
    failed_count=$((failed_count + 1))
    status="修改失败: ${current_display} -> ${value_display}"
    print_set_row "${name}" "${status}" "${desc}"
  fi
done < <(param_rows)

if [[ "${REBOOT}" -eq 1 ]]; then
  echo
  printf "Reboot flight controller... "
  if reboot_fcu; then
    echo "ok"
  else
    failed=$((failed + 1))
    echo "failed"
  fi
fi

echo
echo "Summary: total=${total}, skipped=${skipped}, changed=${changed}, success=${success_count}, dry_run=${dry_run_count}, failed=${failed_count}, elapsed=$(elapsed_seconds)s."
[[ "${failed}" -eq 0 ]]
