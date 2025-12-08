#!/usr/bin/env bash
set -euo pipefail

# Directory where test binaries live
TEST_BIN_DIR="./build"
DEFAULT_SLEEP=2

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Extra args to pass to all sub-tests (for Moteus transport/controller)
MOTEUS_ARGS=()

# Format: "Test Name:binary_name arguments"
tests=(
  "Velocity Control:write --position nan --velocity 2.0 --duration-ms 3000"
  "Constant Acceleration Trajectory:write --position 1.0 --velocity 0.0 --accel-limit 2.0 --velocity-limit 0.5 --duration-ms 2000"
  "Torque Control:write --velocity 0.0 --kp-scale 0.0 --kd-scale 0.0 --ilimit-scale 0.0 --feedforward-torque 0.1 --duration-ms 1000"
  "Read:read"
)

usage() {
  cat <<EOF
Usage: $0 [moteus-options...] [pattern]

Run all test binaries from ${TEST_BIN_DIR}.

Moteus options (forwarded to all sub-tests):
  --moteus-id <id>                 Controller ID
  --socketcan-iface <iface>        SocketCAN interface (e.g. can0)
  --socketcan-ignore-errors <n>    Ignore SocketCAN errors flag
  --can-disable-brs <n>            Disable CAN BRS flag

Optional:
  pattern   Only run tests whose name or binary matches this substring.

Examples:
  $0                                           # run all tests
  $0 torque                                   # run only torque-related tests
  $0 --moteus-id 2 --socketcan-iface can1     # all tests on ID=2 via can1
  $0 --moteus-id 3 torque                     # torque tests on ID=3
EOF
}

run_test() {
  local name="$1"
  local raw_cmd="$2"
  
  # Split the raw command string into an array (e.g., "write" "--velocity" "2.0")
  read -r -a cmd_parts <<< "$raw_cmd"
  
  # Extract the binary name (first element) and arguments (the rest)
  local bin_name="${cmd_parts[0]}"
  local bin_path="${TEST_BIN_DIR}/${bin_name}"

  echo
  echo -e "${YELLOW}==== TEST START: ${name} (${raw_cmd}) ====${NC}"

  # Check if the binary file exists
  if [[ ! -x "$bin_path" ]]; then
    echo -e "${RED}MISSING/NOT EXECUTABLE:${NC} ${bin_path}"
    return 1
  fi

  # Run the binary path with its arguments + global MOTEUS_ARGS
  if "$bin_path" "${cmd_parts[@]:1}" "${MOTEUS_ARGS[@]}"; then
    echo -e "${GREEN}==== EXECUTED : ${name} ====${NC}"
  else
    echo -e "${RED}==== FAIL : ${name} ====${NC}"
    return 1
  fi

  sleep "${DEFAULT_SLEEP}"
}

main() {
  local pattern=""
  MOTEUS_ARGS=()

  # Parse CLI args: extract moteus options, plus optional pattern
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)
        usage
        exit 0
        ;;
      --moteus-id|--socketcan-iface|--socketcan-ignore-errors|--can-disable-brs)
        if [[ $# -lt 2 ]]; then
          echo -e "${RED}Missing value for option:${NC} $1"
          exit 1
        fi
        MOTEUS_ARGS+=("$1" "$2")
        shift 2
        ;;
      *)
        # First non-option is treated as pattern (if provided)
        if [[ -z "${pattern}" ]]; then
          pattern="$1"
          shift
        else
          echo -e "${RED}Unexpected extra argument:${NC} $1"
          echo "Only a single pattern is supported."
          exit 1
        fi
        ;;
    esac
  done

  local any_run=0

  for entry in "${tests[@]}"; do
    # Split the entry by the first colon only
    IFS=":" read -r name full_cmd <<< "$entry"

    # Pattern filter (optional)
    if [[ -n "$pattern" ]] && \
       [[ "$name" != *"$pattern"* && "$full_cmd" != *"$pattern"* ]]; then
      continue
    fi

    any_run=1
    run_test "$name" "$full_cmd"
  done

  if [[ "$any_run" -eq 0 ]]; then
    echo -e "${RED}No tests matched pattern:${NC} '${pattern}'"
    exit 1
  fi

  echo
  echo -e "${GREEN}All selected tests completed.${NC}"
}

main "$@"
