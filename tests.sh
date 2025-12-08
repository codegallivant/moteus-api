#!/usr/bin/env bash
set -euo pipefail

# Directory where test binaries live
TEST_BIN_DIR="./build"
DEFAULT_SLEEP=2

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

tests=(
  "Velocity Control:velocity_control"
  "Constant Acceleration Trajectory:constant_acceleration_trajectory"
  "Torque Control:torque_control"
  "Read:read"
)

usage() {
  cat <<EOF
Usage: $0 [pattern]

Run all test binaries from ${TEST_BIN_DIR}.

Optional:
  pattern   Only run tests whose name or binary matches this substring.

Examples:
  $0              # run all tests
  $0 torque       # run only torque-related tests
EOF
}

run_test() {
  local name="$1"
  local bin="$2"
  local path="${TEST_BIN_DIR}/${bin}"

  echo
  echo -e "${YELLOW}==== TEST START: ${name} (${bin}) ====${NC}"

  if [[ ! -x "$path" ]]; then
    echo -e "${RED}MISSING/NOT EXECUTABLE:${NC} ${path}"
    return 1
  fi

  # Run the test
  if "$path"; then
    echo -e "${GREEN}==== TEST PASS : ${name} ====${NC}"
  else
    echo -e "${RED}==== TEST FAIL : ${name} ====${NC}"
    return 1
  fi

  sleep "${DEFAULT_SLEEP}"
}

main() {
  local pattern="${1:-}"

  if [[ "${pattern}" == "-h" || "${pattern}" == "--help" ]]; then
    usage
    exit 0
  fi

  local any_run=0

  for entry in "${tests[@]}"; do
    IFS=":" read -r name bin <<< "$entry"

    # Pattern filter (optional)
    if [[ -n "$pattern" ]] && \
       [[ "$name" != *"$pattern"* && "$bin" != *"$pattern"* ]]; then
      continue
    fi

    any_run=1
    run_test "$name" "$bin"
  done

  if [[ "$any_run" -eq 0 ]]; then
    echo -e "${RED}No tests matched pattern:${NC} '${pattern}'"
    exit 1
  fi

  echo
  echo -e "${GREEN}All selected tests completed.${NC}"
}

main "$@"
