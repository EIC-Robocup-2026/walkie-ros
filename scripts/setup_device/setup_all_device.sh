#!/bin/bash
# setup_all_device.sh
# Interactive setup for all robot USB devices.
# Calls setup_device.sh for each selected device one at a time.
#
# Usage:
#   sudo ./setup_all_device.sh           → numbered menu, pick which to set up
#   sudo ./setup_all_device.sh all       → set up all devices sequentially

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_DEVICE="${SCRIPT_DIR}/setup_device.sh"

[[ -x "$SETUP_DEVICE" ]] || {
    echo "setup_device.sh not found at: $SETUP_DEVICE" >&2
    exit 1
}

# ── colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
div()   { echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"; }

require_root() { [[ $EUID -eq 0 ]] || error "Run with sudo: sudo $0 $*"; }

# ── device registry ───────────────────────────────────────────────────────────
# Parallel arrays: one entry per device. Index must stay in sync across all.
DEVICE_KEYS=(
    dynamixel
    cubemars
    openarm
    left_wrist_cam
    right_wrist_cam
)
DEVICE_DESCS=(
    "Dynamixel USB-to-serial       →  /dev/ttyDynamixel"
    "Cubemars USB-to-CAN           →  can2"
    "OpenArm USB CAN FD (1 USB)    →  can0, can1"
    "Left wrist camera             →  /dev/left_wrist_cam"
    "Right wrist camera            →  /dev/right_wrist_cam"
)
DEVICE_TYPES=(
    serial
    can
    canfd
    camera
    camera
)
DEVICE_ALIASES=(
    ttyDynamixel
    can2
    can0
    left_wrist_cam
    right_wrist_cam
)
# Extra flags passed verbatim to setup_device.sh (intentionally unquoted on use)
DEVICE_EXTRAS=(
    "--latency 1"
    "--bitrate 1000000"
    "--bitrate 1000000 --dbitrate 5000000"
    ""
    ""
)

# ── menu ───────────────────────────────────────────────────────────────────────
show_menu() {
    echo ""
    div
    echo -e "  ${BOLD}Walkie Robot — USB Device Setup${NC}"
    div
    echo ""
    for i in "${!DEVICE_KEYS[@]}"; do
        printf "  ${CYAN}%d)${NC}  %s\n" "$((i + 1))" "${DEVICE_DESCS[$i]}"
    done
    echo ""
}

# ── run one device ─────────────────────────────────────────────────────────────
run_setup() {
    local idx="$1"
    local type="${DEVICE_TYPES[$idx]}"
    local alias="${DEVICE_ALIASES[$idx]}"
    local extras="${DEVICE_EXTRAS[$idx]}"

    echo ""
    div
    echo -e "  ${BOLD}Setting up: ${DEVICE_DESCS[$idx]}${NC}"
    div

    # shellcheck disable=SC2086  # extras is intentionally word-split
    "$SETUP_DEVICE" install --type "$type" --alias "$alias" $extras
}

# ── main ──────────────────────────────────────────────────────────────────────
main() {
    require_root

    local mode="${1:-menu}"
    local -a selected=()

    if [[ "$mode" == "all" ]]; then
        for i in "${!DEVICE_KEYS[@]}"; do selected+=("$i"); done
    else
        show_menu
        echo -e "  Enter numbers to set up (e.g. ${CYAN}1 3 5${NC}), or ${CYAN}all${NC}:"
        read -rp "  > " input

        if [[ "${input,,}" == "all" ]]; then
            for i in "${!DEVICE_KEYS[@]}"; do selected+=("$i"); done
        else
            for token in $input; do
                if [[ "$token" =~ ^[0-9]+$ ]] \
                    && (( token >= 1 && token <= ${#DEVICE_KEYS[@]} )); then
                    selected+=("$(( token - 1 ))")
                else
                    warn "Ignoring invalid selection: ${token}"
                fi
            done
        fi
    fi

    [[ ${#selected[@]} -gt 0 ]] || { warn "No devices selected."; exit 0; }

    echo ""
    info "Will set up ${#selected[@]} device(s):"
    for idx in "${selected[@]}"; do
        echo -e "   • ${DEVICE_DESCS[$idx]}"
    done
    echo ""
    read -rp "  Proceed? [Y/n]: " confirm
    [[ "${confirm,,}" =~ ^(y|yes|)$ ]] || { warn "Aborted."; exit 0; }

    local ok=0 fail=0
    for idx in "${selected[@]}"; do
        if run_setup "$idx"; then
            (( ok++ )) || true
        else
            warn "Setup failed or was aborted for: ${DEVICE_DESCS[$idx]}"
            (( fail++ )) || true
        fi
    done

    echo ""
    div
    info "Finished — ${ok} succeeded, ${fail} failed."
    div
}

main "$@"
