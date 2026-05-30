#!/bin/bash
# setup_dynamixel_udev.sh
# Auto-detects a Dynamixel USB-to-serial adapter by asking you to plug it in,
# then installs a udev rule that:
#   1. Creates the stable symlink  /dev/ttyDynamixel → /dev/ttyUSB*
#   2. Sets the USB latency timer to 1 ms on every plug-in
#      (equivalent to: echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSBx/latency_timer)
#
# The rule matches by USB serial number (preferred) or Vendor:Product ID so
# the symlink follows the device regardless of which USB port is used.
#
# Usage:
#   sudo ./setup_dynamixel_udev.sh            → detect & install
#   sudo ./setup_dynamixel_udev.sh uninstall  → remove installed rule
#   sudo ./setup_dynamixel_udev.sh status     → show rule + live device state

set -euo pipefail

UDEV_RULE="/etc/udev/rules.d/99-dynamixel-tty.rules"
SYMLINK_NAME="ttyDynamixel"
DETECT_TIMEOUT=30

# ── colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
step()  { echo -e "${CYAN}◆${NC} $*"; }
div()   { echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"; }

require_root() {
    [[ $EUID -eq 0 ]] || error "Run with sudo: sudo $0 $*"
}

# ── snapshot_ttys ─────────────────────────────────────────────────────────────
# Returns a sorted list of all current /dev/ttyUSB* and /dev/ttyACM* nodes.
snapshot_ttys() {
    { ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true; } | sort
}

# ── detect_device ─────────────────────────────────────────────────────────────
# Waits for a new tty node to appear, then reads its USB attributes via udevadm.
# Populates globals: DETECTED_TTY, DETECTED_SERIAL, DETECTED_VENDOR,
#                    DETECTED_PRODUCT, DETECTED_MODEL
detect_device() {
    local before
    before=$(snapshot_ttys)

    div
    echo -e "  ${BOLD}Plug in your Dynamixel USB-to-serial adapter now.${NC}"
    echo -e "  Waiting up to ${DETECT_TIMEOUT} seconds for a new /dev/ttyUSB* or /dev/ttyACM* …"
    div
    echo ""

    local elapsed=0
    DETECTED_TTY=""

    while [[ $elapsed -lt $DETECT_TIMEOUT ]]; do
        local after
        after=$(snapshot_ttys)
        # Find the first node that is in 'after' but not in 'before'
        DETECTED_TTY=$(comm -13 <(echo "$before") <(echo "$after") | head -1)
        if [[ -n "$DETECTED_TTY" ]]; then
            echo ""
            break
        fi
        printf "\r  ⏳ %2ds elapsed… " "$elapsed"
        sleep 1
        (( elapsed++ )) || true
    done

    printf "\r                             \r"

    if [[ -z "$DETECTED_TTY" ]]; then
        error "No new serial device appeared within ${DETECT_TIMEOUT}s.
       Make sure the adapter uses a USB-serial driver (cp210x, ftdi_sio, ch341, etc.)."
    fi

    info "New device detected: ${BOLD}${DETECTED_TTY}${NC}"

    # Walk sysfs to find the USB device node for this tty
    local tty_name="${DETECTED_TTY##*/}"           # e.g. ttyUSB1
    local sys_tty_path="/sys/class/tty/${tty_name}"

    if [[ ! -e "$sys_tty_path" ]]; then
        error "sysfs path not found: ${sys_tty_path}"
    fi

    # Resolve the real sysfs path and walk up to the USB device directory
    # Typical layout: .../usb1/1-x/1-x:1.y/ttyUSB1/tty/ttyUSB1
    local resolved
    resolved=$(readlink -f "$sys_tty_path")

    # Climb until we find a directory that looks like a USB device
    # (has idVendor / idProduct files)
    local usb_dev_path=""
    local walk="$resolved"
    for _ in $(seq 1 10); do
        walk=$(dirname "$walk")
        if [[ -f "$walk/idVendor" ]]; then
            usb_dev_path="$walk"
            break
        fi
    done

    if [[ -z "$usb_dev_path" ]]; then
        error "Could not locate USB device node in sysfs for ${DETECTED_TTY}.
       Try: udevadm info --name=${DETECTED_TTY} --attribute-walk"
    fi

    step "sysfs USB path: ${usb_dev_path}"

    local udev_info
    udev_info=$(udevadm info --query=all --path="$usb_dev_path" 2>/dev/null) || \
        error "udevadm could not read path: ${usb_dev_path}"

    DETECTED_SERIAL=$(echo  "$udev_info" | grep "^E: ID_SERIAL_SHORT=" | cut -d= -f2)
    DETECTED_VENDOR=$(echo  "$udev_info" | grep "^E: ID_VENDOR_ID="    | cut -d= -f2)
    DETECTED_PRODUCT=$(echo "$udev_info" | grep "^E: ID_MODEL_ID="     | cut -d= -f2)
    DETECTED_MODEL=$(echo   "$udev_info" | grep "^E: ID_MODEL="        | cut -d= -f2 | tr '_' ' ')

    # Fallback: read directly from sysfs if udevadm returned nothing
    if [[ -z "$DETECTED_VENDOR" ]]; then
        DETECTED_VENDOR=$(cat "${usb_dev_path}/idVendor"  2>/dev/null || true)
        DETECTED_PRODUCT=$(cat "${usb_dev_path}/idProduct" 2>/dev/null || true)
    fi

    if [[ -z "$DETECTED_VENDOR" && -z "$DETECTED_SERIAL" ]]; then
        error "Could not read USB attributes.
       Try manually: udevadm info --name=${DETECTED_TTY} --attribute-walk"
    fi
}

# ── write_udev_rule ───────────────────────────────────────────────────────────
write_udev_rule() {
    local match_attr
    if [[ -n "$DETECTED_SERIAL" ]]; then
        # Prefer serial — uniquely identifies this specific adapter
        match_attr="ENV{ID_SERIAL_SHORT}==\"${DETECTED_SERIAL}\""
    else
        warn "No USB serial number found — matching by Vendor:Product ID."
        warn "All identical adapters (same model) will get this symlink."
        match_attr="ENV{ID_VENDOR_ID}==\"${DETECTED_VENDOR}\", ENV{ID_MODEL_ID}==\"${DETECTED_PRODUCT}\""
    fi

    [[ -f "$UDEV_RULE" ]] && {
        cp "$UDEV_RULE" "${UDEV_RULE}.bak"
        warn "Backed up existing rule → ${UDEV_RULE}.bak"
    }

    cat > "$UDEV_RULE" <<EOF
# Stable symlink + latency-timer fix for Dynamixel USB-to-serial adapter.
# /dev/${SYMLINK_NAME} → whichever /dev/ttyUSB* this adapter is assigned.
# Latency timer is set to 1 ms on every plug-in for reliable servo comms.
# Auto-generated by setup_dynamixel_udev.sh
#
# Model  : ${DETECTED_MODEL:-unknown}
# USB ID : ${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?}
# Serial : ${DETECTED_SERIAL:-(not available)}
SUBSYSTEM=="tty", ${match_attr}, SYMLINK+="${SYMLINK_NAME}", MODE="0666", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"
EOF
    info "udev rule  → ${UDEV_RULE}"
}

# ── apply_latency_timer ───────────────────────────────────────────────────────
# Sets latency_timer = 1 ms on an already-connected device immediately,
# without waiting for a replug. $1 = full device path, e.g. /dev/ttyUSB1
apply_latency_timer() {
    local tty_name="${1##*/}"    # strip /dev/ prefix
    local latency_path="/sys/bus/usb-serial/devices/${tty_name}/latency_timer"

    if [[ -f "$latency_path" ]]; then
        echo 1 > "$latency_path"
        local val
        val=$(cat "$latency_path")
        info "Latency timer set: ${tty_name} → ${val} ms"
    else
        warn "Latency timer sysfs path not found: ${latency_path}"
        warn "The udev RUN rule will apply it on next plug-in."
    fi
}

# ── install ───────────────────────────────────────────────────────────────────
install() {
    require_root

    detect_device   # populates DETECTED_* globals

    echo ""
    div
    printf "  ${BOLD}%-14s${NC} %s\n" "Device:"        "${DETECTED_TTY}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Model:"         "${DETECTED_MODEL:-unknown}"
    printf "  ${BOLD}%-14s${NC} %s\n" "USB ID:"        "${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Serial:"        "${DETECTED_SERIAL:-(none — matching by ID)}"
    printf "  ${BOLD}%-14s${NC} /dev/%s\n" "Symlink:"   "${SYMLINK_NAME}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Latency timer:" "1 ms (set on every plug-in via udev RUN)"
    printf "  ${BOLD}%-14s${NC} %s\n" "Rule file:"     "${UDEV_RULE}"
    div
    echo ""
    read -rp "  Install udev rule? [Y/n]: " confirm
    [[ "${confirm,,}" =~ ^(y|yes|)$ ]] || { warn "Aborted."; exit 0; }
    echo ""

    write_udev_rule

    # Apply latency timer to the already-connected device right now
    apply_latency_timer "$DETECTED_TTY"

    # Reload and trigger so the symlink appears immediately without replug
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=tty --action=add
    sleep 0.5   # give udev a moment to process

    echo ""
    div
    if [[ -L "/dev/${SYMLINK_NAME}" ]]; then
        local real_dev
        real_dev=$(readlink -f "/dev/${SYMLINK_NAME}")
        info "✅ Symlink active now:  /dev/${SYMLINK_NAME} → ${real_dev}"
    else
        warn "Symlink not yet visible (may need replug if udev trigger didn't fire)."
        warn "After next plug-in:  /dev/${SYMLINK_NAME} → ${DETECTED_TTY}"
    fi
    echo -e "   The symlink and latency timer persist across reboots and USB-port changes."
    echo -e "   Check status: ${CYAN}sudo $0 status${NC}"
    div
}

# ── uninstall ─────────────────────────────────────────────────────────────────
uninstall() {
    require_root

    if [[ -f "$UDEV_RULE" ]]; then
        rm "$UDEV_RULE"
        info "Removed ${UDEV_RULE}"
        udevadm control --reload-rules
        udevadm trigger --subsystem-match=tty --action=add
        info "Rules reloaded. Unplug and replug the adapter to clear the symlink."
    else
        warn "Rule not found (already removed?): ${UDEV_RULE}"
    fi
}

# ── status ────────────────────────────────────────────────────────────────────
status() {
    echo ""
    step "Rule file: ${UDEV_RULE}"
    if [[ -f "$UDEV_RULE" ]]; then
        sed 's/^/    /' "$UDEV_RULE"
    else
        echo -e "    ${YELLOW}(not installed)${NC}"
    fi
    echo ""

    step "Symlink: /dev/${SYMLINK_NAME}"
    if [[ -L "/dev/${SYMLINK_NAME}" ]]; then
        local real_dev
        real_dev=$(readlink -f "/dev/${SYMLINK_NAME}")
        echo -e "    ${GREEN}present${NC} → ${real_dev}"

        # Show latency timer for the resolved device
        local tty_name="${real_dev##*/}"
        local latency_path="/sys/bus/usb-serial/devices/${tty_name}/latency_timer"
        if [[ -f "$latency_path" ]]; then
            local val
            val=$(cat "$latency_path")
            if [[ "$val" == "1" ]]; then
                echo -e "    ${GREEN}latency timer${NC}: ${val} ms ✓"
            else
                echo -e "    ${YELLOW}latency timer${NC}: ${val} ms  (expected 1 ms — replug to apply rule)"
            fi
        fi
    else
        echo -e "    ${YELLOW}(not present — device may be unplugged)${NC}"
    fi
    echo ""

    step "Active USB-serial devices (/dev/ttyUSB* and /dev/ttyACM*):"
    local ttys
    ttys=$(snapshot_ttys)
    if [[ -n "$ttys" ]]; then
        while IFS= read -r tty; do
            local info_line
            info_line=$(udevadm info --name="$tty" 2>/dev/null \
                | grep -E "^E: ID_(VENDOR|MODEL|SERIAL)" \
                | awk -F= '{printf "%s=%s  ", $1, $2}' || true)
            printf "    %-16s %s\n" "$tty" "$info_line"
        done <<< "$ttys"
    else
        echo -e "    ${YELLOW}None found.${NC}"
    fi
    echo ""
}

# ── entry point ───────────────────────────────────────────────────────────────
case "${1:-install}" in
    install)   install ;;
    uninstall) uninstall ;;
    status)    status ;;
    *)
        echo -e "Usage: ${BOLD}sudo $0 {install|uninstall|status}${NC}"
        echo ""
        echo "  install    Detect Dynamixel USB adapter interactively, install symlink rule"
        echo "  uninstall  Remove the installed udev rule"
        echo "  status     Show rule file, symlink state, and active serial devices"
        exit 1
        ;;
esac
