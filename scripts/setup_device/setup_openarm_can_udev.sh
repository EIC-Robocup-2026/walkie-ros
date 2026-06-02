#!/bin/bash
# setup_openarm_can_udev.sh
# Auto-detects the openarm USB CAN adapter (PCAN USB Pro FD) by asking you to
# plug it in, then installs a systemd service that automatically configures
# CAN FD (bitrate 1Mbps / dbitrate 5Mbps) using native ip-link commands
# whenever the adapter is plugged in.
#
# Usage:
#   sudo ./setup_openarm_can_udev.sh            → detect & install
#   sudo ./setup_openarm_can_udev.sh uninstall  → remove service
#   sudo ./setup_openarm_can_udev.sh status     → show service + live interfaces

set -euo pipefail
trap 'echo -e "\033[0;31m[ERROR]\033[0m Script failed at line $LINENO: $BASH_COMMAND" >&2' ERR

SERVICE_FILE="/etc/systemd/system/openarm-can-up.service"
BITRATE=1000000
DBITRATE=5000000
DETECT_TIMEOUT=30
IFACE_SETTLE_SEC=2   # seconds to wait after first interface for multi-channel devices

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

# ── detect ────────────────────────────────────────────────────────────────────
# Prompts the user to plug in the device. Waits for new CAN interface(s) to
# appear, then collects ALL interfaces that share the same USB parent.
# Populates globals: DETECTED_IFACES (array), DETECTED_MODEL, DETECTED_VENDOR,
#                    DETECTED_PRODUCT, DETECTED_USB_PATH
detect_device() {
    local before
    before=$(ls /sys/class/net/ 2>/dev/null | sort)

    local existing_can
    existing_can=$(ls /sys/class/net/ | grep '^can' || true)
    if [[ -n "$existing_can" ]]; then
        warn "CAN interface(s) already present: ${existing_can}"
        warn "Unplug the openarm adapter first, then re-run, OR continue if"
        warn "none of these belong to the openarm device."
        echo ""
        read -rp "  Continue anyway? [y/N]: " cont
        [[ "${cont,,}" =~ ^(y|yes)$ ]] || { warn "Aborted."; exit 0; }
    fi

    div
    echo -e "  ${BOLD}Plug in the openarm USB CAN adapter now.${NC}"
    echo -e "  Waiting up to ${DETECT_TIMEOUT} seconds..."
    div
    echo ""

    local elapsed=0
    local first_iface=""

    while [[ $elapsed -lt $DETECT_TIMEOUT ]]; do
        local after
        after=$(ls /sys/class/net/ 2>/dev/null | sort)
        first_iface=$(comm -13 <(echo "$before") <(echo "$after") | head -1)
        if [[ -n "$first_iface" ]]; then
            echo ""
            break
        fi
        printf "\r  ⏳ %2ds elapsed... " "$elapsed"
        sleep 1
        (( elapsed++ )) || true
    done

    printf "\r                          \r"

    if [[ -z "$first_iface" ]]; then
        error "No new CAN interface appeared within ${DETECT_TIMEOUT}s."
    fi

    info "First interface detected: ${BOLD}${first_iface}${NC}"
    info "Waiting ${IFACE_SETTLE_SEC}s for additional channels from the same device..."
    sleep "$IFACE_SETTLE_SEC"

    # ── resolve USB device sysfs path from first interface ────────────────────
    # Symlink: /sys/class/net/<iface> → .../usb3/<dev>/<dev>:<cfg>/net/<iface>
    # Walk up 3 dirs:  net/<iface> → <cfg>/ → <dev>/
    local first_net_path usb_dev_path
    first_net_path=$(readlink -f "/sys/class/net/${first_iface}" 2>/dev/null) \
        || error "Cannot resolve sysfs path for ${first_iface}"
    usb_dev_path=$(dirname "$(dirname "$(dirname "$first_net_path")")")
    DETECTED_USB_PATH="$usb_dev_path"
    step "USB device path: ${usb_dev_path}"

    # ── collect ALL CAN interfaces that share this USB parent ─────────────────
    # No comm/pipefail needed — just check each current can* against the USB parent.
    DETECTED_IFACES=()
    for iface in $(ls /sys/class/net/ 2>/dev/null | grep -E '^can[0-9]+$' || true); do
        local iface_net_path iface_usb_path
        iface_net_path=$(readlink -f "/sys/class/net/${iface}" 2>/dev/null) || continue
        iface_usb_path=$(dirname "$(dirname "$(dirname "$iface_net_path")")")
        if [[ "$iface_usb_path" == "$usb_dev_path" ]]; then
            DETECTED_IFACES+=("$iface")
            step "Found channel: ${iface}"
        fi
    done

    if [[ ${#DETECTED_IFACES[@]} -eq 0 ]]; then
        warn "No channels matched USB parent — falling back to: ${first_iface}"
        DETECTED_IFACES=("$first_iface")
    fi

    # ── read USB attributes ───────────────────────────────────────────────────
    local udev_info
    udev_info=$(udevadm info --query=all --path="$usb_dev_path" 2>/dev/null) || \
        error "udevadm could not read path: ${usb_dev_path}"

    DETECTED_MODEL=$(echo  "$udev_info" | grep "^E: ID_MODEL="     | cut -d= -f2 | tr '_' ' ')
    DETECTED_VENDOR=$(echo "$udev_info" | grep "^E: ID_VENDOR_ID="  | cut -d= -f2)
    DETECTED_PRODUCT=$(echo "$udev_info" | grep "^E: ID_MODEL_ID="  | cut -d= -f2)
    DETECTED_SERIAL=$(echo "$udev_info" | grep "^E: ID_SERIAL_SHORT=" | cut -d= -f2)
}

# ── write_service ─────────────────────────────────────────────────────────────
write_service() {
    local primary_iface="${DETECTED_IFACES[0]}"

    local after_lines=""
    local binds_lines=""
    for iface in "${DETECTED_IFACES[@]}"; do
        local unit="sys-subsystem-net-devices-${iface}.device"
        after_lines+="After=${unit}"$'\n'
        binds_lines+="BindsTo=${unit}"$'\n'
    done

    local wantedby_lines=""
    for iface in "${DETECTED_IFACES[@]}"; do
        wantedby_lines+="WantedBy=sys-subsystem-net-devices-${iface}.device"$'\n'
    done

    # Build ExecStart lines: bring down → configure FD → bring up for each iface
    local exec_lines=""
    for iface in "${DETECTED_IFACES[@]}"; do
        exec_lines+="-ExecStart=/sbin/ip link set ${iface} down"$'\n'
        exec_lines+="ExecStart=/sbin/ip link set ${iface} type can bitrate ${BITRATE} dbitrate ${DBITRATE} fd on"$'\n'
        exec_lines+="ExecStart=/sbin/ip link set ${iface} up"$'\n'
    done

    local stop_lines=""
    for iface in "${DETECTED_IFACES[@]}"; do
        stop_lines+="-ExecStop=/sbin/ip link set ${iface} down"$'\n'
    done

    [[ -f "$SERVICE_FILE" ]] && { cp "$SERVICE_FILE" "${SERVICE_FILE}.bak"; warn "Backed up → ${SERVICE_FILE}.bak"; }

    cat > "$SERVICE_FILE" <<EOF
# Configures openarm CAN interfaces (FD mode) automatically on plug-in.
# Auto-generated by setup_openarm_can_udev.sh
# Device  : ${DETECTED_MODEL:-unknown}  (${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?})
# Ifaces  : ${DETECTED_IFACES[*]}
[Unit]
Description=Configure openarm CAN interfaces (FD, ${BITRATE}/${DBITRATE})
# Wait until ALL channels from the PCAN adapter are present
${after_lines}${binds_lines}
[Service]
Type=oneshot
RemainAfterExit=yes
${exec_lines}${stop_lines}StandardOutput=journal
StandardError=journal

[Install]
# systemd starts this service when the openarm CAN device unit(s) appear
${wantedby_lines}
EOF
    info "Service    → ${SERVICE_FILE}"
}

# ── install ───────────────────────────────────────────────────────────────────
install() {
    require_root
    detect_device   # populates DETECTED_* globals

    echo ""
    div
    printf "  ${BOLD}%-14s${NC} %s\n" "Model:"    "${DETECTED_MODEL:-unknown}"
    printf "  ${BOLD}%-14s${NC} %s\n" "USB ID:"   "${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Serial:"   "${DETECTED_SERIAL:-(none — model-based match)}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Channels:" "${DETECTED_IFACES[*]}"
    printf "  ${BOLD}%-14s${NC} %s / %s bps (FD)\n" "Bitrate:"  "$BITRATE" "$DBITRATE"
    printf "  ${BOLD}%-14s${NC} %s\n" "Service:"  "$SERVICE_FILE"
    div
    echo ""
    read -rp "  Install service? [Y/n]: " confirm
    [[ "${confirm,,}" =~ ^(y|yes|)$ ]] || { warn "Aborted."; exit 0; }
    echo ""

    write_service

    systemctl daemon-reload
    systemctl enable openarm-can-up.service
    info "Service enabled."

    # Apply native CAN FD config now since interfaces are already live
    info "Configuring CAN FD on current interfaces..."
    for iface in "${DETECTED_IFACES[@]}"; do
        ip link set "$iface" down 2>/dev/null || true
        ip link set "$iface" type can bitrate "$BITRATE" dbitrate "$DBITRATE" fd on \
            && ip link set "$iface" up \
            && info "  ${iface} → up (FD, ${BITRATE}/${DBITRATE})" \
            || warn "  ${iface} → failed (check above)"
    done

    echo ""
    div
    info "✅ Setup complete!"
    echo -e "   Plug in the adapter at any time — openarm CAN will come up automatically."
    echo -e "   Check: ${CYAN}sudo $0 status${NC}"
    div
}

# ── uninstall ─────────────────────────────────────────────────────────────────
uninstall() {
    require_root
    if [[ -f "$SERVICE_FILE" ]]; then
        systemctl disable openarm-can-up.service 2>/dev/null || true
        systemctl stop openarm-can-up.service 2>/dev/null || true
        rm "$SERVICE_FILE"
        systemctl daemon-reload
        info "Removed ${SERVICE_FILE}."
    else
        warn "${SERVICE_FILE} not found — nothing to remove."
    fi
}

# ── status ────────────────────────────────────────────────────────────────────
status() {
    echo ""
    step "Service file: ${SERVICE_FILE}"
    if [[ -f "$SERVICE_FILE" ]]; then
        echo ""
        sed 's/^/    /' "$SERVICE_FILE"
    else
        echo -e "    ${YELLOW}(not installed)${NC}"
    fi

    echo ""
    step "Service state:"
    systemctl status openarm-can-up.service --no-pager 2>/dev/null | head -15 || \
        echo -e "  ${YELLOW}Service not loaded.${NC}"

    echo ""
    step "Live CAN interfaces:"
    ip -details link show type can 2>/dev/null || echo -e "  ${YELLOW}No CAN interfaces found.${NC}"
}

# ── entry point ───────────────────────────────────────────────────────────────
case "${1:-install}" in
    install)   install ;;
    uninstall) uninstall ;;
    status)    status ;;
    *)
        echo -e "Usage: ${BOLD}sudo $0 {install|uninstall|status}${NC}"
        echo ""
        echo "  install    Detect openarm USB CAN adapter and install auto-up service"
        echo "  uninstall  Remove the service"
        echo "  status     Show service file + live CAN state"
        exit 1
        ;;
esac
