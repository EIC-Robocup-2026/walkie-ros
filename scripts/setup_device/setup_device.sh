#!/bin/bash
# setup_device.sh
# Unified persistent USB device setup: serial, camera, CAN, CAN FD.
#
# Usage:
#   sudo ./setup_device.sh install   --type serial  --alias ttyDynamixel  [--latency 1]
#   sudo ./setup_device.sh install   --type camera  --alias video_zed     [--index 0]
#   sudo ./setup_device.sh install   --type can     --alias can2           --bitrate 1000000
#   sudo ./setup_device.sh install   --type canfd   --alias can0           --bitrate 1000000 --dbitrate 5000000
#   sudo ./setup_device.sh uninstall --alias <name>
#   sudo ./setup_device.sh status    --alias <name>

set -euo pipefail
trap 'echo -e "\033[0;31m[ERROR]\033[0m Script failed at line $LINENO: $BASH_COMMAND" >&2' ERR

DETECT_TIMEOUT=30
IFACE_SETTLE_SEC=2   # wait after first CAN interface for multi-channel adapters

# ── colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
step()  { echo -e "${CYAN}◆${NC} $*"; }
div()   { echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"; }

require_root() { [[ $EUID -eq 0 ]] || error "Run with sudo: sudo $0 $*"; }
backup() { [[ -f "$1" ]] && { cp "$1" "${1}.bak"; warn "Backed up → ${1}.bak"; }; }

# Increment numeric suffix of an alias by N.
# "can0" + 1 → "can1",  "video_cam" + 2 → "video_cam2"
alias_n() {
    local base="$1" n="$2"
    if [[ "$base" =~ ^(.*[^0-9])([0-9]+)$ ]]; then
        echo "${BASH_REMATCH[1]}$(( BASH_REMATCH[2] + n ))"
    else
        echo "${base}${n}"
    fi
}

# ── serial / camera detection ─────────────────────────────────────────────────
# Args: type ("serial" | "camera")
# Sets globals: DETECTED_DEV, DETECTED_SERIAL, DETECTED_VENDOR, DETECTED_PRODUCT, DETECTED_MODEL
detect_serial_camera() {
    local type="$1"
    local before after new_dev

    if [[ "$type" == "serial" ]]; then
        before=$(find /dev -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyACM*' \) 2>/dev/null | sort || true)
    else
        before=$(find /dev -maxdepth 1 -name 'video*' 2>/dev/null | sort || true)
    fi

    div
    echo -e "  ${BOLD}Plug in the device now.${NC}"
    echo -e "  Waiting up to ${DETECT_TIMEOUT} seconds..."
    div; echo ""

    local elapsed=0
    new_dev=""
    while [[ $elapsed -lt $DETECT_TIMEOUT ]]; do
        if [[ "$type" == "serial" ]]; then
            after=$(find /dev -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyACM*' \) 2>/dev/null | sort || true)
        else
            after=$(find /dev -maxdepth 1 -name 'video*' 2>/dev/null | sort || true)
        fi
        new_dev=$(comm -13 <(echo "$before") <(echo "$after") | head -1 || true)
        [[ -n "$new_dev" ]] && { echo ""; break; }
        printf "\r  ⏳ %2ds elapsed... " "$elapsed"
        sleep 1
        (( elapsed++ )) || true
    done
    printf "\r                          \r"
    [[ -n "$new_dev" ]] || error "No new device appeared within ${DETECT_TIMEOUT}s."

    DETECTED_DEV="$new_dev"
    info "Detected: ${BOLD}${DETECTED_DEV}${NC}"

    local udev_info
    udev_info=$(udevadm info --query=all --name="$DETECTED_DEV" 2>/dev/null) \
        || error "udevadm failed for ${DETECTED_DEV}"

    DETECTED_SERIAL=$(echo  "$udev_info" | grep "^E: ID_SERIAL_SHORT=" | cut -d= -f2 || true)
    DETECTED_VENDOR=$(echo  "$udev_info" | grep "^E: ID_VENDOR_ID="    | cut -d= -f2 || true)
    DETECTED_PRODUCT=$(echo "$udev_info" | grep "^E: ID_MODEL_ID="     | cut -d= -f2 || true)
    DETECTED_MODEL=$(echo   "$udev_info" | grep "^E: ID_MODEL="        | cut -d= -f2 | tr '_' ' ' || true)
}

# ── CAN interface detection ────────────────────────────────────────────────────
# Sets globals: DETECTED_IFACES[], DETECTED_IFACE_ID_PATHS[],
#               DETECTED_SERIAL, DETECTED_VENDOR, DETECTED_PRODUCT, DETECTED_MODEL
detect_can() {
    local before
    before=$(ls /sys/class/net/ 2>/dev/null | grep '^can' | sort || true)

    if [[ -n "$before" ]]; then
        warn "CAN interface(s) already present: $(echo "$before" | tr '\n' ' ')"
        warn "Unplug the adapter first, or continue if none belong to this device."
        echo ""
        read -rp "  Continue anyway? [y/N]: " cont
        [[ "${cont,,}" =~ ^(y|yes)$ ]] || { warn "Aborted."; exit 0; }
    fi

    div
    echo -e "  ${BOLD}Plug in the USB CAN adapter now.${NC}"
    echo -e "  Waiting up to ${DETECT_TIMEOUT} seconds..."
    div; echo ""

    local elapsed=0 first_iface=""
    while [[ $elapsed -lt $DETECT_TIMEOUT ]]; do
        local after
        after=$(ls /sys/class/net/ 2>/dev/null | grep '^can' | sort || true)
        first_iface=$(comm -13 <(echo "$before") <(echo "$after") | head -1 || true)
        [[ -n "$first_iface" ]] && { echo ""; break; }
        printf "\r  ⏳ %2ds elapsed... " "$elapsed"
        sleep 1
        (( elapsed++ )) || true
    done
    printf "\r                          \r"
    [[ -n "$first_iface" ]] || error "No new CAN interface appeared within ${DETECT_TIMEOUT}s."

    info "First interface: ${BOLD}${first_iface}${NC}"
    info "Waiting ${IFACE_SETTLE_SEC}s for additional channels..."
    sleep "$IFACE_SETTLE_SEC"

    # Resolve USB parent via sysfs
    local first_net_path usb_dev_path
    first_net_path=$(readlink -f "/sys/class/net/${first_iface}") \
        || error "Cannot resolve sysfs path for ${first_iface}"
    usb_dev_path=$(dirname "$(dirname "$(dirname "$first_net_path")")")
    step "USB device path: ${usb_dev_path}"

    # Collect all CAN channels sharing this USB parent
    DETECTED_IFACES=()
    DETECTED_IFACE_ID_PATHS=()
    for iface in $(ls /sys/class/net/ 2>/dev/null | grep -E '^can[0-9]+$' || true); do
        local iface_net_path iface_usb_path
        iface_net_path=$(readlink -f "/sys/class/net/${iface}" 2>/dev/null) || continue
        iface_usb_path=$(dirname "$(dirname "$(dirname "$iface_net_path")")")
        if [[ "$iface_usb_path" == "$usb_dev_path" ]]; then
            local id_path
            id_path=$(udevadm info --query=all --path="/sys/class/net/${iface}" 2>/dev/null \
                | grep "^E: ID_PATH=" | cut -d= -f2 || true)
            DETECTED_IFACES+=("$iface")
            DETECTED_IFACE_ID_PATHS+=("$id_path")
            step "Found channel: ${iface}  (ID_PATH=${id_path:-n/a})"
        fi
    done

    if [[ ${#DETECTED_IFACES[@]} -eq 0 ]]; then
        warn "No channels matched USB parent — falling back to: ${first_iface}"
        DETECTED_IFACES=("$first_iface")
        DETECTED_IFACE_ID_PATHS=("")
    fi

    # USB device attributes
    local udev_info
    udev_info=$(udevadm info --query=all --path="$usb_dev_path" 2>/dev/null) \
        || error "udevadm failed for ${usb_dev_path}"

    DETECTED_SERIAL=$(echo  "$udev_info" | grep "^E: ID_SERIAL_SHORT=" | cut -d= -f2 || true)
    DETECTED_VENDOR=$(echo  "$udev_info" | grep "^E: ID_VENDOR_ID="    | cut -d= -f2 || true)
    DETECTED_PRODUCT=$(echo "$udev_info" | grep "^E: ID_MODEL_ID="     | cut -d= -f2 || true)
    DETECTED_MODEL=$(echo   "$udev_info" | grep "^E: ID_MODEL="        | cut -d= -f2 | tr '_' ' ' || true)
}

# ── serial install ─────────────────────────────────────────────────────────────
install_serial() {
    local alias="$1" latency="${2:-}"
    local rule_file="/etc/udev/rules.d/99-${alias}.rules"

    detect_serial_camera "serial"

    echo ""
    div
    printf "  ${BOLD}%-14s${NC} %s\n" "Model:"   "${DETECTED_MODEL:-unknown}"
    printf "  ${BOLD}%-14s${NC} %s\n" "USB ID:"  "${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Serial:"  "${DETECTED_SERIAL:-(none — VID:PID match)}"
    printf "  ${BOLD}%-14s${NC} /dev/%s\n" "Alias:"   "$alias"
    [[ -n "$latency" ]] && printf "  ${BOLD}%-14s${NC} %s ms\n" "Latency:" "$latency"
    printf "  ${BOLD}%-14s${NC} %s\n" "Rule:"    "$rule_file"
    div; echo ""
    read -rp "  Install? [Y/n]: " confirm
    [[ "${confirm,,}" =~ ^(y|yes|)$ ]] || { warn "Aborted."; exit 0; }
    echo ""

    local match
    if [[ -n "$DETECTED_SERIAL" ]]; then
        match="ENV{ID_SERIAL_SHORT}==\"${DETECTED_SERIAL}\""
    else
        match="ENV{ID_VENDOR_ID}==\"${DETECTED_VENDOR}\", ENV{ID_MODEL_ID}==\"${DETECTED_PRODUCT}\""
    fi

    local run_line=""
    [[ -n "$latency" ]] && \
        run_line="RUN+=\"/bin/sh -c 'echo ${latency} > /sys/bus/usb-serial/devices/%k/latency_timer'\", "

    backup "$rule_file"
    cat > "$rule_file" <<EOF
# Persistent alias /dev/${alias} — ${DETECTED_MODEL:-unknown} (${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?})
# Generated by setup_device.sh
SUBSYSTEM=="tty", ${match}, ${run_line}SYMLINK+="${alias}", MODE="0666"
EOF
    info "Rule → ${rule_file}"

    udevadm control --reload-rules
    udevadm trigger --subsystem-match=tty --action=add
    info "udev rules reloaded and triggered."

    # Apply latency immediately without waiting for replug
    if [[ -n "$latency" ]]; then
        local kern_dev latency_path
        kern_dev=$(basename "$DETECTED_DEV")
        latency_path="/sys/bus/usb-serial/devices/${kern_dev}/latency_timer"
        if [[ -f "$latency_path" ]]; then
            echo "$latency" > "$latency_path"
            info "Latency timer set to ${latency}ms on ${kern_dev}."
        fi
    fi

    echo ""; div
    info "✅ Done! /dev/${alias} active now and persists across plug-in."
    echo -e "   Check: ${CYAN}sudo $0 status --alias ${alias}${NC}"
    div
}

# ── camera install ─────────────────────────────────────────────────────────────
install_camera() {
    local alias="$1" index="${2:-0}"
    local rule_file="/etc/udev/rules.d/99-${alias}.rules"

    detect_serial_camera "camera"

    echo ""
    div
    printf "  ${BOLD}%-14s${NC} %s\n" "Model:"   "${DETECTED_MODEL:-unknown}"
    printf "  ${BOLD}%-14s${NC} %s\n" "USB ID:"  "${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Serial:"  "${DETECTED_SERIAL:-(none — VID:PID match)}"
    printf "  ${BOLD}%-14s${NC} /dev/%s  (video index %s)\n" "Alias:" "$alias" "$index"
    printf "  ${BOLD}%-14s${NC} %s\n" "Rule:"    "$rule_file"
    div; echo ""
    read -rp "  Install? [Y/n]: " confirm
    [[ "${confirm,,}" =~ ^(y|yes|)$ ]] || { warn "Aborted."; exit 0; }
    echo ""

    local match
    if [[ -n "$DETECTED_SERIAL" ]]; then
        match="ENV{ID_SERIAL_SHORT}==\"${DETECTED_SERIAL}\""
    else
        match="ENV{ID_VENDOR_ID}==\"${DETECTED_VENDOR}\", ENV{ID_MODEL_ID}==\"${DETECTED_PRODUCT}\""
    fi

    backup "$rule_file"
    cat > "$rule_file" <<EOF
# Persistent alias /dev/${alias} — ${DETECTED_MODEL:-unknown} (${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?})
# index ${index} selects the primary video node when a camera exposes multiple /dev/video* entries
# Generated by setup_device.sh
SUBSYSTEM=="video4linux", ${match}, ATTR{index}=="${index}", SYMLINK+="${alias}", MODE="0660", GROUP="video"
EOF
    info "Rule → ${rule_file}"

    udevadm control --reload-rules
    udevadm trigger --subsystem-match=video4linux --action=add
    info "udev rules reloaded and triggered."

    echo ""; div
    info "✅ Done! /dev/${alias} active now and persists across plug-in."
    echo -e "   Check: ${CYAN}sudo $0 status --alias ${alias}${NC}"
    div
}

# ── CAN / CAN FD install ───────────────────────────────────────────────────────
install_can_canfd() {
    local alias="$1" type="$2" bitrate="$3" dbitrate="${4:-}"

    detect_can

    echo ""
    div
    printf "  ${BOLD}%-14s${NC} %s\n" "Model:"    "${DETECTED_MODEL:-unknown}"
    printf "  ${BOLD}%-14s${NC} %s\n" "USB ID:"   "${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?}"
    printf "  ${BOLD}%-14s${NC} %s\n" "Serial:"   "${DETECTED_SERIAL:-(none — VID:PID match)}"
    printf "  ${BOLD}%-14s${NC} %s channel(s)\n"  "Detected:" "${#DETECTED_IFACES[@]}"
    for i in "${!DETECTED_IFACES[@]}"; do
        local chan_alias
        chan_alias=$(alias_n "$alias" "$i")
        printf "  ${BOLD}  %-12s${NC} %s → %s\n" "" "${DETECTED_IFACES[$i]}" "$chan_alias"
    done
    printf "  ${BOLD}%-14s${NC} %s bps\n" "Bitrate:"  "$bitrate"
    [[ -n "$dbitrate" ]] && printf "  ${BOLD}%-14s${NC} %s bps\n" "D-Bitrate:" "$dbitrate"
    printf "  ${BOLD}%-14s${NC} /etc/systemd/system/%s-up.service\n" "Service:" "$alias"
    div; echo ""
    read -rp "  Install? [Y/n]: " confirm
    [[ "${confirm,,}" =~ ^(y|yes|)$ ]] || { warn "Aborted."; exit 0; }
    echo ""

    # Write one .link file per channel for persistent interface naming
    for i in "${!DETECTED_IFACES[@]}"; do
        local chan_alias
        chan_alias=$(alias_n "$alias" "$i")
        local link_file="/etc/systemd/network/70-${chan_alias}.link"
        local id_path="${DETECTED_IFACE_ID_PATHS[$i]:-}"

        local match_section
        if [[ ${#DETECTED_IFACES[@]} -gt 1 && -n "$id_path" ]]; then
            # Multi-channel: ID_PATH distinguishes between channels on the same USB device
            match_section="Path=${id_path}"
        elif [[ -n "$DETECTED_SERIAL" ]]; then
            match_section="Property=ID_SERIAL_SHORT=${DETECTED_SERIAL}"
        else
            match_section="Property=ID_VENDOR_ID=${DETECTED_VENDOR}
Property=ID_MODEL_ID=${DETECTED_PRODUCT}"
        fi

        backup "$link_file"
        cat > "$link_file" <<EOF
# Persistent rename to ${chan_alias} — ${DETECTED_MODEL:-unknown}
# Generated by setup_device.sh
[Match]
${match_section}

[Link]
Name=${chan_alias}
EOF
        info "Link file → ${link_file}"
    done

    # Build service file covering all channels
    local after_lines="" binds_lines="" wantedby_lines="" exec_lines="" stop_lines=""
    for i in "${!DETECTED_IFACES[@]}"; do
        local chan_alias
        chan_alias=$(alias_n "$alias" "$i")
        local unit="sys-subsystem-net-devices-${chan_alias}.device"
        after_lines+="After=${unit}"$'\n'
        binds_lines+="BindsTo=${unit}"$'\n'
        wantedby_lines+="WantedBy=${unit}"$'\n'

        local ip_type_args
        if [[ "$type" == "canfd" ]]; then
            ip_type_args="type can bitrate ${bitrate} dbitrate ${dbitrate} fd on"
        else
            ip_type_args="type can bitrate ${bitrate}"
        fi
        exec_lines+="-ExecStart=/sbin/ip link set ${chan_alias} down"$'\n'
        exec_lines+="ExecStart=/sbin/ip link set ${chan_alias} ${ip_type_args}"$'\n'
        exec_lines+="ExecStart=/sbin/ip link set ${chan_alias} up"$'\n'
        stop_lines+="-ExecStop=/sbin/ip link set ${chan_alias} down"$'\n'
    done

    local service_file="/etc/systemd/system/${alias}-up.service"
    backup "$service_file"
    cat > "$service_file" <<EOF
# Auto-configures ${alias} CAN interface(s) on plug-in.
# Generated by setup_device.sh
# Device: ${DETECTED_MODEL:-unknown} (${DETECTED_VENDOR:-?}:${DETECTED_PRODUCT:-?})
[Unit]
Description=Configure ${alias} CAN interface(s) (${type^^}, ${bitrate}${dbitrate:+/${dbitrate}})
${after_lines}${binds_lines}
[Service]
Type=oneshot
RemainAfterExit=yes
${exec_lines}${stop_lines}StandardOutput=journal
StandardError=journal

[Install]
${wantedby_lines}
EOF
    info "Service → ${service_file}"

    systemctl daemon-reload
    systemctl enable "${alias}-up.service"
    info "Service enabled."

    # Apply native CAN config immediately since interfaces are already live
    info "Configuring interfaces now..."
    for i in "${!DETECTED_IFACES[@]}"; do
        local iface="${DETECTED_IFACES[$i]}"
        local ip_type_args
        if [[ "$type" == "canfd" ]]; then
            ip_type_args="type can bitrate ${bitrate} dbitrate ${dbitrate} fd on"
        else
            ip_type_args="type can bitrate ${bitrate}"
        fi
        ip link set "$iface" down 2>/dev/null || true
        ip link set "$iface" $ip_type_args \
            && ip link set "$iface" up \
            && info "  ${iface} → up" \
            || warn "  ${iface} → failed (check above)"
    done

    echo ""; div
    info "✅ Done! Interface(s) will auto-configure on plug-in."
    echo -e "   Check: ${CYAN}sudo $0 status --alias ${alias}${NC}"
    div
}

# ── uninstall ─────────────────────────────────────────────────────────────────
cmd_uninstall() {
    local alias="$1"
    local removed=0

    # serial / camera: udev rule
    local rule_file="/etc/udev/rules.d/99-${alias}.rules"
    if [[ -f "$rule_file" ]]; then
        rm "$rule_file"
        udevadm control --reload-rules
        info "Removed ${rule_file} and reloaded udev."
        (( removed++ )) || true
    fi

    # CAN / CAN FD: service + link files
    local service_file="/etc/systemd/system/${alias}-up.service"
    if [[ -f "$service_file" ]]; then
        # Extract all channel aliases from BindsTo= lines before removing the file
        local chan_aliases=()
        mapfile -t chan_aliases < <(
            grep '^BindsTo=' "$service_file" \
            | sed 's/BindsTo=sys-subsystem-net-devices-\(.*\)\.device/\1/' || true
        )

        systemctl disable "${alias}-up.service" 2>/dev/null || true
        systemctl stop    "${alias}-up.service" 2>/dev/null || true
        rm "$service_file"
        info "Removed service ${service_file}."
        (( removed++ )) || true

        for chan in "${chan_aliases[@]}"; do
            local link_file="/etc/systemd/network/70-${chan}.link"
            if [[ -f "$link_file" ]]; then
                rm "$link_file"
                info "Removed link file ${link_file}."
                (( removed++ )) || true
            fi
        done
        systemctl daemon-reload
    fi

    [[ $removed -gt 0 ]] || warn "Nothing found for alias '${alias}'."
}

# ── status ────────────────────────────────────────────────────────────────────
cmd_status() {
    local alias="$1"

    # serial / camera
    local rule_file="/etc/udev/rules.d/99-${alias}.rules"
    if [[ -f "$rule_file" ]]; then
        echo ""; step "udev rule: ${rule_file}"
        sed 's/^/    /' "$rule_file"
        echo ""; step "Device symlink:"
        ls -la "/dev/${alias}" 2>/dev/null \
            || echo -e "  ${YELLOW}/dev/${alias} not present (device unplugged?)${NC}"
        return
    fi

    # CAN / CAN FD
    local service_file="/etc/systemd/system/${alias}-up.service"
    local found_can=0

    if [[ -f "$service_file" ]]; then
        # Show link files listed in service BindsTo= lines
        local chan_aliases=()
        mapfile -t chan_aliases < <(
            grep '^BindsTo=' "$service_file" \
            | sed 's/BindsTo=sys-subsystem-net-devices-\(.*\)\.device/\1/' || true
        )
        for chan in "${chan_aliases[@]}"; do
            local link_file="/etc/systemd/network/70-${chan}.link"
            if [[ -f "$link_file" ]]; then
                echo ""; step "Link file: ${link_file}"
                sed 's/^/    /' "$link_file"
            fi
        done

        echo ""; step "Service file: ${service_file}"
        sed 's/^/    /' "$service_file"

        echo ""; step "Service state:"
        systemctl status "${alias}-up.service" --no-pager 2>/dev/null | head -15 \
            || echo -e "  ${YELLOW}Service not loaded.${NC}"

        echo ""; step "Live CAN interfaces:"
        ip -details link show type can 2>/dev/null \
            || echo -e "  ${YELLOW}No CAN interfaces found.${NC}"

        found_can=1
    fi

    [[ $found_can -eq 1 ]] || warn "No setup found for alias '${alias}'."
}

# ── usage ─────────────────────────────────────────────────────────────────────
usage() {
    echo -e "${BOLD}Unified USB device setup — persistent alias / interface naming${NC}"
    echo ""
    echo -e "${BOLD}Usage:${NC}"
    echo "  sudo $0 install   --type <type> --alias <name> [options]"
    echo "  sudo $0 uninstall --alias <name>"
    echo "  sudo $0 status    --alias <name>"
    echo ""
    echo -e "${BOLD}Types and options:${NC}"
    echo "  serial   USB-to-serial  →  /dev/<alias>"
    echo "           --latency <ms>     set USB latency timer (e.g. 1 for Dynamixel)"
    echo ""
    echo "  camera   USB camera     →  /dev/<alias>"
    echo "           --index <n>        video node index to alias (default: 0)"
    echo ""
    echo "  can      USB CAN        →  interface <alias>"
    echo "           --bitrate <bps>    (required)"
    echo ""
    echo "  canfd    USB CAN FD     →  interface <alias>"
    echo "           --bitrate <bps>    nominal bitrate  (required)"
    echo "           --dbitrate <bps>   data bitrate     (required)"
    echo "           Multi-channel adapters are auto-detected; sibling channels"
    echo "           are named <alias>+1, <alias>+2, etc."
    echo ""
    echo -e "${BOLD}Examples:${NC}"
    echo "  sudo $0 install   --type serial --alias ttyDynamixel --latency 1"
    echo "  sudo $0 install   --type camera --alias video_zed"
    echo "  sudo $0 install   --type can    --alias can2 --bitrate 1000000"
    echo "  sudo $0 install   --type canfd  --alias can0 --bitrate 1000000 --dbitrate 5000000"
    echo "  sudo $0 status    --alias ttyDynamixel"
    echo "  sudo $0 uninstall --alias can0"
}

# ── entry point ───────────────────────────────────────────────────────────────
main() {
    local cmd="${1:-}"
    [[ -n "$cmd" ]] || { usage; exit 1; }
    shift

    local type="" alias="" bitrate="" dbitrate="" latency="" index="0"
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --type)     type="$2";     shift 2 ;;
            --alias)    alias="$2";    shift 2 ;;
            --bitrate)  bitrate="$2";  shift 2 ;;
            --dbitrate) dbitrate="$2"; shift 2 ;;
            --latency)  latency="$2";  shift 2 ;;
            --index)    index="$2";    shift 2 ;;
            *) error "Unknown option: $1" ;;
        esac
    done

    case "$cmd" in
        install)
            require_root
            [[ -n "$alias" ]] || error "--alias is required"
            [[ -n "$type"  ]] || error "--type is required (serial|camera|can|canfd)"
            case "$type" in
                serial) install_serial  "$alias" "$latency" ;;
                camera) install_camera  "$alias" "$index" ;;
                can)
                    [[ -n "$bitrate" ]] || error "--bitrate is required for --type can"
                    install_can_canfd "$alias" "can" "$bitrate"
                    ;;
                canfd)
                    [[ -n "$bitrate"  ]] || error "--bitrate is required for --type canfd"
                    [[ -n "$dbitrate" ]] || error "--dbitrate is required for --type canfd"
                    install_can_canfd "$alias" "canfd" "$bitrate" "$dbitrate"
                    ;;
                *) error "Unknown type '${type}'. Use: serial|camera|can|canfd" ;;
            esac
            ;;
        uninstall)
            require_root
            [[ -n "$alias" ]] || error "--alias is required"
            cmd_uninstall "$alias"
            ;;
        status)
            [[ -n "$alias" ]] || error "--alias is required"
            cmd_status "$alias"
            ;;
        *)
            usage; exit 1 ;;
    esac
}

main "$@"
