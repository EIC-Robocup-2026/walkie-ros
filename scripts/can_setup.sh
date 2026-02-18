#!/bin/bash

# Configuration
IFACE=${2:-can0}
BITRATE=${3:-1000000}

function up() {
    echo "Starting $IFACE at $BITRATE bps..."
    sudo ip link set "$IFACE" type can bitrate "$BITRATE"
    sudo ip link set up "$IFACE"
    ip link show "$IFACE"
}

function down() {
    echo "Stopping $IFACE..."
    sudo ip link set down "$IFACE"
}

case "$1" in
    up)
        up
        ;;
    down)
        down
        ;;
    *)
        echo "Usage: $0 {up|down} [interface] [bitrate]"
        echo "Example: $0 up can0 1000000"
        exit 1
        ;;
esac