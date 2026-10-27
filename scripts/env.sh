#!/usr/bin/env bash
# Source this file from the repository root in each terminal.
TB4_GESTURE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [ -f "$TB4_GESTURE_ROOT/install/local_setup.bash" ]; then
    source "$TB4_GESTURE_ROOT/install/local_setup.bash"
fi
export ROS_LOG_DIR="$TB4_GESTURE_ROOT/debug/ros"
export IGN_LOG_PATH="$TB4_GESTURE_ROOT/debug/ignition"
export MPLCONFIGDIR="$TB4_GESTURE_ROOT/debug/matplotlib"
export FASTRTPS_DEFAULT_PROFILES_FILE="$TB4_GESTURE_ROOT/scripts/fastdds_udp.xml"
export PYTHONDONTWRITEBYTECODE=1
# Simulator and gesture nodes are on this laptop. Loopback avoids Ignition
# discovery choosing an unreachable container/VPN interface.
export IGN_IP="${IGN_IP:-127.0.0.1}"
mkdir -p "$ROS_LOG_DIR"
