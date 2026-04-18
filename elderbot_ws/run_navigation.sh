#!/bin/bash
# ElderBot - Run Navigation (with pre-built map)
# Usage: bash run_navigation.sh [map_path] [initial_x] [initial_y] [initial_yaw]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source Orbbec camera workspace if available
if [ -f ~/orbbec_ws/install/setup.bash ]; then
    source ~/orbbec_ws/install/setup.bash
fi

# Source the workspace that contains this script so we don't accidentally pick
# up another elderbot_ws from HOME.
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "[ERROR] Workspace setup file not found: $WORKSPACE_DIR/install/setup.bash"
    echo "        Please build this workspace first."
    exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"

if ! ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
    echo "[ERROR] ROS environment cannot find package 'orbbec_camera'."
    echo "        Having the source folder is not enough; it must be built into install/ first."
    echo "        Try:"
    echo "        cd $WORKSPACE_DIR"
    echo "        source /opt/ros/humble/setup.bash"
    echo "        colcon build --packages-select orbbec_camera_msgs orbbec_description orbbec_camera elderbot_bringup elderbot_navigation"
    exit 1
fi

python_has_module() {
    local module="$1"
    python3 - "$module" <<'PY' >/dev/null 2>&1
import importlib.util
import sys

sys.exit(0 if importlib.util.find_spec(sys.argv[1]) else 1)
PY
}

add_pythonpath_if_module_exists() {
    local module="$1"
    local site_path="$2"

    if [ -z "$site_path" ] || [ ! -d "$site_path" ]; then
        return 1
    fi

    if PYTHONPATH="$site_path${PYTHONPATH:+:$PYTHONPATH}" python3 - "$module" <<'PY' >/dev/null 2>&1
import importlib.util
import sys

sys.exit(0 if importlib.util.find_spec(sys.argv[1]) else 1)
PY
    then
        export PYTHONPATH="$site_path${PYTHONPATH:+:$PYTHONPATH}"
        return 0
    fi

    return 1
}

ensure_python_module() {
    local module="$1"
    local package_hint="$2"

    if python_has_module "$module"; then
        return 0
    fi

    local py_minor
    py_minor="$(python3 - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"

    local current_user
    current_user="$(id -un)"

    local workspace_owner=""
    workspace_owner="$(stat -c '%U' "$WORKSPACE_DIR" 2>/dev/null || true)"

    local candidate_users=()
    if [ -n "${SUDO_USER:-}" ] && [ "$SUDO_USER" != "$current_user" ]; then
        candidate_users+=("$SUDO_USER")
    fi
    if [ -n "$workspace_owner" ] && [ "$workspace_owner" != "$current_user" ]; then
        local already_added="false"
        local candidate
        for candidate in "${candidate_users[@]}"; do
            if [ "$candidate" = "$workspace_owner" ]; then
                already_added="true"
                break
            fi
        done
        if [ "$already_added" = "false" ]; then
            candidate_users+=("$workspace_owner")
        fi
    fi

    local candidate_user=""
    for candidate_user in "${candidate_users[@]}"; do
        local candidate_home=""
        candidate_home="$(eval echo "~$candidate_user")"
        if add_pythonpath_if_module_exists \
            "$module" \
            "$candidate_home/.local/lib/python$py_minor/site-packages"; then
            echo "[INFO] Reused Python module '$module' from $candidate_user's user site-packages."
            return 0
        fi
    done

    echo "[ERROR] Python module '$module' is missing in the current launch environment."
    echo "        This blocks the ultrasonic driver from starting."
    echo "        Install it system-wide or for the user that runs this script:"
    echo "        python3 -m pip install --user $package_hint"
    echo "        or"
    echo "        sudo apt install python3-$package_hint"
    exit 1
}

ensure_python_module serial pyserial

MAP_PATH="${1:-$HOME/maps/my_map1.yaml}"
INIT_X="${2:-0.0}"
INIT_Y="${3:-0.0}"
INIT_YAW="${4:-0.0}"

echo "=========================================="
echo "  ElderBot Navigation"
echo "  Map: $MAP_PATH"
echo "  Initial Pose: x=$INIT_X, y=$INIT_Y, yaw=$INIT_YAW"
echo "=========================================="

ros2 launch elderbot_bringup bringup.launch.py \
    map:="$MAP_PATH" \
    initial_pose_x:="$INIT_X" \
    initial_pose_y:="$INIT_Y" \
    initial_pose_yaw:="$INIT_YAW"
