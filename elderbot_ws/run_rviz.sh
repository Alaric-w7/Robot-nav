#!/bin/bash
# ElderBot - Run RViz with RK3588-friendly OpenGL defaults

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

print_usage() {
    cat <<'EOF'
Usage:
  bash run_rviz.sh [--software] [navigation|slam|description|/path/to/file.rviz] [extra rviz2 args...]

Examples:
  bash run_rviz.sh
  bash run_rviz.sh navigation
  bash run_rviz.sh slam
  bash run_rviz.sh --software navigation
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    print_usage
    exit 0
fi

MODE="${ELDERBOT_RVIZ_MODE:-auto}"
if [ "${1:-}" = "--software" ]; then
    MODE="software"
    shift
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble is not installed at /opt/ros/humble."
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "Workspace is not built yet: $SCRIPT_DIR/install/setup.bash"
    echo "Run: cd $SCRIPT_DIR && colcon build"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

PROFILE="${1:-navigation}"
if [ "$#" -gt 0 ]; then
    shift
fi
EXTRA_ARGS=("$@")

case "$PROFILE" in
    navigation)
        CANDIDATES=(
            "$SCRIPT_DIR/src/elderbot_navigation/rviz/navigation.rviz"
            "$SCRIPT_DIR/install/elderbot_navigation/share/elderbot_navigation/rviz/navigation.rviz"
        )
        ;;
    slam)
        CANDIDATES=(
            "$SCRIPT_DIR/src/elderbot_navigation/rviz/slam.rviz"
            "$SCRIPT_DIR/install/elderbot_navigation/share/elderbot_navigation/rviz/slam.rviz"
        )
        ;;
    description)
        CANDIDATES=(
            "$SCRIPT_DIR/src/elderbot_description/rviz/description.rviz"
            "$SCRIPT_DIR/install/elderbot_description/share/elderbot_description/rviz/description.rviz"
        )
        ;;
    "~/"*)
        CANDIDATES=("$HOME/${PROFILE#~/}")
        ;;
    *)
        CANDIDATES=("$PROFILE")
        ;;
esac

RVIZ_CONFIG=""
for candidate in "${CANDIDATES[@]}"; do
    if [ -f "$candidate" ]; then
        RVIZ_CONFIG="$candidate"
        break
    fi
done

if [ -z "$RVIZ_CONFIG" ]; then
    echo "RViz config not found: $PROFILE"
    exit 1
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "Neither DISPLAY nor WAYLAND_DISPLAY is set, RViz cannot open a window."
    exit 1
fi

enable_rk3588_workaround() {
    if [ "$(uname -m)" != "aarch64" ]; then
        return
    fi

    if [ ! -f /usr/lib/aarch64-linux-gnu/dri/rockchip_dri.so ]; then
        return
    fi

    local mesa_path="/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu"
    export LD_LIBRARY_PATH="${mesa_path}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export QT_OPENGL=desktop
    export QT_XCB_GL_INTEGRATION=xcb_glx
    export __GLX_VENDOR_LIBRARY_NAME=mesa
    export LIBGL_DRI3_DISABLE=1
    unset LIBGL_ALWAYS_SOFTWARE
    unset MESA_LOADER_DRIVER_OVERRIDE

    echo "Using RK3588 RViz GL compatibility settings."
}

if [ "$MODE" = "software" ]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export QT_OPENGL=desktop
    export QT_XCB_GL_INTEGRATION=xcb_glx
    export __GLX_VENDOR_LIBRARY_NAME=mesa
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
    export LIBGL_DRI3_DISABLE=1
    echo "Using software OpenGL for RViz."
else
    enable_rk3588_workaround
fi

echo "RViz config: $RVIZ_CONFIG"
exec rviz2 -d "$RVIZ_CONFIG" "${EXTRA_ARGS[@]}"
