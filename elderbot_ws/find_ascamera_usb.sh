#!/usr/bin/env bash

set -euo pipefail

RULES_FILE="/home/ztl/Robot-nav/elderbot_ws/src/ascamera/scripts/angstrong-camera.rules"
SYS_USB_DIR="/sys/bus/usb/devices"

if [[ ! -f "$RULES_FILE" ]]; then
    echo "未找到规则文件: $RULES_FILE"
    exit 1
fi

if [[ ! -d "$SYS_USB_DIR" ]]; then
    echo "未找到 USB sysfs 目录: $SYS_USB_DIR"
    exit 1
fi

mapfile -t RULE_PAIRS < <(
    sed -n 's/.*ATTRS{idVendor}=="\([^"]*\)".*ATTRS{idProduct}=="\([^"]*\)".*/\1:\2/p' "$RULES_FILE" | sort -u
)

if [[ ${#RULE_PAIRS[@]} -eq 0 ]]; then
    echo "规则文件中没有解析出任何 vid:pid"
    exit 1
fi

declare -a MATCHES=()

for dev in "$SYS_USB_DIR"/*; do
    [[ -f "$dev/idVendor" && -f "$dev/idProduct" ]] || continue

    vid="$(tr '[:upper:]' '[:lower:]' < "$dev/idVendor")"
    pid="$(tr '[:upper:]' '[:lower:]' < "$dev/idProduct")"
    pair="$vid:$pid"

    matched="false"
    for rule_pair in "${RULE_PAIRS[@]}"; do
        if [[ "$pair" == "$rule_pair" ]]; then
            matched="true"
            break
        fi
    done

    [[ "$matched" == "true" ]] || continue

    bus="$(cat "$dev/busnum" 2>/dev/null || echo '?')"
    path="$(cat "$dev/devpath" 2>/dev/null || echo '?')"
    manufacturer="$(cat "$dev/manufacturer" 2>/dev/null || echo '?')"
    product="$(cat "$dev/product" 2>/dev/null || echo '?')"
    serial="$(cat "$dev/serial" 2>/dev/null || echo '-')"
    sys_name="$(basename "$dev")"

    MATCHES+=("${bus}|${path}|${vid}|${pid}|${manufacturer}|${product}|${serial}|${sys_name}")
done

if [[ ${#MATCHES[@]} -eq 0 ]]; then
    echo "当前没有检测到 Angstrong 相机类 USB 设备。"
    echo "请确认相机已插好，并先运行过 udev 规则安装脚本。"
    exit 0
fi

mapfile -t SORTED_MATCHES < <(printf '%s\n' "${MATCHES[@]}" | sort -t'|' -k1,1n -k2,2V)

echo "检测到 ${#SORTED_MATCHES[@]} 个 Angstrong 相机类 USB 设备:"
echo

index=1
for item in "${SORTED_MATCHES[@]}"; do
    IFS='|' read -r bus path vid pid manufacturer product serial sys_name <<< "$item"
    echo "[$index] sysfs=$sys_name  vid:pid=$vid:$pid  bus=$bus  path=$path"
    echo "    manufacturer=$manufacturer  product=$product  serial=$serial"
    echo "    启动参数: camera_${index}_usb_bus_no:=${bus} camera_${index}_usb_path:=${path}"
    echo
    index=$((index + 1))
done

if [[ ${#SORTED_MATCHES[@]} -ge 2 ]]; then
    IFS='|' read -r bus1 path1 _ <<< "${SORTED_MATCHES[0]}"
    IFS='|' read -r bus2 path2 _ <<< "${SORTED_MATCHES[1]}"
    echo "双相机启动示例:"
    echo "ros2 launch ascamera hp60c.launch.py camera_1_usb_bus_no:=${bus1} camera_1_usb_path:=${path1} camera_2_enabled:=true camera_2_usb_bus_no:=${bus2} camera_2_usb_path:=${path2}"
else
    echo "目前只检测到 1 台设备。接上第二台后重新运行本脚本，就会给出双相机启动参数。"
fi
