#!/usr/bin/env python3
# 读取 snappy-real 控制进程的内存快照，打印各传感器当前数值（名称默认中文）。
#
# 用法（板子上以 root 执行）：
#   ./sensor_peek.py                 一次，按表格打印全部传感器数值
#   ./sensor_peek.py -w              持续输出，默认 1 秒一行，Ctrl-C 退出
#   ./sensor_peek.py -w 0.5          自定义刷新周期（秒）
#   ./sensor_peek.py -e              显示原始英文逻辑名（排查/对配置时用）
#   ./sensor_peek.py -j              输出原始 JSON（含 quality/age_ms/devices）
#   ./sensor_peek.py -s <path>       指定控制 socket（默认 /run/snappy-real/control.sock）
#   ./sensor_peek.py --timeout 2     单次请求超时秒数（默认 1）
#
#    python3 /root/snappy-real/tools/sensor_peek.py
#
#
# 输出示例：
#   NAME                  VALUE  UNIT    QUALITY     AGE_MS
#   机柜 温度              26.5  degC    GOOD           100
#   24V电源 电压          24.03  V       GOOD           405
#   故障设备: 48V电源
#
# 行为与退出码：
#   - 请求内容固定为 gateway.status_snapshot（snappy-gateway 自己每 250ms 就发同一种请求），
#     脚本不会发送任何其它 action，也不做重试轰炸。
#   - 只读：每次连一条连接、发一个 gateway.status_snapshot 请求、读完即关；
#     只复制控制进程里的内存缓存，不触发 RS485 读写，不下发任何控制指令。
#   - 退出码 0 成功；1 查询失败；2 找不到 socket（控制进程没在运行）。

import argparse
import json
import os
import socket
import sys
import time
import unicodedata

DEFAULT_SOCKET = "/run/snappy-real/control.sock"


REQUEST = {
    "schema_version": 1,
    "kind": "command",
    "source_id": "sensor_peek",
    "action": "gateway.status_snapshot",
}

# 样本逻辑名到中文标签的对照表，未收录的名字原样输出。
SENSOR_NAMES = {
    "anemometer_left_speed": "左风速 风速",
    "anemometer_right_speed": "右风速 风速",
    "lidar_body_front_distance": "机身前 距离",
    "lidar_body_rear_distance": "机身后 距离",
    "lidar_brush_left_distance": "毛刷左 距离",
    "lidar_brush_left_distance_temperature": "毛刷左 温度",
    "lidar_brush_right_distance": "毛刷右 距离",
    "lidar_brush_right_distance_temperature": "毛刷右 温度",
    "power_24v_current": "24V电源 电流",
    "power_24v_operation_enabled": "24V电源 输出使能",
    "power_24v_power": "24V电源 功率",
    "power_24v_temperature": "24V电源 温度",
    "power_24v_voltage": "24V电源 电压",
    "power_48v_current": "48V电源 电流",
    "power_48v_operation_enabled": "48V电源 输出使能",
    "power_48v_power": "48V电源 功率",
    "power_48v_temperature": "48V电源 温度",
    "power_48v_voltage": "48V电源 电压",
    "temperature_cabinet_humidity": "机柜 湿度",
    "temperature_cabinet_temperature": "机柜 温度",
    "wit_motion_accel_x": "姿态传感器 X轴加速度",
    "wit_motion_accel_y": "姿态传感器 Y轴加速度",
    "wit_motion_accel_z": "姿态传感器 Z轴加速度",
    "wit_motion_gyro_x": "姿态传感器 X轴角速度",
    "wit_motion_gyro_y": "姿态传感器 Y轴角速度",
    "wit_motion_gyro_z": "姿态传感器 Z轴角速度",
    "wit_motion_pitch": "姿态传感器 俯仰角",
    "wit_motion_roll": "姿态传感器 横滚角",
    "wit_motion_yaw": "姿态传感器 偏航角",
    "wit_motion_temperature": "姿态传感器 温度",
}

# 设备逻辑名到中文名称的对照表，用于故障设备提示。
DEVICE_NAMES = {
    "anemometer_left": "左风速仪",
    "anemometer_right": "右风速仪",
    "lidar_body_front": "前机身激光",
    "lidar_body_rear": "后机身激光",
    "lidar_brush_left": "左刷盘激光",
    "lidar_brush_right": "右刷盘激光",
    "power_24v": "24V电源",
    "power_48v": "48V电源",
    "temperature_cabinet": "机柜温湿度",
    "wit_motion": "姿态传感器",
}


def query(sock_path, timeout):
    """向控制 socket 发一次快照请求并返回解析后的 JSON。"""
    client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    client.settimeout(timeout)
    try:
        client.connect(sock_path)
        client.send(json.dumps(REQUEST).encode("utf-8"))
        raw = client.recv(262144)
    finally:
        client.close()
    if not raw:
        raise RuntimeError("控制进程没有返回数据")
    return json.loads(raw.decode("utf-8", "replace"))


def label(name, english):
    """把逻辑名翻成中文标签，未知名称或要求英文时原样返回。"""
    if english:
        return name
    return SENSOR_NAMES.get(name) or DEVICE_NAMES.get(name) or name


def disp_width(text):
    """按终端显示宽度计算字符串宽度（中日韩字符占两格）。"""
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)


def pad(text, width, right=False):
    """按显示宽度补空格对齐一列。"""
    gap = max(width - disp_width(text), 0)
    return (" " * gap + text) if right else (text + " " * gap)


def format_number(sample):
    """把一条样本的数值格式化成字符串，空值输出占位符。"""
    value = sample.get("value")
    if value is None:
        return "-"
    if isinstance(value, float):
        return "%g" % value
    return str(value)


def format_value(sample):
    """把一条样本格式化成 数值+单位，用于单行输出。"""
    if sample.get("value") is None:
        return "-"
    return "{}{}".format(format_number(sample), sample.get("unit") or "")


def print_table(data, english=False):
    """按表格输出全部传感器样本，并列出故障设备。"""
    sensors = data.get("sensors")
    if not isinstance(sensors, dict):
        print("控制进程返回非快照内容：{}".format(json.dumps(data, ensure_ascii=False)))
        return
    samples = sensors.get("samples") or []
    if not samples:
        print("没有取到传感器样本（mode={}）".format(sensors.get("mode")))
        return

    rows = []
    for item in samples:
        rows.append((
            label(str(item.get("name", "")), english),
            format_number(item),
            str(item.get("unit") or "-"),
            str(item.get("quality") or "-"),
            str(item.get("age_ms", "-")),
        ))

    name_width = max([disp_width(row[0]) for row in rows] + [disp_width("NAME")])
    value_width = max([len(row[1]) for row in rows] + [len("VALUE")])

    print("{}  {}  {}  {}  {}".format(
        pad("NAME", name_width),
        pad("VALUE", value_width, True),
        pad("UNIT", 7),
        pad("QUALITY", 8),
        pad("AGE_MS", 8, True)))
    for row in rows:
        print("{}  {}  {}  {}  {}".format(
            pad(row[0], name_width),
            pad(row[1], value_width, True),
            pad(row[2], 7),
            pad(row[3], 8),
            pad(row[4], 8, True)))

    faults = [label(str(item.get("name")), english)
              for item in (sensors.get("devices") or [])
              if not item.get("online", True)]
    if faults:
        print("故障设备: {}".format(", ".join(faults)))


def print_line(data, english=False):
    """单行输出全部传感器数值，便于 watch 时连续打印。"""
    sensors = data.get("sensors")
    if not isinstance(sensors, dict):
        print("控制进程返回非快照内容")
        return
    parts = []
    for item in sensors.get("samples") or []:
        parts.append("{}={}".format(label(str(item.get("name", "?")), english),
                                    format_value(item)))
    print("{}  {}".format(time.strftime("%H:%M:%S"),
                          "  ".join(parts) if parts else "无样本"))


def main():
    """解析参数并执行一次或持续查询。"""
    parser = argparse.ArgumentParser(
        description="打印 snappy-real 各传感器当前数值（只读内存快照，不触发总线读写）")
    parser.add_argument("-s", "--socket", default=DEFAULT_SOCKET, help="控制 socket 路径")
    parser.add_argument("-w", "--watch", nargs="?", type=float, const=1.0,
                        metavar="SEC", help="持续输出，默认 1 秒一次")
    parser.add_argument("-e", "--english", action="store_true",
                        help="显示原始英文逻辑名（默认中文）")
    parser.add_argument("-j", "--json", action="store_true", help="输出原始 JSON")
    parser.add_argument("--timeout", type=float, default=1.0, help="单次请求超时秒数")
    args = parser.parse_args()

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)

    if not os.path.exists(args.socket):
        sys.stderr.write(
            "找不到 {}，控制进程可能没在运行（systemctl status snappy-real-control）\n"
            .format(args.socket))
        return 2

    def render(payload):
        """按当前模式输出一次查询结果。"""
        if args.json:
            print(json.dumps(payload, ensure_ascii=False))
        elif args.watch is not None:
            print_line(payload, args.english)
        else:
            print_table(payload, args.english)

    try:
        while True:
            try:
                render(query(args.socket, args.timeout))
            except Exception as error:
                sys.stderr.write("查询失败: {}\n".format(error))
                if args.watch is None:
                    return 1
            if args.watch is None:
                return 0
            time.sleep(max(args.watch, 0.1))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
