#!/usr/bin/env python3
"""Windows ADAS control panel. All UDP waits run outside the Tk event thread."""

from __future__ import annotations

import argparse
import queue
import socket
import threading
import time
import uuid


ACTIONS = {"stop": "停止", "straight": "直行", "left": "左转", "right": "右转",
           "lane_stop": "停止变道", "lane_left": "左变道", "lane_right": "右变道"}
REASONS = {
    "stop": "停止指令", "assist_off": "辅助已关闭", "command_expired": "驾驶指令已过期",
    "red_signal": "对应方向红灯，停车", "green_signal": "对应方向绿灯",
    "right_turn_yield": "未识别到右转专用灯，可右转；须确认让行",
    "yellow_direction_unresolved": "黄灯警告（模型未区分方向）",
    "left_solid_line": "左侧实线警告", "right_solid_line": "右侧实线警告",
    "signal_stale": "信号结果未就绪或已过期", "signal_unknown": "对应方向灯态未知",
    "green_confirming": "正在确认绿灯", "signal_assignment_required": "尚未确认车辆信号区域",
    "lane_unknown": "对应侧车道线未知", "dashed_line": "对应侧虚线，允许变道",
    "dashed_line_no_boundary_warning": "虚线，旧版未放行",
    "both_allowed": "灯控与变道均允许",
}


def describe_status(values: dict[str, str]) -> tuple[str, str, str, str, str, str, str]:
    """Turn the three independent board decisions into operator-facing text."""
    signal_action = values.get("action", "stop")
    lane_action = values.get("lane_action", "stop")
    signal_state = values.get("signal_state", "idle")
    lane_state = values.get("lane_state", "idle")
    pedestrian_state = values.get("pedestrian_state", "unknown")
    direction = ACTIONS.get(signal_action, signal_action)
    maneuver = ACTIONS.get(lane_action, lane_action)
    side = "左侧" if lane_action == "lane_left" else "右侧"

    def lamp(value: str) -> str:
        if value.endswith("_red"):
            return "红灯"
        if value.endswith("_green"):
            return "绿灯"
        return "熄灭" if value == "off" else "未知"

    signal_seen = ("红绿灯识别：直行 " + lamp(values.get("straight", "unknown")) +
                   "    左转 " + lamp(values.get("left", "unknown")) +
                   "    右转 " + lamp(values.get("right", "unknown")))
    if values.get("yellow") == "1":
        signal_seen += "    检测到黄灯"

    line_names = {"solid": "实线", "dashed": "虚线", "unknown": "未知"}
    def line_description(side_name: str, key: str) -> str:
        kind = values.get(key, "unknown")
        meaning = {"solid": "禁止变道", "dashed": "具备跨线条件", "unknown": "暂不变道"}.get(kind, "暂不变道")
        return side_name + line_names.get(kind, "未知") + "（" + meaning + "）"
    lane_seen = "车道线识别：" + line_description("左", "lane_left") + "    " + line_description("右", "lane_right")
    if pedestrian_state == "idle":
        pedestrian_seen = "行人/斑马线识别：检测暂停"
    else:
        pedestrian_seen = ("行人/斑马线识别：行人 " + ("有" if values.get("person_seen") == "1" else "无") +
                           "    斑马线 " + ("有" if values.get("crosswalk_seen") == "1" else "无") +
                           "    本帧行人在斑马线及本车路径内 " +
                           ("是" if values.get("person_on_crosswalk") == "1" else "否"))

    if signal_action == "stop":
        signal_text = "灯控判断：未选择行驶方向"
    elif signal_state == "idle":
        signal_text = "灯控判断：指令已过期，暂不通行"
    elif signal_state == "stop":
        signal_text = f"灯控判断：{direction}红灯，停车并报警"
    elif signal_state == "allow" and values.get("signal_reason") == "right_turn_yield":
        signal_text = "灯控判断：未识别到右转专用灯，可右转；须确认让行"
    elif signal_state == "allow":
        signal_text = f"灯控判断：{direction}绿灯，可通行"
    elif signal_state == "warning":
        signal_text = f"灯控判断：{direction}黄灯警告，暂不通行"
    elif values.get("signal_reason") == "green_confirming":
        signal_text = f"灯控判断：正在确认{direction}绿灯，暂不通行"
    else:
        signal_text = f"灯控判断：{direction}灯态未确认，暂不通行"

    if lane_action == "stop":
        lane_text = "变道判断：未选择变道方向"
    elif lane_state == "idle":
        lane_text = "变道判断：指令已过期，暂不变道"
    elif lane_state == "warning":
        lane_text = f"变道判断：{side}实线，禁止{maneuver}并报警"
    elif lane_state == "allow":
        lane_text = f"变道判断：{side}虚线，允许{maneuver}"
    else:
        lane_text = f"变道判断：{side}线型未确认，暂不{maneuver}"

    if pedestrian_state == "stop":
        pedestrian_text = ("行人制动判断：紧急制动请求并报警" if values.get("person_on_crosswalk") == "1"
                           else "行人制动判断：短时保持紧急制动请求并报警")
    elif pedestrian_state == "confirming":
        pedestrian_text = "行人制动判断：疑似斑马线行人，正在二次确认"
    elif pedestrian_state == "clear":
        pedestrian_text = "行人制动判断：本帧未检测到本车路径内的斑马线行人"
    elif pedestrian_state == "idle":
        pedestrian_text = "行人制动判断：未启用"
    else:
        pedestrian_text = "行人制动判断：识别结果未就绪"

    if pedestrian_state == "stop":
        final = "综合建议：斑马线行人风险，紧急制动请求并报警"
    elif signal_action == "stop" and lane_action == "stop":
        final = "综合建议：待机"
    elif signal_action == "stop":
        final = "综合建议：" + lane_text.removeprefix("变道判断：")
    elif lane_action == "stop":
        final = "综合建议：" + signal_text.removeprefix("灯控判断：")
    elif signal_state == "stop":
        final = f"综合建议：{direction}红灯停车；不执行{maneuver}"
        if lane_state == "warning":
            final += "（实线报警）"
    elif signal_state == "idle":
        final = f"综合建议：灯控指令已过期，暂不执行{maneuver}"
    elif signal_state != "allow":
        final = f"综合建议：{direction}灯态未放行，暂不执行{maneuver}"
    elif lane_state == "allow":
        final = f"综合建议：{direction}可通行，并允许{maneuver}"
    elif lane_state == "warning":
        final = f"综合建议：{direction}可通行；{side}实线禁止{maneuver}（报警）"
    elif lane_state == "idle":
        final = f"综合建议：{direction}可通行；变道指令已过期"
    else:
        final = f"综合建议：{direction}可通行；{side}线型未确认，暂不{maneuver}"
    if signal_state == "allow" and lane_action != "stop" and values.get("signal_reason") == "right_turn_yield":
        final += "；右转须确认让行"
    return final, signal_seen, lane_seen, pedestrian_seen, signal_text, lane_text, pedestrian_text


def describe_latency(values: dict[str, str]) -> str:
    """Keep the latest processing breakdown visible without freshness ages."""
    def duration(key: str):
        try:
            value = float(values[key])
            return value if 0 <= value < 100000 else None
        except (KeyError, ValueError):
            return None

    def shown(value) -> str:
        return f"{value:.1f} ms" if value is not None else "暂无数据"

    signal_parts = [duration(key) for key in ("pre", "det_npu", "cls_npu")]
    signal_ms = sum(signal_parts) if all(part is not None for part in signal_parts) else None
    return (f"最近一次处理耗时：灯控核心合计 {shown(signal_ms)}"
            f"（预处理 {shown(signal_parts[0])}、检测推理 {shown(signal_parts[1])}、"
            f"灯态分类 {shown(signal_parts[2])}）\n"
            f"车道线检测 {shown(duration('lane_ms'))}    图像复制 {shown(duration('copy'))}")


def send_command(host: str, port: int, command: str, timeout: float = .3) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.send(command.encode("ascii"))
        return sock.recv(8192).decode("ascii", errors="replace")


def parse_status(response: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in response.split() if "=" in part)


class NetworkWorker:
    def __init__(self, host: str, port: int, timeout: float = .3):
        self.requests: queue.Queue[str] = queue.Queue(maxsize=16)
        self.events: queue.Queue[tuple[str, str]] = queue.Queue(maxsize=64)
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.host, self.port, self.timeout = host, port, timeout
        self.actions = {"signal": "stop", "lane": "stop"}
        self.revisions = {"signal": 0, "lane": 0}
        self.client = uuid.uuid4().hex[:16]
        self.sequence = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def configure(self, host: str, port: int) -> None:
        with self.lock:
            self.host, self.port = host, port
            for channel in self.actions:
                self.actions[channel] = "stop"
                self.revisions[channel] += 1

    def drive(self, action: str) -> None:
        if action not in ACTIONS:
            raise ValueError("Invalid driving action")
        channel = "lane" if action.startswith("lane_") else "signal"
        with self.lock:
            self.actions[channel] = action
            self.revisions[channel] += 1

    def submit(self, command: str) -> bool:
        try:
            self.requests.put_nowait(command)
            return True
        except queue.Full:
            return False

    def event(self, kind: str, value: str) -> None:
        try:
            self.events.put_nowait((kind, value))
        except queue.Full:
            try:
                self.events.get_nowait()
                self.events.put_nowait((kind, value))
            except queue.Empty:
                pass

    def transmit(self, host: str, port: int, command: str) -> None:
        try:
            response = send_command(host, port, command, self.timeout)
            self.event("status" if command == "STATUS" else "reply", response)
        except (OSError, UnicodeError) as exc:
            self.event("error", str(exc))

    def run(self) -> None:
        next_status = 0.0
        next_drive = {"signal": 0.0, "lane": 0.0}
        last_revision = {"signal": -1, "lane": -1}
        while not self.stopping.is_set():
            with self.lock:
                host, port = self.host, self.port
                actions, revisions = self.actions.copy(), self.revisions.copy()
            now = time.monotonic()
            for channel in ("signal", "lane"):
                action = actions[channel]
                if revisions[channel] != last_revision[channel] or (action != "stop" and now >= next_drive[channel]):
                    self.sequence += 1
                    command = f"DRIVE {action}" if channel == "signal" else f"LANE {action.removeprefix('lane_')}"
                    self.transmit(host, port, f"{command} {self.sequence} {self.client}")
                    last_revision[channel] = revisions[channel]
                    next_drive[channel] = time.monotonic() + .4
            try:
                command = self.requests.get_nowait()
            except queue.Empty:
                command = None
            if command is not None:
                self.transmit(host, port, command)
            elif time.monotonic() >= next_status:
                self.transmit(host, port, "STATUS")
                next_status = time.monotonic() + .4
            self.stopping.wait(.02)
        # Board command TTL also expires if this final datagram is lost.
        with self.lock:
            host, port = self.host, self.port
        try:
            send_command(host, port, f"DRIVE stop {self.sequence+1} {self.client}", .15)
            send_command(host, port, f"LANE stop {self.sequence+2} {self.client}", .15)
        except OSError:
            pass

    def close(self) -> None:
        self.stopping.set()


def run_gui(host: str, port: int, timeout: float) -> int:
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("智能驾驶辅助系统 · RK3568")
    root.geometry("940x900")
    root.minsize(800, 780)
    root.columnconfigure(0, weight=1)
    root.rowconfigure(4, weight=1)
    worker = NetworkWorker(host, port, timeout)
    host_var = tk.StringVar(value=host)
    port_var = tk.StringVar(value=str(port))
    message = tk.StringVar(value="连接中")
    result_var = tk.StringVar(value="综合建议：状态未知")
    signal_seen_var = tk.StringVar(value="红绿灯识别：未连接")
    pedestrian_seen_var = tk.StringVar(value="行人/斑马线识别：未连接")
    signal_result_var = tk.StringVar(value="灯控判断：未选择行驶方向")
    lane_result_var = tk.StringVar(value="变道判断：未选择变道方向")
    pedestrian_result_var = tk.StringVar(value="行人制动判断：等待板端数据")
    action_var = tk.StringVar(value="灯控意图：停止")
    lane_action_var = tk.StringVar(value="变道意图：停止")
    latency_var = tk.StringVar(value="最近一次处理耗时：等待板端数据")
    lane_var = tk.StringVar(value="车道线识别：未连接")
    lane_conf_var = tk.StringVar(value="")
    lane_detail_var = tk.StringVar(value="扫描线：100")
    model_var = tk.StringVar(value="检测：未加载    分类：未加载")
    det_var = tk.StringVar(value="fp")
    cls_var = tk.StringVar(value="fp")
    confirmed = tk.BooleanVar(value=True)
    lane_enabled = tk.BooleanVar(value=True)
    pedestrian_enabled = tk.BooleanVar(value=True)
    lane_debug = tk.BooleanVar(value=False)
    color_compare = tk.BooleanVar(value=False)
    view_order = tk.StringVar(value="rbg")
    roi_vars = [tk.StringVar(value=v) for v in ("0", "0", "1", "1")]
    params = {
        "infer_interval": ("推理间隔", "1"), "lane_hz": ("车道检测频率", "8"),
        "person_conf": ("行人阈值", "0.40"), "crosswalk_conf": ("斑马线阈值", "0.50"),
        "light_conf": ("交通灯阈值", "0.20"), "cls_conf": ("灯态阈值", "0.65"),
        "result_ttl_ms": ("灯态有效期/ms", "1000"), "lane_ttl_ms": ("车道有效期/ms", "900"),
        "pedestrian_hold_ms": ("制动请求保持/ms", "700"),
        "pedestrian_path_left": ("本车路径左界", "0.15"),
        "pedestrian_path_right": ("本车路径右界", "0.85"),
        "green_confirm": ("绿灯确认次数", "2"), "max_lights": ("最多分类灯数", "8"),
        "lane_top": ("道路区域上界", "0.58"), "lane_bottom": ("道路区域下界", "0.94"),
        "lane_x1": ("道路区域左界", "0"), "lane_x2": ("道路区域右界", "1"),
        "lane_contrast": ("白线候选局部亮度差", "12"),
        "lane_scan_rows": ("车道扫描线数", "100"),
        "lane_center": ("道路消失点横坐标先验", "0.50"),
        "lane_horizon": ("道路消失线纵坐标先验", "0.62"),
        "lane_vehicle_x": ("本车参考点横坐标", "0.50"),
        "lane_trap_tl": ("道路梯形上沿左端", "0.28"),
        "lane_trap_tr": ("道路梯形上沿右端", "0.72"),
        "lane_trap_bl": ("道路梯形下沿左端", "0.02"),
        "lane_trap_br": ("道路梯形下沿右端", "0.98"),
    }
    fields = {key: tk.StringVar(value=value) for key, (_, value) in params.items()}

    def submit(command: str) -> None:
        if not worker.submit(command):
            message.set("命令队列已满，请稍后重试")

    def connect() -> None:
        try:
            value = int(port_var.get())
            if not 1 <= value <= 65535 or not host_var.get().strip():
                raise ValueError()
            worker.configure(host_var.get().strip(), value)
            action_var.set("灯控意图：停止")
            lane_action_var.set("变道意图：停止")
            submit("PING")
        except ValueError:
            message.set("地址或端口无效")

    def drive(action: str) -> None:
        worker.drive(action)
        if action.startswith("lane_"):
            lane_action_var.set("变道意图：" + ACTIONS[action])
        else:
            action_var.set("灯控意图：" + ACTIONS[action])

    connection = ttk.Frame(root, padding=12)
    connection.grid(row=0, column=0, sticky="ew")
    ttk.Label(connection, text="开发板 IP").pack(side="left")
    ttk.Entry(connection, textvariable=host_var, width=22).pack(side="left", padx=8)
    ttk.Label(connection, text="端口").pack(side="left")
    ttk.Entry(connection, textvariable=port_var, width=7).pack(side="left", padx=8)
    ttk.Button(connection, text="连接", command=connect).pack(side="left")
    ttk.Label(connection, textvariable=message).pack(side="left", padx=12)

    status = ttk.Frame(root, padding=(12, 4))
    status.grid(row=1, column=0, sticky="ew")
    ttk.Label(status, textvariable=result_var, font=("Microsoft YaHei", 16, "bold"), wraplength=850).pack(anchor="w")
    ttk.Label(status, textvariable=signal_seen_var).pack(anchor="w", pady=(5, 0))
    ttk.Label(status, textvariable=lane_var).pack(anchor="w")
    ttk.Label(status, textvariable=pedestrian_seen_var).pack(anchor="w")
    ttk.Label(status, textvariable=signal_result_var).pack(anchor="w", pady=(5, 0))
    ttk.Label(status, textvariable=lane_result_var).pack(anchor="w")
    ttk.Label(status, textvariable=pedestrian_result_var).pack(anchor="w")
    ttk.Label(status, textvariable=lane_conf_var).pack(anchor="w")
    ttk.Label(status, textvariable=lane_detail_var).pack(anchor="w")
    ttk.Label(status, textvariable=latency_var, wraplength=900, justify="left").pack(anchor="w", pady=(4, 0))
    ttk.Label(status, textvariable=model_var).pack(anchor="w")

    driving = ttk.LabelFrame(root, text="单项或同时测试", padding=10)
    driving.grid(row=2, column=0, sticky="ew", padx=12, pady=8)
    for n, action in enumerate(("stop", "straight", "left", "right")):
        driving.columnconfigure(n, weight=1)
        ttk.Button(driving, text=ACTIONS[action], command=lambda a=action: drive(a)).grid(row=0, column=n, sticky="ew", padx=3)
    ttk.Label(driving, textvariable=action_var).grid(row=1, column=0, columnspan=4, sticky="w", pady=(4, 6))
    for n, action in enumerate(("lane_stop", "lane_left", "lane_right")):
        ttk.Button(driving, text=ACTIONS[action], command=lambda a=action: drive(a)).grid(row=2, column=n, sticky="ew", padx=3)
    ttk.Label(driving, textvariable=lane_action_var).grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))

    config = ttk.LabelFrame(root, text="运行配置", padding=10)
    config.grid(row=3, column=0, sticky="ew", padx=12, pady=4)
    for n in range(8):
        config.columnconfigure(n, weight=1)
    ttk.Label(config, text="检测模型").grid(row=0, column=0)
    ttk.Combobox(config, textvariable=det_var, values=("fp", "int8"), width=7, state="readonly").grid(row=0, column=1)
    ttk.Label(config, text="分类模型").grid(row=0, column=2)
    ttk.Combobox(config, textvariable=cls_var, values=("fp", "int8"), width=7, state="readonly").grid(row=0, column=3)
    ttk.Button(config, text="切换模型", command=lambda: (submit(f"MODEL det {det_var.get()}"), submit(f"MODEL cls {cls_var.get()}"))).grid(row=0, column=4)
    ttk.Button(config, text="辅助开启", command=lambda: submit("MODE assist")).grid(row=0, column=5)
    ttk.Button(config, text="辅助关闭", command=lambda: (drive("stop"), drive("lane_stop"), submit("MODE off"))).grid(row=0, column=6)
    ttk.Label(config, text="识别区域").grid(row=1, column=0, pady=8)
    ttk.Label(config, text="第一象限（左上）").grid(row=1, column=1, columnspan=3, sticky="w")
    ttk.Checkbutton(config, text="车道线检测", variable=lane_enabled,
                    command=lambda: submit(f"SET lane_enabled {int(lane_enabled.get())}")).grid(row=1, column=4, columnspan=2)
    ttk.Checkbutton(config, text="车道调试图", variable=lane_debug,
                    command=lambda: submit(f"SET lane_debug {int(lane_debug.get())}")).grid(row=1, column=6, columnspan=2)
    ttk.Checkbutton(config, text="行人紧急制动请求（可单独测试）", variable=pedestrian_enabled,
                    command=lambda: submit(f"SET pedestrian_enabled {int(pedestrian_enabled.get())}")).grid(
                        row=4, column=0, columnspan=8, sticky="w", pady=(4, 0))
    for n, var in enumerate(roi_vars):
        ttk.Label(config, text=("灯区 x1", "y1", "x2", "y2")[n]).grid(row=2, column=n*2)
        ttk.Entry(config, textvariable=var, width=7).grid(row=2, column=n*2+1)
    ttk.Button(config, text="应用灯区", command=lambda: (confirmed.set(False), submit("ROI " + " ".join(v.get() for v in roi_vars)))).grid(row=3, column=0, columnspan=2, pady=6)
    ttk.Checkbutton(config, text="已确认该区域为本车道车辆信号", variable=confirmed,
                    command=lambda: submit(f"SET signal_confirmed {int(confirmed.get())}")).grid(row=3, column=2, columnspan=6, sticky="w")

    tabs = ttk.Notebook(root)
    tabs.grid(row=4, column=0, sticky="nsew", padx=12, pady=8)
    parameter_page = ttk.Frame(tabs)
    tabs.add(parameter_page, text="算法参数")
    parameter_canvas = tk.Canvas(parameter_page, highlightthickness=0)
    parameter_scroll = ttk.Scrollbar(parameter_page, orient="vertical", command=parameter_canvas.yview)
    parameter_canvas.configure(yscrollcommand=parameter_scroll.set)
    parameter_scroll.pack(side="right", fill="y")
    parameter_canvas.pack(side="left", fill="both", expand=True)
    parameter_tab = ttk.Frame(parameter_canvas, padding=10)
    parameter_window = parameter_canvas.create_window((0, 0), window=parameter_tab, anchor="nw")
    parameter_tab.bind("<Configure>", lambda _: parameter_canvas.configure(scrollregion=parameter_canvas.bbox("all")))
    parameter_canvas.bind("<Configure>", lambda event: parameter_canvas.itemconfigure(parameter_window, width=event.width))
    for n, (key, (label, _)) in enumerate(params.items()):
        row, col = divmod(n, 2)
        ttk.Label(parameter_tab, text=label).grid(row=row, column=col*3, sticky="w", padx=4, pady=4)
        ttk.Entry(parameter_tab, textvariable=fields[key], width=10).grid(row=row, column=col*3+1, padx=4)
        ttk.Button(parameter_tab, text="应用", command=lambda k=key: submit(f"SET {k} {fields[k].get().strip()}")).grid(row=row, column=col*3+2, padx=6)
    for col in (0, 3):
        parameter_tab.columnconfigure(col, weight=1)
    hint_row = (len(params) + 1) // 2
    ttk.Label(parameter_tab,
              text="白线候选局部亮度差：默认 12，范围 3～60；越小越敏感，也更容易把反光当标线。此值不是虚实线置信度阈值。",
              wraplength=760).grid(row=hint_row, column=0, columnspan=6, sticky="w", pady=(12, 4))
    ttk.Label(parameter_tab,
              text="虚实线分类条件固定在板端：拟合置信度至少 0.55；实线覆盖率至少 0.88；虚线覆盖率 0.22～0.88，且需要足够的线段和间隔。",
              wraplength=760).grid(row=hint_row + 1, column=0, columnspan=6, sticky="w")
    ttk.Button(parameter_tab, text="整体应用本车路径", command=lambda: submit(
        "PEDESTRIAN_PATH " + " ".join(fields[key].get().strip() for key in
                                      ("pedestrian_path_left", "pedestrian_path_right")))).grid(
                                          row=hint_row + 2, column=0, columnspan=6, sticky="w", pady=(8, 0))
    diagnostic_tab = ttk.Frame(tabs, padding=12)
    tabs.add(diagnostic_tab, text="画面诊断")
    ttk.Label(diagnostic_tab, text="显示与识别通道").grid(row=0, column=0, sticky="w", pady=8)
    ttk.Combobox(diagnostic_tab, textvariable=view_order, values=("bgr", "rgb", "brg", "rbg", "gbr", "grb"),
                 state="readonly", width=10).grid(row=0, column=1, padx=8)
    ttk.Button(diagnostic_tab, text="应用通道", command=lambda: submit(f"SET camera_order {view_order.get()}")).grid(row=0, column=2, padx=8)
    ttk.Button(diagnostic_tab, text="恢复 RBG", command=lambda: (view_order.set("rbg"), submit("SET camera_order rbg"))).grid(row=0, column=3)
    ttk.Checkbutton(diagnostic_tab, text="板端显示六种通道对照", variable=color_compare,
                    command=lambda: submit(f"SET color_compare {int(color_compare.get())}")).grid(row=1, column=0, columnspan=3, sticky="w", pady=8)
    ttk.Button(diagnostic_tab, text="保存板端诊断帧", command=lambda: submit("SNAPSHOT")).grid(row=1, column=3)
    ttk.Button(diagnostic_tab, text="整体应用道路区域", command=lambda: submit("LANE_ROI " + " ".join(
        fields[key].get().strip() for key in ("lane_x1", "lane_top", "lane_x2", "lane_bottom")))).grid(row=2, column=0, columnspan=2, sticky="w", pady=8)
    ttk.Button(diagnostic_tab, text="整体应用道路梯形", command=lambda: submit("LANE_TRAP " + " ".join(
        fields[key].get().strip() for key in ("lane_trap_tl", "lane_trap_tr", "lane_trap_bl", "lane_trap_br")))).grid(row=2, column=2, columnspan=2, sticky="w", pady=8)
    log_tab = ttk.Frame(tabs)
    tabs.add(log_tab, text="通信日志")
    log = tk.Text(log_tab, height=10, wrap="word", font=("Consolas", 10))
    log.pack(fill="both", expand=True)

    def poll() -> None:
        for _ in range(32):
            try:
                kind, response = worker.events.get_nowait()
            except queue.Empty:
                break
            if kind == "status" and response.startswith("OK"):
                values = parse_status(response)
                if "signal_confirmed" in values:
                    confirmed.set(values["signal_confirmed"] == "1")
                if "pedestrian_enabled" in values:
                    pedestrian_enabled.set(values["pedestrian_enabled"] == "1")
                if "signal_state" in values and "lane_state" in values:
                    final, signal_seen, lane_seen, pedestrian_seen, signal_text, lane_text, pedestrian_text = describe_status(values)
                    if "pedestrian_state" not in values:
                        pedestrian_seen = "行人/斑马线识别：板端版本未提供"
                        pedestrian_text = "行人制动判断：请同步新版板端程序"
                    result_var.set(final)
                    signal_seen_var.set(signal_seen)
                    lane_var.set(lane_seen)
                    pedestrian_seen_var.set(pedestrian_seen)
                    signal_result_var.set(signal_text)
                    lane_result_var.set(lane_text)
                    pedestrian_result_var.set(pedestrian_text)
                else:
                    result_var.set("综合建议：板端版本不匹配")
                    signal_seen_var.set("红绿灯识别：板端旧版")
                    lane_var.set("车道线识别：板端旧版")
                    pedestrian_seen_var.set("行人/斑马线识别：板端旧版")
                    signal_result_var.set("灯控判断：板端未提供独立结果")
                    lane_result_var.set("变道判断：板端未提供独立结果")
                    pedestrian_result_var.set("行人制动判断：板端未提供独立结果")
                action_var.set("灯控意图：" + ACTIONS.get(values.get("action", "stop"), "停止"))
                lane_action_var.set("变道意图：" + ACTIONS.get(values.get("lane_action", "lane_stop"), "停止变道"))
                lane_conf_var.set(f"车道拟合置信度：左 {values.get('lane_left_conf', '-')} / "
                                  f"右 {values.get('lane_right_conf', '-')}")
                try:
                    paint = f"{100*float(values.get('lane_paint', '0')):.1f}%"
                    occlusion = f"{100*float(values.get('lane_occlusion', '0')):.1f}%"
                except ValueError:
                    paint = occlusion = "-"
                fitted = ["已拟合" if values.get(f"lane_{side}_fitted") == "1" else "未拟合"
                          for side in ("left", "right")]
                lane_detail_var.set(f"扫描线 {values.get('lane_actual_rows', '-')}/{values.get('lane_scan_rows', '-')} | 标线候选 {paint} | "
                                    f"遮挡 {occlusion} | 左侧{fitted[0]} / 右侧{fitted[1]}")
                latency_var.set(describe_latency(values))
                model_var.set(f"检测：{values.get('loaded_det', 'none')}    分类：{values.get('loaded_cls', 'none')}    "
                              f"识别通道 {values.get('infer_order', '-').upper()} / 第一象限")
                message.set("已连接" if values.get("version") == "ADAS-quad-V2.0.10"
                            else "板端版本不匹配，请同步 V2.0.10")
            else:
                if kind == "error":
                    message.set("连接异常")
                    result_var.set("连接异常，板端指令将过期")
                    signal_seen_var.set("红绿灯识别：连接异常")
                    lane_var.set("车道线识别：连接异常")
                    pedestrian_seen_var.set("行人/斑马线识别：连接异常")
                    signal_result_var.set("灯控判断：等待板端数据")
                    lane_result_var.set("变道判断：等待板端数据")
                    pedestrian_result_var.set("行人制动判断：等待板端数据")
                    latency_var.set("最近一次处理耗时：连接异常")
                elif response.startswith("ERR"):
                    message.set("板端拒绝指令：" + response[:60])
                log.insert("end", time.strftime("%H:%M:%S ") + response + "\n")
                if int(log.index("end-1c").split(".")[0]) > 300:
                    log.delete("1.0", "50.0")
                log.see("end")
        root.after(50, poll)

    def close() -> None:
        worker.close()
        worker.thread.join(timeout=.5)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    root.bind("<Escape>", lambda _: (drive("stop"), drive("lane_stop")))
    root.after(50, poll)
    root.mainloop()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.100")
    parser.add_argument("--port", type=int, default=9527)
    parser.add_argument("--timeout", type=float, default=.3)
    parser.add_argument("--command", help="ASCII board command; omit to open GUI")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not .05 <= args.timeout <= 5:
        parser.error("Port must be 1..65535 and timeout must be 0.05..5 seconds")
    if args.command:
        print(send_command(args.host, args.port, args.command, args.timeout))
        return 0
    return run_gui(args.host, args.port, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
