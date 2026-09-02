# -*- coding: utf-8 -*-
"""
风电示廓系统 RS-485 上位机（PySide6 重构版）
================================================

相对原 Tkinter 版的关键改动：
  1. 界面从"串口助手"升级为"上位机"形态：
     - 实时状态面板：温度 / 出光功率 / 激光器开关 / 控制模式 / 设备信息，收到回传即自动刷新；
     - 功能控制区：光源控制、功率/模式、固件升级 三个标签页，操作即下发；
     - 日志区：降级为纯记录（解析日志 / 原始帧两个标签页），不再承担状态显示。
  2. 自动查询机制：
     - 连接成功后定时器按周期自动下发「查询系统状态数据」帧，温度等状态持续刷新，无需手动点查询；
     - 设备序列号 / 固件版本只在连接成功后查询一次；
     - 控制模式在连接成功后查询一次，之后由设置命令的回传自动同步。
  3. 查询帧优先级最低：
     - 固件升级（准备 → 发数据 → 结束 → 跳转 APP）期间完全禁止查询帧；
     - 查询帧仅在无固件流程、距上次任意下发超过门控间隔时才发送，不影响数据帧接收与主动帧下发。
  4. 固件升级流程：
     选 .bin → 下发「固件升级准备」→ 设备复位进入 Bootloader（此时禁止查询）→ 逐帧下发固件数据
     → 下发「结束固件升级」（携带 CRC32 与大小）→ Bootloader 校验通过 → 回传跳转 APP 状态
     → 检测到 JUMPTOAPP / DONE 后自动恢复自动查询。

依赖：PySide6  +  pyserial
  pip install PySide6 pyserial

运行：python wind_laser_upper_pc_pyside6.py
"""
import os
import struct
import sys
import tempfile
import threading
import time
import zlib

from PySide6.QtCore import Qt, QThread, QTimer, Signal, QPointF
from PySide6.QtGui import QFont, QColor, QPixmap, QPainter, QPolygonF
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QLineEdit, QSpinBox, QCheckBox,
    QRadioButton, QButtonGroup, QFrame, QTabWidget, QPlainTextEdit,
    QSplitter, QGroupBox, QMessageBox, QFileDialog, QProgressBar,
    QStatusBar, QScrollArea,
)

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover
    serial = None
    list_ports = None

# ---------------------------------------------------------------------------
# 协议常量（与原版完全一致）
# ---------------------------------------------------------------------------
START_BYTE = 0x5A
DEVICE_TYPE = 0xFD

CMD_LIGHT_OFF = 0x00
CMD_LIGHT_ON = 0x01
CMD_SET_MODE = 0x02
CMD_GET_MODE = 0x03
CMD_SET_POWER = 0x04
CMD_GET_POWER = 0x05
CMD_GET_DEVICE_INFO = 0x10
CMD_GET_SYS_STATE = 0x23
CMD_FIRMWARE_PREPARE = 0x45

FW_DATA_CMD = 0x01
FW_FINISH_CMD = 0x02
FW_ABORT_CMD = 0x03
FW_CHUNK_SIZE = 128
FW_STATUS_TIMEOUT = 10.0

FIRMWARE_STATUS_TEXT = {
    'BOOTSTART': '刚进入 Bootloader 程序',
    'WAITFORBOOT': '正在等待固件升级',
    'JUMPTOAPP': '开始跳转到 APP 程序',
    'LEN_FAIL': '接收数据长度字节失败',
    'LEN_ERR': '固件数据长度错误',
    'DATA_FAIL': '接收固件数据错误',
    'WRITE_FAIL': '写入固件到 Flash 失败',
    'OK': '本帧固件数据成功写入 Flash',
    'CRC_FAIL': '读取 CRC32 校验和失败',
    'SIZE_FAIL': '读取固件数据大小失败',
    'SIZE_MISMATCH': '接收到的固件数据与下发数据大小不符',
    'DONE': '固件数据校验完成，接收数据与下发数据相符',
    'FAIL': '固件 CRC32 校验失败',
}
FIRMWARE_STATUS_WORDS = tuple(FIRMWARE_STATUS_TEXT)

BLADE_BITS = {
    '1号扇叶': 0x01,
    '2号扇叶': 0x02,
    '3号扇叶': 0x04,
    '全部扇叶': 0x07,
}
BLADE_ORDER = ['1号扇叶', '2号扇叶', '3号扇叶']

CONTROL_MODE_MAP = {
    0x00: '外触发控制模式',
    0x01: '串口常亮控制模式',
}
LIGHT_STATE_MAP = {
    0x00: '1,2,3号扇叶光源全关',
    0x01: '1号风机扇叶光源打开，其余光源关闭',
    0x02: '2号风机扇叶光源打开，其余光源关闭',
    0x03: '1,2号风机扇叶光源打开，3号光源关闭',
    0x04: '3号风机扇叶光源打开，其余光源关闭',
    0x05: '1,3号风机扇叶光源打开，2号光源关闭',
    0x06: '2,3号风机扇叶光源打开，1号光源关闭',
    0x07: '1,2,3号风机扇叶光源全打开',
}

# 查询帧（最低优先级）相对最近一次任意下发的门控间隔（秒）
QUERY_MIN_INTERVAL = 0.15


class WindLaserProtocol:
    """数据帧封装 / 解析 / 十六进制格式化（与原版一致）。"""

    @staticmethod
    def build_frame(cmd: int, payload: bytes = b'') -> bytes:
        frame = bytes([START_BYTE, DEVICE_TYPE, cmd, len(payload)]) + payload
        checksum = sum(frame) & 0xFF
        return frame + bytes([checksum])

    @staticmethod
    def decode_frame(frame: bytes):
        if len(frame) < 5:
            raise ValueError('帧长度不足')
        if frame[0] != START_BYTE:
            raise ValueError(f'帧头错误: {frame[0]:02X}')
        if frame[1] != DEVICE_TYPE:
            raise ValueError(f'设备类型错误: {frame[1]:02X}')
        cmd = frame[2]
        data_size = frame[3]
        expected_len = 5 + data_size
        if len(frame) != expected_len:
            raise ValueError(
                f'数据长度不匹配: expected={expected_len}, actual={len(frame)}, cmd=0x{cmd:02X}'
            )
        payload = frame[4:-1]
        checksum = frame[-1]
        if (sum(frame[:-1]) & 0xFF) != checksum:
            raise ValueError(
                f'校验和错误: calc=0x{sum(frame[:-1]) & 0xFF:02X}, recv=0x{checksum:02X}'
            )
        return cmd, payload

    @staticmethod
    def format_hex(data: bytes) -> str:
        return ' '.join(f'{b:02X}' for b in data)


# ---------------------------------------------------------------------------
# 串口读取线程：只负责读字节流并转发，所有解析/UI 更新都在主线程完成，
# 天然规避跨线程更新界面问题（这也是 PySide 相对 Tkinter 的强项）。
# ---------------------------------------------------------------------------
class SerialReaderThread(QThread):
    chunk_received = Signal(bytes)
    error_occurred = Signal(str)

    def __init__(self, serial_port, parent=None):
        super().__init__(parent)
        self.serial_port = serial_port
        self._running = True

    def run(self):
        while self._running and self.serial_port is not None and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting <= 0:
                    self.msleep(20)
                    continue
                chunk = self.serial_port.read(self.serial_port.in_waiting)
                if chunk:
                    self.chunk_received.emit(bytes(chunk))
            except Exception as exc:
                self.error_occurred.emit(f'串口读取异常：{exc}')
                break

    def stop(self):
        self._running = False


# ---------------------------------------------------------------------------
# 固件数据发送线程：逐帧下发，每帧等待板端回传状态字（'OK'）后发下一帧。
# 运行期间主线程定时器不会发送任何查询帧（见 _can_send_query）。
# ---------------------------------------------------------------------------
class FirmwareDataWorker(QThread):
    log = Signal(str)
    raw = Signal(str)
    progress = Signal(int, int)   # 当前帧号, 总帧数
    done = Signal(bool, str)      # 是否成功, 信息

    def __init__(self, window, parent=None):
        super().__init__(parent)
        self.window = window

    def run(self):
        total = len(self.window.firmware_data)
        frame_count = (total + FW_CHUNK_SIZE - 1) // FW_CHUNK_SIZE
        try:
            for offset in range(0, total, FW_CHUNK_SIZE):
                chunk = self.window.firmware_data[offset:offset + FW_CHUNK_SIZE]
                frame = bytes([FW_DATA_CMD, len(chunk)]) + chunk
                # 板端状态字由主线程在接收解析时写入并 set event
                self.window.firmware_status = ''
                self.window.firmware_status_event.clear()
                self.window.serial_port.write(frame)
                self.raw.emit(
                    f'发送固件数据 {offset + 1}-{offset + len(chunk)}/{total}：'
                    f'{WindLaserProtocol.format_hex(frame)}'
                )
                frame_number = offset // FW_CHUNK_SIZE + 1
                self.progress.emit(frame_number, frame_count)
                self.log.emit(f'已发送固件数据帧 {frame_number}，等待板端接收状态...')
                if not self.window.firmware_status_event.wait(FW_STATUS_TIMEOUT):
                    self.done.emit(
                        False,
                        f'固件数据帧 {frame_number} 等待回传状态超时，已停止继续发送。')
                    return
                if self.window.firmware_status != 'OK':
                    self.done.emit(
                        False,
                        f'固件数据帧 {frame_number} 未成功写入 Flash'
                        f'（状态：{self.window.firmware_status}），已停止继续发送。')
                    return
            self.done.emit(True, '')
        except Exception as exc:
            self.done.emit(False, f'固件数据发送失败：{exc}')


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('风电示廓系统 上位机')
        self.resize(1280, 820)
        self.setMinimumSize(1024, 680)

        # 串口 / 线程
        self.serial_port = None
        self.reader = None
        self.fw_worker = None

        # 接收缓冲（主线程维护）
        self.rx_buffer = bytearray()

        # 状态镜像：界面唯一权威状态源，所有回传解析写到这里，面板据此刷新
        self.device = {
            'temps': [None, None, None],     # 3 路温度 ℃
            'powers': [None, None, None],    # 3 路出光功率级数
            'lights': [None, None, None],    # 3 路激光器开关 True/False/None
            'light_state': None,             # 光源回传 0~7 编码
            'mode': None,                    # 控制模式字节
            'sn': '',                        # 序列号
            'fw': '',                        # 固件版本
        }

        # 固件升级状态机
        self.firmware_path = ''
        self.firmware_data = b''
        self.firmware_crc32 = 0
        self.firmware_size = 0
        self.firmware_phase = ''          # '' / 'prepare' / 'data' / 'finish' / 'abort'
        self.firmware_sending = False     # 固件数据发送中
        self.firmware_prepare_cooldown = False
        self.firmware_locked = False      # True 表示固件流程中，禁止任何查询帧
        self.firmware_status = ''
        self.firmware_status_event = threading.Event()

        # 查询帧优先级最低：记录最近一次任意下发的时间
        self.last_tx = 0.0

        # 自动轮询定时器（查询系统状态 → 温度等）
        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(1000)
        self.poll_timer.timeout.connect(self._poll)

        self._build_ui()
        self._refresh_ports()
        self._refresh_panel()
        self._set_firmware_ui_state()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        root.addWidget(self._build_connect_bar())

        # 状态显示 / 按钮控制 / 日志输出 三段放入同一 QSplitter，均可拖动调节大小
        main_splitter = QSplitter(Qt.Vertical)
        main_splitter.setChildrenCollapsible(False)

        main_splitter.addWidget(self._build_status_panel())

        tabs = QTabWidget()
        tabs.addTab(self._wrap_scroll(self._build_light_tab()), '光源控制')
        tabs.addTab(self._wrap_scroll(self._build_power_mode_tab()), '功率 / 模式')
        tabs.addTab(self._wrap_scroll(self._build_firmware_tab()), '固件升级')
        main_splitter.addWidget(tabs)

        log_tabs = QTabWidget()
        log_tabs.addTab(self._build_parsed_log_tab(), '解析日志')
        log_tabs.addTab(self._build_raw_log_tab(), '原始帧（调试）')
        main_splitter.addWidget(log_tabs)

        main_splitter.setSizes([300, 330, 180])
        main_splitter.setStretchFactor(0, 3)
        main_splitter.setStretchFactor(1, 3)
        main_splitter.setStretchFactor(2, 2)
        root.addWidget(main_splitter, 1)

        self.statusBar().showMessage('就绪')

    def _build_connect_bar(self):
        bar = QFrame()
        bar.setObjectName('topBar')
        bar.setFrameShape(QFrame.StyledPanel)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(8)

        lay.addWidget(QLabel('串口：'))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(140)
        lay.addWidget(self.port_combo)

        self.refresh_port_btn = QPushButton('刷新端口')
        self.refresh_port_btn.clicked.connect(self._refresh_ports)
        lay.addWidget(self.refresh_port_btn)

        lay.addSpacing(8)
        lay.addWidget(QLabel('波特率：'))
        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200', '230400'])
        self.baud_combo.setCurrentText('115200')
        lay.addWidget(self.baud_combo)

        lay.addSpacing(8)
        lay.addWidget(QLabel('数据位：'))
        self.databit_combo = QComboBox()
        self.databit_combo.addItems(['5', '6', '7', '8'])
        self.databit_combo.setCurrentText('8')
        lay.addWidget(self.databit_combo)

        lay.addWidget(QLabel('停止位：'))
        self.stopbit_combo = QComboBox()
        self.stopbit_combo.addItems(['1', '1.5', '2'])
        lay.addWidget(self.stopbit_combo)

        lay.addWidget(QLabel('校验：'))
        self.parity_combo = QComboBox()
        self.parity_combo.addItems(['N', 'E', 'O', 'M', 'S'])
        lay.addWidget(self.parity_combo)

        lay.addWidget(QLabel('流控：'))
        self.flowctrl_combo = QComboBox()
        self.flowctrl_combo.addItems(['N', 'RTS/CTS', 'XON/XOFF'])
        lay.addWidget(self.flowctrl_combo)

        lay.addStretch(1)

        self.conn_light = QLabel('●')
        self.conn_light.setFixedWidth(18)
        self.conn_light.setStyleSheet('color:#C0C0C0;font-size:16px;')
        self.conn_label = QLabel('未连接')
        lay.addWidget(self.conn_light)
        lay.addWidget(self.conn_label)

        self.connect_btn = QPushButton('打开串口')
        self.connect_btn.setMinimumWidth(100)
        self.connect_btn.clicked.connect(self._toggle_connection)
        lay.addWidget(self.connect_btn)
        return bar

    def _build_status_panel(self):
        """② 实时状态面板：绑定状态镜像，收到回传即自动刷新。
        左侧状态值卡片 + 右侧控制模式/设备信息，左右可拖动，右侧保证足够宽度。"""
        panel = QFrame()
        panel.setObjectName('statusPanel')
        panel_lay = QVBoxLayout(panel)
        panel_lay.setContentsMargins(0, 0, 0, 0)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        # ---- 左：状态值卡片（保留浅绿小框背景） ----
        left = QWidget()
        lay = QGridLayout(left)
        lay.setContentsMargins(12, 10, 4, 10)
        lay.setHorizontalSpacing(10)
        lay.setVerticalSpacing(8)

        lay.addWidget(self._section_title('温度（℃）'), 0, 0, alignment=Qt.AlignLeft)
        self.temp_labels = []
        temp_wrap = QHBoxLayout()
        temp_wrap.setSpacing(6)
        card, v = self._make_card('T1', '--')
        self.temp_labels.append(v)
        temp_wrap.addWidget(card, 1)
        temp_wrap.addStretch(1)
        lay.addLayout(temp_wrap, 1, 0)

        lay.addWidget(self._section_title('出光功率级数'), 2, 0, alignment=Qt.AlignLeft)
        self.power_labels = []
        power_wrap = QHBoxLayout()
        power_wrap.setSpacing(6)
        for i in range(3):
            card, v = self._make_card(f'P{i + 1}', '--')
            self.power_labels.append(v)
            power_wrap.addWidget(card, 1)
        lay.addLayout(power_wrap, 3, 0)

        lay.addWidget(self._section_title('激光器开关状态'), 4, 0, alignment=Qt.AlignLeft)
        self.light_labels = []
        light_wrap = QHBoxLayout()
        light_wrap.setSpacing(6)
        for i in range(3):
            card, info = self._make_light_card(f'{i + 1}号', None)
            self.light_labels.append(info)
            light_wrap.addWidget(card, 1)
        lay.addLayout(light_wrap, 5, 0)
        splitter.addWidget(left)

        # ---- 右：控制模式 / 设备信息 / 自动刷新（保证足够宽度，文字完整显示） ----
        right = QWidget()
        right.setMinimumWidth(300)
        rlay = QVBoxLayout(right)
        rlay.setContentsMargins(16, 10, 12, 10)
        rlay.setSpacing(6)

        rlay.addWidget(self._section_title('控制模式'))
        self.mode_label = QLabel('--')
        self.mode_label.setObjectName('bigValue')
        self.mode_label.setWordWrap(True)
        rlay.addWidget(self.mode_label)

        rlay.addSpacing(8)
        rlay.addWidget(self._section_title('设备信息'))
        info_box = QFrame()
        info_lay = QGridLayout(info_box)
        info_lay.setContentsMargins(0, 0, 0, 0)
        info_lay.setHorizontalSpacing(8)
        info_lay.setVerticalSpacing(6)
        info_lay.addWidget(QLabel('序列号：'), 0, 0, alignment=Qt.AlignLeft)
        self.sn_label = QLabel('--')
        self.sn_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        info_lay.addWidget(self.sn_label, 0, 1, alignment=Qt.AlignLeft)
        info_lay.addWidget(QLabel('固件版本：'), 1, 0, alignment=Qt.AlignLeft)
        self.fw_label = QLabel('--')
        self.fw_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        info_lay.addWidget(self.fw_label, 1, 1, alignment=Qt.AlignLeft)
        rlay.addWidget(info_box)

        rlay.addSpacing(8)
        self.auto_refresh_var = QCheckBox('自动刷新状态')
        self.auto_refresh_var.setChecked(True)
        self.auto_refresh_var.toggled.connect(self._on_auto_refresh_toggled)
        rlay.addWidget(self.auto_refresh_var)
        rlay.addStretch(1)
        splitter.addWidget(right)

        splitter.setSizes([700, 340])
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        panel_lay.addWidget(splitter)
        return panel

    def _wrap_scroll(self, widget):
        """把标签页内容包进滚动容器，保证窗口/区域较小时按钮也能滚动到、按得到。"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)
        scroll.setFrameShape(QFrame.NoFrame)
        return scroll

    # ---------- 光源控制 Tab ----------
    def _build_light_tab(self):
        box = QGroupBox('光源控制（选择扇叶后点击开/关）')
        lay = QVBoxLayout(box)

        self.light_blade_group = QButtonGroup(self)
        sel_wrap = QHBoxLayout()
        sel_wrap.setSpacing(8)
        for i, name in enumerate(BLADE_ORDER + ['全部扇叶']):
            rb = QRadioButton(name)
            if i == 0:
                rb.setChecked(True)
            self.light_blade_group.addButton(rb, list(BLADE_BITS.values())[i])
            sel_wrap.addWidget(rb)
        lay.addLayout(sel_wrap)

        btn_wrap = QHBoxLayout()
        btn_wrap.setSpacing(12)
        on_btn = QPushButton('开光源')
        on_btn.setObjectName('primaryBtn')
        on_btn.setMinimumWidth(120)
        on_btn.clicked.connect(lambda: self._send_light_command(CMD_LIGHT_ON))
        off_btn = QPushButton('关光源')
        off_btn.setObjectName('dangerBtn')
        off_btn.setMinimumWidth(120)
        off_btn.clicked.connect(lambda: self._send_light_command(CMD_LIGHT_OFF))
        btn_wrap.addWidget(on_btn)
        btn_wrap.addWidget(off_btn)
        btn_wrap.addStretch(1)
        lay.addLayout(btn_wrap)
        return box

    # ---------- 功率 / 模式 Tab ----------
    def _build_power_mode_tab(self):
        box = QGroupBox('功率设置')
        lay = QVBoxLayout(box)

        lay.addWidget(QLabel('目标扇叶：'))
        self.power_blade_group = QButtonGroup(self)
        blade_wrap = QHBoxLayout()
        blade_wrap.setSpacing(8)
        for i, name in enumerate(BLADE_ORDER + ['全部扇叶']):
            rb = QRadioButton(name)
            if i == 0:
                rb.setChecked(True)
            self.power_blade_group.addButton(rb, list(BLADE_BITS.values())[i])
            blade_wrap.addWidget(rb)
        lay.addLayout(blade_wrap)

        p_row = QHBoxLayout()
        p_row.addWidget(QLabel('功率级数（0~4095）：'))
        self.power_spin = QSpinBox()
        self.power_spin.setRange(0, 4095)
        self.power_spin.setValue(1000)
        p_row.addWidget(self.power_spin)
        p_row.addSpacing(10)
        send_power_btn = QPushButton('发送设置功率')
        send_power_btn.setObjectName('primaryBtn')
        send_power_btn.setMinimumWidth(140)
        send_power_btn.clicked.connect(self._send_power_command)
        p_row.addWidget(send_power_btn)
        p_row.addStretch(1)
        lay.addLayout(p_row)

        lay.addSpacing(6)
        lay.addWidget(QLabel('控制模式：'))
        mode_wrap = QHBoxLayout()
        mode_wrap.setSpacing(12)
        ext_btn = QPushButton('外触发控制模式')
        ext_btn.setMinimumWidth(160)
        ext_btn.clicked.connect(lambda: self._send_mode_command(0x00))
        serial_btn = QPushButton('串口常亮控制模式')
        serial_btn.setMinimumWidth(160)
        serial_btn.clicked.connect(lambda: self._send_mode_command(0x01))
        mode_wrap.addWidget(ext_btn)
        mode_wrap.addWidget(serial_btn)
        mode_wrap.addStretch(1)
        lay.addLayout(mode_wrap)
        return box

    # ---------- 固件升级 Tab ----------
    def _build_firmware_tab(self):
        box = QGroupBox('固件升级')
        lay = QVBoxLayout(box)

        file_row = QHBoxLayout()
        self.firmware_path_edit = QLineEdit()
        self.firmware_path_edit.setReadOnly(True)
        self.firmware_path_edit.setPlaceholderText('未选择固件文件')
        file_row.addWidget(self.firmware_path_edit, 1)
        browse_btn = QPushButton('选择 .bin')
        browse_btn.setMinimumWidth(120)
        browse_btn.clicked.connect(self._select_firmware)
        file_row.addWidget(browse_btn)
        lay.addLayout(file_row)

        self.firmware_info_label = QLabel('请选择固件文件')
        lay.addWidget(self.firmware_info_label)

        self.fw_progress = QProgressBar()
        self.fw_progress.setRange(0, 100)
        self.fw_progress.setValue(0)
        lay.addWidget(self.fw_progress)
        self.fw_progress_label = QLabel('0 / 0 帧')
        lay.addWidget(self.fw_progress_label)

        btn_grid = QGridLayout()
        btn_grid.setHorizontalSpacing(12)
        btn_grid.setVerticalSpacing(10)
        self.fw_prepare_btn = QPushButton('固件升级准备')
        self.fw_prepare_btn.setObjectName('blueBtn')
        self.fw_prepare_btn.setMinimumWidth(150)
        self.fw_prepare_btn.clicked.connect(self._send_firmware_prepare)
        self.fw_data_btn = QPushButton('发送固件数据')
        self.fw_data_btn.setObjectName('primaryBtn')
        self.fw_data_btn.setMinimumWidth(150)
        self.fw_data_btn.clicked.connect(self._start_firmware_data)
        self.fw_finish_btn = QPushButton('结束固件升级')
        self.fw_finish_btn.setObjectName('dangerBtn')
        self.fw_finish_btn.setMinimumWidth(150)
        self.fw_finish_btn.clicked.connect(self._send_firmware_finish)
        self.fw_abort_btn = QPushButton('中止固件升级')
        self.fw_abort_btn.setObjectName('warnBtn')
        self.fw_abort_btn.setMinimumWidth(150)
        self.fw_abort_btn.clicked.connect(self._send_firmware_abort)
        self.fw_unlock_btn = QPushButton('升级完成 / 恢复查询')
        self.fw_unlock_btn.setMinimumWidth(150)
        self.fw_unlock_btn.clicked.connect(self._unlock_firmware)
        btn_grid.addWidget(self.fw_prepare_btn, 0, 0)
        btn_grid.addWidget(self.fw_data_btn, 0, 1)
        btn_grid.addWidget(self.fw_finish_btn, 1, 0)
        btn_grid.addWidget(self.fw_abort_btn, 1, 1)
        btn_grid.addWidget(self.fw_unlock_btn, 2, 0, 1, 2)
        lay.addLayout(btn_grid)
        return box

    # ---------- 日志区 ----------
    def _build_parsed_log_tab(self):
        wrap = QWidget()
        lay = QVBoxLayout(wrap)
        lay.setContentsMargins(4, 4, 4, 4)
        head = QHBoxLayout()
        head.addStretch(1)
        clear_btn = QPushButton('清除日志')
        clear_btn.clicked.connect(lambda: self._clear_log(self.log_box))
        head.addWidget(clear_btn)
        lay.addLayout(head)
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(5000)
        lay.addWidget(self.log_box)
        return wrap

    def _build_raw_log_tab(self):
        wrap = QWidget()
        lay = QVBoxLayout(wrap)
        lay.setContentsMargins(4, 4, 4, 4)
        head = QHBoxLayout()
        head.addStretch(1)
        clear_btn = QPushButton('清除日志')
        clear_btn.clicked.connect(lambda: self._clear_log(self.raw_log_box))
        head.addWidget(clear_btn)
        lay.addLayout(head)
        self.raw_log_box = QPlainTextEdit()
        self.raw_log_box.setReadOnly(True)
        self.raw_log_box.setMaximumBlockCount(5000)
        self.raw_log_box.setFont(QFont('Courier New', 9))
        lay.addWidget(self.raw_log_box)
        return wrap

    # ---------------- UI 小工具 ----------------
    def _section_title(self, text):
        lab = QLabel(text)
        lab.setObjectName('sectionTitle')
        return lab

    def _make_card(self, title, value):
        """返回 (容器QFrame, 数值QLabel)。容器必须被布局持有，否则会被 GC 连带删除。"""
        card = QFrame()
        card.setObjectName('metricCard')
        card_lay = QVBoxLayout(card)
        card_lay.setContentsMargins(8, 6, 8, 6)
        card_lay.setSpacing(2)
        t = QLabel(title)
        t.setObjectName('cardTitle')
        v = QLabel(value)
        v.setObjectName('bigValue')
        card_lay.addWidget(t)
        card_lay.addWidget(v)
        return card, v

    def _make_light_card(self, title, state):
        card = QFrame()
        card.setObjectName('metricCard')
        card_lay = QVBoxLayout(card)
        card_lay.setContentsMargins(8, 6, 8, 6)
        card_lay.setSpacing(2)
        t = QLabel(title)
        t.setObjectName('cardTitle')
        dot = QLabel('●')
        dot.setAlignment(Qt.AlignCenter)
        state_label = QLabel('未知')
        state_label.setAlignment(Qt.AlignCenter)
        card_lay.addWidget(t)
        card_lay.addWidget(dot)
        card_lay.addWidget(state_label)
        return card, {'dot': dot, 'text': state_label}

    # ------------------------------------------------------------------ 串口
    def _refresh_ports(self):
        if serial is None or list_ports is None:
            self.port_combo.clear()
            self.port_combo.addItem('pyserial 未安装')
            self._log('未检测到 pyserial，无法访问串口设备。请执行：pip install pyserial')
            return
        ports = [port.device for port in list_ports.comports()]
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(ports if ports else ['无可用串口'])
        if current and current not in ('无可用串口',):
            self.port_combo.setCurrentText(current)

    def _toggle_connection(self):
        if serial is None:
            QMessageBox.warning(self, '依赖缺失',
                                '当前环境缺少 pyserial，请先安装：pip install pyserial')
            return
        if self.serial_port is not None and self.serial_port.is_open:
            self._close_serial()
            return
        port = self.port_combo.currentText().strip()
        if not port or port in ('无可用串口', 'pyserial 未安装'):
            QMessageBox.warning(self, '串口未选择', '请先选择有效串口或确认设备连接。')
            return
        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, '参数错误', '波特率必须为整数。')
            return
        try:
            databit = int(self.databit_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, '参数错误', '数据位必须为整数。')
            return
        stopbit_str = self.stopbit_combo.currentText()
        stopbit = 1.5 if stopbit_str == '1.5' else (2 if stopbit_str == '2' else 1)
        parity = self.parity_combo.currentText()
        flowctrl = self.flowctrl_combo.currentText()
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=databit,
                parity=parity,
                stopbits=stopbit,
                xonxoff=(flowctrl == 'XON/XOFF'),
                rtscts=(flowctrl == 'RTS/CTS'),
                timeout=0.1,
            )
        except Exception as exc:
            QMessageBox.critical(self, '串口打开失败', str(exc))
            self._log(f'串口打开失败：{exc}')
            return

        self.reader = SerialReaderThread(self.serial_port, self)
        self.reader.chunk_received.connect(self._on_chunk)
        self.reader.error_occurred.connect(self._on_serial_error)
        self.reader.start()

        self.connect_btn.setText('关闭串口')
        self.conn_light.setStyleSheet('color:#52C41A;font-size:16px;')
        self.conn_label.setText(f'已连接 {port}')
        self._log(f'串口已打开：{port}，配置：{baud}/{databit}/{stopbit_str}/{parity}/{flowctrl}')

        # 连接成功：启动自动轮询 + 一次性读取设备信息/模式
        if self.auto_refresh_var.isChecked() and not self.firmware_locked:
            self.poll_timer.start()
        self._initial_queries()

    def _close_serial(self):
        self.poll_timer.stop()
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait(2000)
            self.reader = None
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self.connect_btn.setText('打开串口')
        self.conn_light.setStyleSheet('color:#C0C0C0;font-size:16px;')
        self.conn_label.setText('未连接')
        self._log('串口已关闭。')

    def _on_serial_error(self, msg):
        self._log(msg)
        self._close_serial()

    def _serial_ready(self):
        if self.serial_port is None or not self.serial_port.is_open:
            QMessageBox.warning(self, '串口未连接', '请先打开串口再发送指令。')
            return False
        return True

    def _initial_queries(self):
        """连接后只各查询一次：设备信息（序列号/固件版本/初始功率）、控制模式、系统状态。"""
        if not self._serial_ready():
            return
        QTimer.singleShot(150, lambda: self._send_frame(CMD_GET_DEVICE_INFO))
        QTimer.singleShot(350, lambda: self._send_frame(CMD_GET_MODE))
        QTimer.singleShot(550, lambda: self._send_frame(CMD_GET_SYS_STATE))

    # ------------------------------------------------------------------ 接收
    def _on_chunk(self, chunk: bytes):
        self.rx_buffer.extend(chunk)
        while True:
            if not self.rx_buffer:
                break
            if self.rx_buffer[0] != START_BYTE:
                if self._consume_firmware_status():
                    continue
                if any(word.encode('ascii').startswith(bytes(self.rx_buffer))
                       for word in FIRMWARE_STATUS_WORDS):
                    break
                self.rx_buffer.pop(0)
                continue
            if len(self.rx_buffer) < 5:
                break
            data_size = self.rx_buffer[3]
            frame_len = 5 + data_size
            if len(self.rx_buffer) < frame_len:
                break
            frame = bytes(self.rx_buffer[:frame_len])
            del self.rx_buffer[:frame_len]
            self._handle_received_frame(frame)

    def _handle_received_frame(self, frame: bytes):
        self._raw_log(f'回传：{WindLaserProtocol.format_hex(frame)}')
        try:
            cmd, payload = WindLaserProtocol.decode_frame(frame)
        except Exception as exc:
            self._log(f'回传数据帧解析失败：{exc}')
            return
        self._log(f'回传数据帧校验通过，命令字：0x{cmd:02X}')
        self._parse_response(cmd, payload)

    def _consume_firmware_status(self):
        for status in FIRMWARE_STATUS_WORDS:
            status_bytes = status.encode('ascii')
            if self.rx_buffer.startswith(status_bytes):
                del self.rx_buffer[:len(status_bytes)]
                while self.rx_buffer[:1] in (b'\r', b'\n', b' '):
                    del self.rx_buffer[:1]
                description = FIRMWARE_STATUS_TEXT[status]
                self._log(f'固件升级状态：{status}，{description}')
                if self.firmware_phase == 'data':
                    self.firmware_status = status
                    self.firmware_status_event.set()
                # 检测到跳转 APP / 校验完成：稍后自动解锁并恢复自动查询
                if status in ('JUMPTOAPP', 'DONE'):
                    QTimer.singleShot(1500, self._unlock_firmware)
                return True
        return False

    # ------------------------------------------------------------------ 解析
    def _parse_response(self, cmd: int, payload: bytes):
        """回传帧解析：写入状态镜像并刷新面板（不再只打日志）。"""
        if cmd in (CMD_LIGHT_OFF, CMD_LIGHT_ON):
            state = payload[0] if payload else 0
            self.device['light_state'] = state
            for i in range(3):
                self.device['lights'][i] = bool(state & (1 << i))
            self._log(f'光源状态：{LIGHT_STATE_MAP.get(state, f"未知状态(0x{state:02X})")}')
            self._refresh_panel()
            return

        if cmd in (CMD_SET_MODE, CMD_GET_MODE):
            value = payload[0] if payload else 0
            self.device['mode'] = value
            self._log(f'控制模式：{CONTROL_MODE_MAP.get(value, f"未知模式(0x{value:02X})")}')
            self._refresh_panel()
            return

        if cmd in (CMD_SET_POWER, CMD_GET_POWER):
            if len(payload) >= 6:
                values = [
                    ((payload[0] << 8) | payload[1]),
                    ((payload[2] << 8) | payload[3]),
                    ((payload[4] << 8) | payload[5]),
                ]
                for i in range(3):
                    self.device['powers'][i] = values[i]
                self._log('功率级数：1号扇叶=' + str(values[0]) +
                          ', 2号扇叶=' + str(values[1]) +
                          ', 3号扇叶=' + str(values[2]))
                self._refresh_panel()
            else:
                self._log('功率回传数据长度不足。')
            return

        if cmd == CMD_GET_DEVICE_INFO:
            if len(payload) < 23:
                self._log('设备信息数据长度不足。')
                return
            power_values = [
                ((payload[0] << 8) | payload[1]),
                ((payload[2] << 8) | payload[3]),
                ((payload[4] << 8) | payload[5]),
            ]
            for i in range(3):
                self.device['powers'][i] = power_values[i]
            self.device['sn'] = payload[6:17].decode('ascii', errors='replace').rstrip('\x00')
            self.device['fw'] = payload[17:23].decode('ascii', errors='replace').rstrip('\x00')
            self._log('设备信息：1号扇叶功率=' + str(power_values[0]) +
                      ', 2号扇叶功率=' + str(power_values[1]) +
                      ', 3号扇叶功率=' + str(power_values[2]) +
                      ', 序列号=' + self.device['sn'] +
                      ', 固件版本=' + self.device['fw'])
            self._refresh_panel()
            return

        if cmd == CMD_GET_SYS_STATE:
            if len(payload) < 21:
                self._log('系统状态数据长度不足。')
                return
            temp1 = struct.unpack('<f', payload[0:4])[0]
            temp2 = struct.unpack('<f', payload[4:8])[0]
            temp3 = struct.unpack('<f', payload[8:12])[0]
            self.device['temps'] = [temp1, temp2, temp3]
            self.device['powers'] = [
                int.from_bytes(payload[12:14], 'little'),
                int.from_bytes(payload[14:16], 'little'),
                int.from_bytes(payload[16:18], 'little'),
            ]
            self.device['lights'] = [
                payload[18] == 0x01,
                payload[19] == 0x01,
                payload[20] == 0x01,
            ]
            self._log(
                '系统状态：T1=' + format(temp1, '.2f') + '℃, '
                'T2=' + format(temp2, '.2f') + '℃, '
                'T3=' + format(temp3, '.2f') + '℃; '
                'P1=' + str(self.device['powers'][0]) +
                ', P2=' + str(self.device['powers'][1]) +
                ', P3=' + str(self.device['powers'][2]) + '; '
                '状态=1号:' + ('开启' if self.device['lights'][0] else '关闭') +
                ', 2号:' + ('开启' if self.device['lights'][1] else '关闭') +
                ', 3号:' + ('开启' if self.device['lights'][2] else '关闭'))
            self._refresh_panel()
            return

        self._log(f'未定义命令：0x{cmd:02X}，数据={WindLaserProtocol.format_hex(payload)}')

    def _refresh_panel(self):
        """状态镜像 → 面板刷新（收到回传即调用，无需手动刷新）。"""
        d = self.device
        for i in range(3):
            p = d['powers'][i]
            self.power_labels[i].setText(str(p) if p is not None else '--')
            self._set_light_card(i, d['lights'][i])
        # 温度只显示 T1
        t = d['temps'][0]
        self.temp_labels[0].setText(f'{t:.2f} ℃' if t is not None else '--')
        mode = d['mode']
        self.mode_label.setText(
            CONTROL_MODE_MAP.get(mode, f'未知(0x{mode:02X})') if mode is not None else '--')
        self.sn_label.setText(d['sn'] if d['sn'] else '--')
        self.fw_label.setText(d['fw'] if d['fw'] else '--')

    def _set_light_card(self, i, state):
        card = self.light_labels[i]
        if state is None:
            card['dot'].setStyleSheet('color:#C0C0C0;font-size:16px;')
            card['text'].setText('未知')
        elif state:
            card['dot'].setStyleSheet('color:#52C41A;font-size:16px;')
            card['text'].setText('开')
        else:
            card['dot'].setStyleSheet('color:#EA6668;font-size:16px;')
            card['text'].setText('关')

    # ------------------------------------------------------------------ 发送
    def _send_frame(self, cmd: int, payload: bytes = b''):
        if not self._serial_ready():
            return False
        frame = WindLaserProtocol.build_frame(cmd, payload)
        try:
            self.serial_port.write(frame)
            self.last_tx = time.monotonic()
            self._raw_log(f'发送：{WindLaserProtocol.format_hex(frame)}')
            self._log(f'发送命令成功：0x{cmd:02X}')
            return True
        except Exception as exc:
            self._log(f'发送失败：{exc}')
            QMessageBox.critical(self, '发送失败', str(exc))
            return False

    def _can_send_query(self):
        """查询帧优先级最低：
        1) 固件升级（准备/数据/结束/中止）进行中 → 完全禁止；
        2) 距最近一次任意下发不足门控间隔 → 本轮跳过，避免影响数据帧接收与主动下发；
        3) 自动刷新开关关闭 → 不发送。
        """
        if self.firmware_locked or self.firmware_phase or self.firmware_sending:
            return False
        if self.firmware_prepare_cooldown:
            return False
        return (time.monotonic() - self.last_tx) >= QUERY_MIN_INTERVAL

    def _poll(self):
        """自动轮询：仅发送「查询系统状态数据」帧（用于温度等持续刷新）。"""
        if not self._serial_ready():
            return
        if not self.auto_refresh_var.isChecked():
            return
        if not self._can_send_query():
            return
        self._send_frame(CMD_GET_SYS_STATE)

    def _on_auto_refresh_toggled(self, checked):
        if checked and self.serial_port is not None and self.serial_port.is_open \
                and not self.firmware_locked:
            self.poll_timer.start()
        else:
            self.poll_timer.stop()

    # ---------- 光源 / 模式 / 功率 ----------
    def _send_light_command(self, cmd: int):
        if not self._serial_ready():
            return
        value = self.light_blade_group.checkedId()
        if value is None or value < 0:
            QMessageBox.warning(self, '参数错误', '请选择扇叶。')
            return
        self._send_frame(cmd, bytes([value]))

    def _send_mode_command(self, mode: int):
        self._send_frame(CMD_SET_MODE, bytes([mode]))

    def _send_power_command(self):
        if not self._serial_ready():
            return
        value = self.power_blade_group.checkedId()
        if value is None or value < 0:
            QMessageBox.warning(self, '参数错误', '请选择目标扇叶。')
            return
        level = self.power_spin.value()
        payload = bytes([value, (level >> 8) & 0xFF, level & 0xFF])
        self._send_frame(CMD_SET_POWER, payload)

    # ------------------------------------------------------------------ 固件
    def _select_firmware(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '选择固件文件', '', '固件文件 (*.bin);;所有文件 (*)')
        if not path:
            return
        try:
            with open(path, 'rb') as f:
                data = f.read()
        except OSError as exc:
            QMessageBox.critical(self, '固件读取失败', str(exc))
            return
        if not data:
            QMessageBox.warning(self, '固件文件无效', '固件文件为空，不能进行升级。')
            return
        self.firmware_path = path
        self.firmware_data = data
        self.firmware_size = len(data)
        self.firmware_crc32 = zlib.crc32(data) & 0xFFFFFFFF
        self.firmware_path_edit.setText(path)
        self.firmware_info_label.setText(
            f'大小：{self.firmware_size} 字节，CRC32：0x{self.firmware_crc32:08X}')
        self.fw_progress.setValue(0)
        self.fw_progress_label.setText('0 / 0 帧')
        self._log(f'已加载固件：{path}，大小 {self.firmware_size} 字节，'
                  f'CRC32=0x{self.firmware_crc32:08X}')

    def _send_firmware_prepare(self):
        """下发固件升级准备 → 设备复位进 Bootloader。此刻起锁定查询帧。"""
        if self.firmware_sending:
            self._log('固件数据正在发送，暂不能重新准备升级。')
            return
        if self.firmware_phase:
            self._log('固件升级流程进行中，请先完成或中止后再准备。')
            QMessageBox.information(self, '固件流程进行中',
                                    '固件升级流程进行中，请先点击“升级完成 / 恢复查询”'
                                    '或“中止固件升级”后再准备。')
            return
        if not self._serial_ready():
            return
        self.firmware_phase = 'prepare'
        self.firmware_locked = True          # 固件流程锁定：禁止查询帧
        self.poll_timer.stop()               # 直接停止轮询（双保险）
        self.firmware_prepare_cooldown = True
        self._set_firmware_ui_state()
        if self._send_frame(CMD_FIRMWARE_PREPARE, b''):
            self._log('固件升级准备已发送：设备将复位进入 Bootloader，已暂停自动查询。', blank=True)
            QTimer.singleShot(2000, self._finish_prepare_cooldown)

    def _finish_prepare_cooldown(self):
        self.firmware_prepare_cooldown = False
        if not self.firmware_sending:
            self._set_firmware_ui_state()
            self._log('固件升级准备完成，请及时发送固件数据。', blank=True)

    def _start_firmware_data(self):
        if not self.firmware_data:
            QMessageBox.warning(self, '未选择固件', '请先选择有效的 .bin 固件文件。')
            return
        if not self._serial_ready():
            return
        if self.firmware_prepare_cooldown:
            self._log('固件升级准备发送后需等待 2 秒，暂不能发送固件数据。')
            return
        if self.firmware_sending:
            self._log('固件数据正在发送，请等待当前传输完成。')
            return
        if self.firmware_phase not in ('prepare', ''):
            self._log('当前固件流程状态不允许发送固件数据。')
            return
        self.firmware_phase = 'data'
        self.firmware_sending = True
        self.firmware_status_event.clear()
        self._set_firmware_ui_state()

        self.fw_worker = FirmwareDataWorker(self, self)
        self.fw_worker.log.connect(self._log)
        self.fw_worker.raw.connect(self._raw_log)
        self.fw_worker.progress.connect(self._on_fw_progress)
        self.fw_worker.done.connect(self._on_fw_done)
        self.fw_worker.start()

    def _on_fw_progress(self, cur, total):
        self.fw_progress.setMaximum(max(total, 1))
        self.fw_progress.setValue(cur)
        self.fw_progress_label.setText(f'{cur} / {total} 帧')

    def _on_fw_done(self, ok, msg):
        self.firmware_sending = False
        self._set_firmware_ui_state()
        if ok:
            self._log('固件数据发送完成，请点击“结束固件升级”完成校验并跳转 APP。', blank=True)
        else:
            self._log(msg, blank=True)
            QMessageBox.critical(self, '固件数据发送失败', msg)

    def _send_firmware_finish(self):
        if not self.firmware_data:
            QMessageBox.warning(self, '未选择固件', '请先选择有效的 .bin 固件文件。')
            return
        if not self._serial_ready():
            return
        if self.firmware_sending:
            self._log('固件数据仍在发送，请等待发送完成后再结束。')
            return
        payload = (bytes([FW_FINISH_CMD]) +
                   self.firmware_crc32.to_bytes(4, 'little') +
                   self.firmware_size.to_bytes(4, 'little'))
        self.firmware_phase = 'finish'       # 保持锁定，等跳转 APP 状态字后解锁
        self._write_raw_firmware_frame(payload, '结束固件升级')

    def _send_firmware_abort(self):
        if not self._serial_ready():
            return
        self.firmware_phase = 'abort'
        self._write_raw_firmware_frame(bytes([FW_ABORT_CMD]), '中止固件升级')

    def _write_raw_firmware_frame(self, frame: bytes, description: str):
        try:
            self.serial_port.write(frame)
            self.last_tx = time.monotonic()
            self._raw_log(f'发送{description}：{WindLaserProtocol.format_hex(frame)}')
            self._log(f'{description}发送成功', blank=True)
        except Exception as exc:
            self._log(f'{description}发送失败：{exc}')
            QMessageBox.critical(self, '发送失败', str(exc))

    def _unlock_firmware(self):
        """固件升级流程结束（跳转 APP 成功 / 用户手动确认）后恢复自动查询。"""
        if self.firmware_sending:
            self._log('固件数据仍在发送，无法恢复查询。')
            return
        was_locked = self.firmware_locked
        self.firmware_phase = ''
        self.firmware_locked = False
        self.firmware_prepare_cooldown = False
        self._set_firmware_ui_state()
        if was_locked:
            self._log('固件升级流程结束，已恢复自动查询。', blank=True)
        if self.serial_port is not None and self.serial_port.is_open:
            if self.auto_refresh_var.isChecked():
                self.poll_timer.start()
            # 跳转回 APP 后重新读取一次设备信息与控制模式
            QTimer.singleShot(300, lambda: self._send_frame(CMD_GET_DEVICE_INFO))
            QTimer.singleShot(500, lambda: self._send_frame(CMD_GET_MODE))
            QTimer.singleShot(700, lambda: self._send_frame(CMD_GET_SYS_STATE))

    def _set_firmware_ui_state(self):
        busy = self.firmware_sending
        in_flow = bool(self.firmware_phase)
        # 固件升级准备 / 发送固件数据：仅在数据发送期间禁用，其余时间始终可点击，
        # 未选文件、未连接串口、流程状态不对等情况由点击后的弹窗/日志提示引导。
        self.fw_prepare_btn.setEnabled(not busy)
        self.fw_data_btn.setEnabled(not busy)
        self.fw_finish_btn.setEnabled(bool(self.firmware_data) and not busy and not self.firmware_prepare_cooldown)
        self.fw_abort_btn.setEnabled(not busy and (in_flow or self.firmware_locked))
        self.fw_unlock_btn.setEnabled(not busy and (in_flow or self.firmware_locked))

    # ------------------------------------------------------------------ 日志
    def _log(self, msg: str, blank: bool = False):
        ts = time.strftime('%H:%M:%S')
        self.log_box.appendPlainText(f'[{ts}] {msg}')
        if blank:
            self.log_box.appendPlainText('')
        self.statusBar().showMessage(f'最近事件：{msg}')

    def _raw_log(self, msg: str):
        ts = time.strftime('%H:%M:%S')
        self.raw_log_box.appendPlainText(f'[{ts}] {msg}')

    def _clear_log(self, box: QPlainTextEdit):
        box.clear()

    # ------------------------------------------------------------------ 关闭
    def closeEvent(self, event):
        self._close_serial()
        if self.fw_worker is not None and self.fw_worker.isRunning():
            self.fw_worker.wait(3000)
        event.accept()


# ---------------------------------------------------------------------------
# 样式（浅色工业风，工程现场/办公环境通用）
# ---------------------------------------------------------------------------
def _ensure_spin_arrow_icons():
    """生成 QSpinBox 上下箭头 PNG，返回 (up_path, down_path)。

    QSS 中 QSpinBox::up-arrow / down-arrow 的 image 需要文件路径，
    先尝试写入脚本同目录（打包后可写），不可写则回退到系统临时目录。
    """
    base = os.path.dirname(os.path.abspath(__file__))
    if not os.access(base, os.W_OK):
        base = tempfile.gettempdir()
    up_path = os.path.join(base, '_spin_up_arrow.png')
    down_path = os.path.join(base, '_spin_down_arrow.png')
    if os.path.exists(up_path) and os.path.exists(down_path):
        return up_path, down_path
    for path, dy in ((up_path, -1), (down_path, 1)):
        pm = QPixmap(16, 16)
        pm.fill(Qt.transparent)
        painter = QPainter(pm)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor('#4A4A46'))
        if dy < 0:
            painter.drawPolygon(QPolygonF([QPointF(3, 10), QPointF(8, 4), QPointF(13, 10)]))
        else:
            painter.drawPolygon(QPolygonF([QPointF(3, 6), QPointF(8, 12), QPointF(13, 6)]))
        painter.end()
        pm.save(path)
    return up_path, down_path


def apply_style(app: QApplication):
    app.setStyle('Fusion')
    up_arrow, down_arrow = _ensure_spin_arrow_icons()
    qss = """
        QMainWindow, QWidget { background: #F4F3EE; color: #1A1B1C; font-size: 13px; }
        QFrame#topBar { border: 1px solid #D8D8D2; border-radius: 8px; }
        QFrame#statusPanel { border: 1px solid #D8D8D2; border-radius: 8px; }
        QGroupBox { border: 1px solid #D8D8D2; border-radius: 8px;
                    margin-top: 10px; padding-top: 8px; font-weight: 600; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #1A1B1C; }
        QLabel#sectionTitle { font-weight: 600; color: #2E5B5B; }
        QLabel#bigValue { font-size: 20px; font-weight: 600; color: #1A1B1C; }
        QLabel#cardTitle { color: #6B7280; font-size: 11px; }
        QFrame#metricCard { background: #F0F8F7; border: 1px solid #CFE7E5; border-radius: 8px; }
        QComboBox, QLineEdit, QSpinBox, QPlainTextEdit {
            background: #FFFFFF; border: 1px solid #C9C9C2; border-radius: 6px;
            padding: 3px 6px; selection-background-color: #94D4D0; }
        QSpinBox { padding-right: 24px; }
        QSpinBox::up-button, QSpinBox::down-button {
            width: 18px; border: 1px solid #C9C9C2; background: #F4F3EE;
            subcontrol-origin: border; }
        QSpinBox::up-button { subcontrol-position: top right; border-top-right-radius: 5px; }
        QSpinBox::down-button { subcontrol-position: bottom right; border-bottom-right-radius: 5px; }
        QSpinBox::up-button:hover, QSpinBox::down-button:hover { background: #E7EFEE; }
        QSpinBox::up-arrow { image: url(__UP_ARROW__); width: 10px; height: 10px; }
        QSpinBox::down-arrow { image: url(__DOWN_ARROW__); width: 10px; height: 10px; }
        QPushButton {
            background: #FFFFFF; border: 1px solid #B9B9B2; border-radius: 6px;
            padding: 6px 14px; }
        QPushButton:hover { background: #ECF7F6; border-color: #6FBFBB; }
        QPushButton:pressed { background: #DCEFED; }
        QPushButton:disabled { color: #B0B0AA; background: #F0EFEA; }
        QPushButton#primaryBtn { background: #2F8E8B; color: #FFFFFF; border: none; font-weight: 600; }
        QPushButton#primaryBtn:hover { background: #3AA3A0; }
        QPushButton#dangerBtn { background: #C0504D; color: #FFFFFF; border: none; font-weight: 600; }
        QPushButton#dangerBtn:hover { background: #D0605C; }
        QPushButton#warnBtn { background: #C98A2D; color: #FFFFFF; border: none; font-weight: 600; }
        QPushButton#warnBtn:hover { background: #DA9A3C; }
        QPushButton#blueBtn { background: #2E6BE6; color: #FFFFFF; border: none; font-weight: 600; }
        QPushButton#blueBtn:hover { background: #3F7BF0; }
        QTabWidget::pane { border: 1px solid #D8D8D2; border-radius: 6px; }
        QTabBar::tab { background: #E8E8E2; padding: 6px 14px; border: 1px solid #D8D8D2;
                       border-bottom: none; border-top-left-radius: 6px; border-top-right-radius: 6px; }
        QTabBar::tab:selected { background: #FFFFFF; color: #1A1B1C; font-weight: 600; }
        QProgressBar { border: 1px solid #C9C9C2; border-radius: 6px; background: #F0EFEA;
                       text-align: center; height: 18px; }
        QProgressBar::chunk { background: #3F9C99; border-radius: 5px; }
        QStatusBar { background: #E8E8E2; color: #4A4A46; }
        QSplitter::handle { background: #E2E2DC; width: 7px; margin: 2px 0; border-radius: 3px; }
        QSplitter::handle:hover { background: #94D4D0; }
    """.replace('__UP_ARROW__', up_arrow).replace('__DOWN_ARROW__', down_arrow)
    app.setStyleSheet(qss)


def main():
    app = QApplication(sys.argv)
    apply_style(app)
    app.setApplicationName('风电示廓系统 上位机')
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
