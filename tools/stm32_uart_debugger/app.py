import argparse
import json
import math
import sys
import time
from collections import deque
from queue import Queue, Empty
from pathlib import Path

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal, QPointF
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPen, QPolygonF, QFont
from PySide6.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QLabel,
                               QMainWindow, QMessageBox, QPushButton, QSplitter,
                               QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
                               QPlainTextEdit, QHeaderView, QFrame, QGridLayout,
                               QCheckBox, QDialog, QFormLayout, QDoubleSpinBox, QFileDialog)

try:
    from build_version import BUILD_VERSION
except ImportError:
    BUILD_VERSION = 'development'

from pose_filter import PoseSmoother
from scene import AgvView
from uart_protocol import (parse_scan, parse_reply, CAL_NAMES, FACTORY_CAL,
                           validate_calibration, calibration_command)


def parse_pose_line(line: bytes) -> dict:
    return parse_scan(line)


class UartReader(QThread):
    line = Signal(str)
    result = Signal(object)
    state = Signal(str)
    reply = Signal(object)
    opened = Signal()

    def __init__(self, port, baud):
        super().__init__()
        self.port_name = port
        self.baud = baud
        self.commands = Queue()

    def send_command(self, command):
        self.commands.put(command.encode('ascii') + b'\r\n')

    def run(self):
        try:
            with serial.Serial(self.port_name, self.baud, timeout=0.05, write_timeout=.5) as port:
                self.state.emit(f'已连接 {self.port_name} · {self.baud} baud · 8N1')
                self.opened.emit()
                buffer = bytearray()
                while not self.isInterruptionRequested():
                    while not self.commands.empty():
                        try:
                            command = self.commands.get_nowait()
                        except Empty:
                            break
                        if port.write(command) != len(command):
                            raise OSError('串口命令发送不完整')
                    buffer.extend(port.read(min(max(port.in_waiting, 1), 4096)))
                    while b'\n' in buffer:
                        end = buffer.index(10)
                        raw = bytes(buffer[:end]).rstrip(b'\r')
                        del buffer[:end + 1]
                        if not raw:
                            continue
                        text = raw.decode('ascii', errors='replace')
                        self.line.emit(text)
                        try:
                            if raw.startswith((b'@CAL,', b'@OUT,')):
                                self.reply.emit(parse_reply(raw))
                            else:
                                self.result.emit(parse_pose_line(raw))
                        except (ValueError, UnicodeError) as error:
                            self.state.emit(f'收到非姿态行或格式错误：{error}')
                    if len(buffer) > 8192:
                        buffer.clear()
                        self.state.emit('串口行超过8192字节，已丢弃')
        except (serial.SerialException, OSError) as error:
            self.state.emit(f'串口错误：{error}')
        finally:
            self.state.emit('串口已断开')


class CalibrationDialog(QDialog):
    """Edit values locally; write only on an explicit download click."""
    def __init__(self, owner):
        super().__init__(owner)
        self.owner = owner
        self.pending = None
        self.setWindowTitle('相机内参 · 读取 / 下载至 P4 Flash')
        self.resize(540, 600)
        layout = QVBoxLayout(self)
        intro = QLabel('分辨率固定 1280×720。USB-UART TX 还需接 P4 GPIO42 RX。\n下载后立即生效并保存至 Flash，断电保留。')
        intro.setWordWrap(True)
        layout.addWidget(intro)
        form = QFormLayout()
        self.inputs = {}
        for name in CAL_NAMES[2:]:
            box = QDoubleSpinBox()
            box.setDecimals(9)
            if name in ('fx', 'fy'):
                box.setRange(100, 10000)
            elif name in ('cx', 'cy'):
                box.setRange(0, 1279.999999 if name == 'cx' else 719.999999)
            else:
                box.setRange(-10, 10)
            box.setSingleStep(.01 if name in ('fx', 'fy', 'cx', 'cy') else .0001)
            form.addRow(name, box)
            self.inputs[name] = box
        layout.addLayout(form)
        self.message = QLabel('请先读取设备当前内参。默认值仅供参考，尚未写入设备。')
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        buttons = QGridLayout()
        for i, (text, action) in enumerate((('读取 P4 当前内参', self.read_device),
                ('下载内参到 P4 Flash', self.download), ('导入 JSON', self.import_json),
                ('导出 JSON', self.export_json), ('填入工厂默认值', self.factory))):
            button = QPushButton(text)
            button.clicked.connect(action)
            buttons.addWidget(button, i//2, i%2)
            if i == 1:
                self.download_button = button
        layout.addLayout(buttons)
        self.timeout = QTimer(self)
        self.timeout.setSingleShot(True)
        self.timeout.timeout.connect(self.command_timeout)
        self.set_values(owner.calibration or FACTORY_CAL)

    def set_values(self, c):
        for name, box in self.inputs.items():
            box.setValue(c[name])

    def values(self):
        return validate_calibration(dict(width=1280, height=720,
            **{name: box.value() for name, box in self.inputs.items()}))

    def read_device(self):
        if self.pending:
            self.message.setText('请等待当前内参操作完成。')
            return
        if self.owner.send_command('@CAL,GET'):
            self.pending = 'read'
            self.timeout.start(3000)
            self.message.setText('正在读取设备当前内参…')

    def download(self):
        if self.pending:
            self.message.setText('请等待当前内参操作完成。')
            return
        try:
            c = self.values()
            if self.owner.send_command(calibration_command(c)):
                self.pending = c
                self.download_button.setEnabled(False)
                self.timeout.start(5000)
                self.message.setText('等待 P4 保存 Flash 后回读确认…')
        except ValueError as e:
            self.message.setText(str(e))

    def response(self, reply):
        if reply['kind'] == 'calibration':
            c = reply['values']
            if isinstance(self.pending, dict):
                match = all(math.isclose(c[k], self.pending[k], rel_tol=2e-6, abs_tol=2e-6) for k in CAL_NAMES)
                self.message.setText('下载成功：P4 已保存 Flash，并回读确认。' if match else '回读与下载值不一致，请重新读取核对。')
            else:
                self.message.setText('已读取 P4 当前生效内参。')
            self.set_values(c)
        elif reply['kind'] == 'error' and reply['command'] == '@CAL':
            self.message.setText('设备拒绝内参：' + reply['reason'])
        else:
            return
        self.timeout.stop()
        self.pending = None
        self.download_button.setEnabled(True)

    def command_timeout(self):
        self.pending = None
        self.download_button.setEnabled(True)
        self.message.setText('未收到确认，无法判断操作是否完成。请检查 TX→GPIO42 接线，再读取设备核对。')

    def factory(self):
        self.set_values(FACTORY_CAL)
        self.message.setText('已填入工厂值；点击下载才会写入 P4。')

    def import_json(self):
        filename, _ = QFileDialog.getOpenFileName(self, '导入相机内参', '', 'JSON (*.json)')
        if filename:
            try:
                c = json.loads(Path(filename).read_text(encoding='utf-8-sig'))
                if 'camera_matrix' in c:
                    k, d = c['camera_matrix'], c['dist_coeffs']
                    size = c['image_size']
                    c = dict(zip(CAL_NAMES, (size[0], size[1], k[0][0], k[1][1], k[0][2], k[1][2], *d)))
                self.set_values(validate_calibration(c))
                self.message.setText('已导入；点击下载才会写入 P4。')
            except (ValueError, KeyError, TypeError, IndexError, OSError) as e:
                self.message.setText('导入失败：' + str(e))

    def export_json(self):
        filename, _ = QFileDialog.getSaveFileName(self, '导出当前编辑内参', 'camera_calibration.json', 'JSON (*.json)')
        if filename:
            try:
                Path(filename).write_text(json.dumps(self.values(), indent=2)+'\n', encoding='utf-8')
                self.message.setText('已导出当前编辑值。')
            except (ValueError, OSError) as e:
                self.message.setText('导出失败：' + str(e))


class YawWaveform(QWidget):
    """Time plot of incoming yaw and its time-constant EMA filtered value."""
    WINDOW_SECONDS = 10.0

    def __init__(self):
        super().__init__()
        self.samples = deque(maxlen=1200)
        self.setMinimumHeight(190)
        self.setMaximumHeight(220)

    def add_sample(self, timestamp, raw_deg, filtered_deg):
        self.samples.append((timestamp, raw_deg, filtered_deg))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor('#ffffff'))
        painter.setPen(QColor('#183650'))
        painter.setFont(QFont('Microsoft YaHei UI', 10, QFont.Weight.DemiBold))
        painter.drawText(12, 21, '偏航角 yaw_y · 原始值与平滑值')
        painter.setFont(QFont('Microsoft YaHei UI', 8))
        painter.setPen(QColor('#64788c'))
        painter.drawText(12, 39, '时间窗口 10 s · 指数平滑时间常数 0.45 s')

        left, top, right, bottom = 52, 48, self.width() - 14, self.height() - 27
        if right <= left or bottom <= top:
            painter.end()
            return
        values = [v for sample in self.samples for v in sample[1:]]
        if values:
            low, high = min(values), max(values)
            center = (low + high) * .5
            half_span = max(10.0, (high - low) * .65)
            low, high = center - half_span, center + half_span
        else:
            low, high = -10.0, 10.0
        latest = self.samples[-1][0] if self.samples else time.monotonic()
        earliest = latest - self.WINDOW_SECONDS

        painter.setFont(QFont('Microsoft YaHei UI', 8))
        for fraction in range(5):
            y = top + (bottom - top) * fraction / 4
            value = high - (high - low) * fraction / 4
            painter.setPen(QPen(QColor('#e2e9ef'), 1))
            painter.drawLine(left, int(y), right, int(y))
            painter.setPen(QColor('#6d7f90'))
            painter.drawText(2, int(y + 4), f'{value:+.1f}°')
        for age in (0, 2.5, 5, 7.5, 10):
            x = right - (right - left) * age / self.WINDOW_SECONDS
            painter.setPen(QPen(QColor('#edf1f5'), 1))
            painter.drawLine(int(x), top, int(x), bottom)
            painter.setPen(QColor('#6d7f90'))
            painter.drawText(int(x - 11), self.height() - 7, f'-{age:g}s')

        def point(sample, index):
            x = left + (sample[0] - earliest) * (right - left) / self.WINDOW_SECONDS
            y = bottom - (sample[index] - low) * (bottom - top) / (high - low)
            return QPointF(x, y)

        for index, color, width in ((1, '#e27a2d', 1.5), (2, '#087bb5', 2.4)):
            visible = [sample for sample in self.samples if earliest <= sample[0] <= latest]
            if len(visible) >= 2:
                painter.setPen(QPen(QColor(color), width))
                painter.drawPolyline(QPolygonF([point(sample, index) for sample in visible]))

        legend_x = max(left + 8, right - 175)
        painter.setPen(QPen(QColor('#e27a2d'), 2))
        painter.drawLine(legend_x, top + 10, legend_x + 20, top + 10)
        painter.setPen(QColor('#52677b'))
        painter.drawText(legend_x + 25, top + 14, '原始')
        painter.setPen(QPen(QColor('#087bb5'), 2.5))
        painter.drawLine(legend_x + 70, top + 10, legend_x + 90, top + 10)
        painter.setPen(QColor('#52677b'))
        painter.drawText(legend_x + 95, top + 14, '平滑（用于 3D）')
        painter.end()


class MainWindow(QMainWindow):
    PARAMS = (('yaw_y_deg', '原始偏航角 yaw_y\n相机相对码面左右转角', '°'),
              ('pitch_x_deg', '俯仰角 pitch_x\n相机相对码面上下倾角', '°'),
              ('roll_z_deg', '横滚角 roll_z\n绕相机光轴的旋转角', '°'),
              ('range_m', '码中心直线距离 range_m\n相机光心到码中心的直线距离', 'm'),
              ('optical_depth_m', '光轴深度 optical_depth_m\n码中心沿相机光轴方向的深度', 'm'),
              ('distance_to_plane_m', '码平面垂直距离 distance_to_plane_m\n相机光心到码平面的垂直距离', 'm'),
              ('tx_m', '横向偏移 tx\n码中心相对光轴的左右偏移', 'm'),
              ('along_track_m', '沿过道偏移\n二维码相对相机，车前方为正', 'm'))

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.reader = None
        self.calibration = None
        self.calibration_dialog = None
        self.mode_timeout = QTimer(self)
        self.mode_timeout.setSingleShot(True)
        self.mode_timeout.timeout.connect(lambda: self.mode_status.setText('未确认模式：检查 TX→GPIO42 或固件版本'))
        self.received = 0
        self.last_content = ''
        self.pose_smoother = PoseSmoother(.45)
        self.rate_times = deque()
        self.setWindowTitle(f'UART 3D AGV 状态模拟器 · v{BUILD_VERSION}')
        self.resize(1440, 900)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        title = QLabel(f'P4 UART → STM32 接收模拟   ·   过道 AGV 3D 状态   ·   v{BUILD_VERSION}')
        title.setStyleSheet('font-size:22px;font-weight:700;color:#183650;padding:4px')
        layout.addWidget(title)
        bar = QHBoxLayout()
        bar.addWidget(QLabel('USB-UART 接收端口'))
        self.ports = QComboBox()
        self.ports.setMinimumWidth(320)
        bar.addWidget(self.ports)
        self.refresh = QPushButton('刷新')
        self.refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(self.refresh)
        bar.addWidget(QLabel('波特率'))
        self.baud = QComboBox()
        for rate in (921600, 460800, 230400, 115200):
            self.baud.addItem(str(rate), rate)
        bar.addWidget(self.baud)
        self.connect_button = QPushButton('连接接收')
        self.connect_button.clicked.connect(self.toggle_connection)
        bar.addWidget(self.connect_button)
        self.status = QLabel('P4 GPIO41 TX→转接器 RX；转接器 TX→P4 GPIO42 RX；共地。')
        bar.addWidget(self.status, 1)
        layout.addLayout(bar)

        controls = QHBoxLayout()
        self.full_upload = QCheckBox('上传完整数据（四角点、3 个角度、5 个距离）')
        self.full_upload.setChecked(False)
        self.full_upload.toggled.connect(self.request_output_mode)
        controls.addWidget(self.full_upload)
        self.mode_status = QLabel('未勾选：仅帧号、内容、偏航角、离墙距离、沿过道偏移')
        controls.addWidget(self.mode_status, 1)
        calibration_button = QPushButton('相机内参 · 读取 / 下载')
        calibration_button.clicked.connect(self.show_calibration)
        controls.addWidget(calibration_button)
        layout.addLayout(controls)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.values = {}
        grid = QGridLayout()
        for i, (key, label, unit) in enumerate(self.PARAMS):
            card = QFrame()
            card.setStyleSheet('QFrame{background:white;border:1px solid #d8e2ec;border-radius:7px}')
            card_layout = QVBoxLayout(card)
            caption = QLabel(label)
            caption.setStyleSheet('font-size:12px;font-weight:600;color:#41566b;border:0')
            caption.setWordWrap(True)
            value = QLabel('—')
            value.setStyleSheet('font-size:20px;font-weight:700;color:#086cac;border:0')
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            card_layout.addWidget(caption)
            card_layout.addWidget(value)
            grid.addWidget(card, i // 2, i % 2)
            self.values[key] = (value, unit)
        left_layout.addLayout(grid)
        self.content = QLabel('解码内容：等待数据')
        self.content.setStyleSheet('font-size:18px;font-weight:600;padding:8px;background:white;color:#086cac')
        left_layout.addWidget(self.content)
        self.frame_info = QLabel('帧号：—    角点：—')
        left_layout.addWidget(self.frame_info)
        self.history = QTableWidget(0, 6)
        self.history.setHorizontalHeaderLabels(['帧号', '内容', 'yaw (°)', 'tx (m)', '沿过道 (m)', '时间'])
        self.history.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        self.history.setColumnWidth(1, self.history.fontMetrics().horizontalAdvance('888888') + 14)
        for column, width in ((0, 52), (2, 68), (3, 68), (4, 83), (5, 70)):
            self.history.setColumnWidth(column, width)
        self.history.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.history.setMaximumHeight(260)
        left_layout.addWidget(QLabel('最近接收的解码结果'))
        left_layout.addWidget(self.history, 1)
        self.raw = QPlainTextEdit()
        self.raw.setReadOnly(True)
        self.raw.setMaximumBlockCount(1000)
        self.raw.setPlaceholderText('串口接收到的原始 CSV 行会显示在这里')
        left_layout.addWidget(QLabel('原始 UART 数据行'))
        left_layout.addWidget(self.raw, 1)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        self.simulation = AgvView()
        right_layout.addWidget(self.simulation, 1)
        self.yaw_waveform = YawWaveform()
        right_layout.addWidget(self.yaw_waveform)
        note = QLabel('左侧为串口原始值；3D 使用平滑姿态与沿过道偏移（时间常数 0.45 s）。沿过道偏移以固定墙面方向计算，原地绕相机光心旋转不会造成假位移。')
        note.setWordWrap(True)
        note.setStyleSheet('color:#52677e;padding:8px')
        right_layout.addWidget(note)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([540, 860])
        layout.addWidget(splitter, 1)
        self.rate = QLabel('有效结果：0    最近接收速率：— 次/秒')
        layout.addWidget(self.rate)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_rate)
        self.timer.start(250)
        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self.update_scene)
        self.animation_timer.start(33)
        self.refresh_ports()
        if args.port:
            index = self.ports.findData(args.port)
            if index < 0:
                self.ports.addItem(args.port, args.port)
                index = self.ports.count() - 1
            self.ports.setCurrentIndex(index)
        if args.auto_connect:
            QTimer.singleShot(300, self.toggle_connection)
        if args.smoke_seconds:
            QTimer.singleShot(int(args.smoke_seconds * 1000), self.finish_smoke)

    def refresh_ports(self):
        current = self.ports.currentData()
        self.ports.clear()
        for port in list_ports.comports():
            self.ports.addItem(f'{port.device}  {port.description}', port.device)
        index = self.ports.findData(current)
        if index >= 0:
            self.ports.setCurrentIndex(index)

    def toggle_connection(self):
        if self.reader and self.reader.isRunning():
            self.mode_timeout.stop()
            self.reader.requestInterruption()
            self.reader.wait(1000)
            self.reader = None
            self.connect_button.setText('连接接收')
            self.status.setText('串口已断开')
            return
        port = self.ports.currentData()
        if not port:
            QMessageBox.information(self, '选择串口', '请选择连接 P4 GPIO41 TX 的 USB-UART 接收端口。')
            return
        self.pose_smoother.reset()
        self.yaw_waveform.samples.clear()
        self.yaw_waveform.update()
        self.simulation.set_pose(None)
        self.reader = UartReader(port, self.baud.currentData())
        self.reader.line.connect(self.receive_line)
        self.reader.result.connect(self.receive_result)
        self.reader.state.connect(self.status.setText)
        self.reader.reply.connect(self.receive_reply)
        self.reader.opened.connect(self.connected)
        self.reader.start()
        self.connect_button.setText('断开接收')

    def send_command(self, command):
        if not self.reader or not self.reader.isRunning():
            self.status.setText('请先连接双向 UART 串口。')
            return False
        self.reader.send_command(command)
        return True

    def connected(self):
        self.request_output_mode()
        self.send_command('@CAL,GET')

    def request_output_mode(self, checked=None):
        mode = 'FULL' if self.full_upload.isChecked() else 'COMPACT'
        if self.send_command('@OUT,' + mode):
            self.mode_status.setText('正在请求' + ('完整' if mode == 'FULL' else '精简') + '上传…')
            self.mode_timeout.start(3000)
        else:
            self.mode_status.setText('连接后应用：' + ('完整上传' if mode == 'FULL' else '精简上传'))

    def receive_reply(self, reply):
        if reply['kind'] == 'output':
            desired = 'FULL' if self.full_upload.isChecked() else 'COMPACT'
            if reply['mode'] == desired:
                self.mode_timeout.stop()
                self.mode_status.setText(('完整' if desired == 'FULL' else '精简') + '上传已确认 · 整数 / CRC16')
        elif reply['kind'] == 'calibration':
            self.calibration = reply['values']
        elif reply['kind'] == 'error':
            self.status.setText('P4 命令错误：' + reply['reason'])
        if self.calibration_dialog:
            self.calibration_dialog.response(reply)

    def show_calibration(self):
        if self.calibration_dialog is None:
            self.calibration_dialog = CalibrationDialog(self)
        self.calibration_dialog.show()
        self.calibration_dialog.raise_()
        if self.reader and self.reader.isRunning():
            self.calibration_dialog.read_device()

    def receive_line(self, line):
        self.raw.appendPlainText(line)

    def receive_result(self, result):
        self.received += 1
        now = time.monotonic()
        self.rate_times.append(now)
        self.pose_smoother.set_target(result, now)
        filtered = self.pose_smoother.advance(now)
        if filtered is not None:
            self.yaw_waveform.add_sample(now, result['yaw_y_deg'], filtered['yaw_y_deg'])
        self.simulation.set_pose(filtered)
        self.last_content = result['payload']
        self.content.setText(f"解码内容：{result['payload']}")
        corners = str(result['corners']) if result['corners'] else '精简模式未上传'
        self.frame_info.setText(f"帧号：{result['sequence']}    角点 P0→P3：{corners}")
        for key, label, unit in self.PARAMS:
            value, suffix = self.values[key]
            number = result[key]
            value.setText('—' if number is None else f'{number:+.3f} {suffix}' if key in ('tx_m', 'along_track_m') else f'{number:.3f} {suffix}' if suffix == 'm' else f'{number:+.2f} {suffix}')
        row = self.history.rowCount()
        self.history.insertRow(row)
        cells = (str(result['sequence']), result['payload'],
                 '—' if result['yaw_y_deg'] is None else f"{result['yaw_y_deg']:+.2f}",
                 '—' if result['tx_m'] is None else f"{result['tx_m']:+.3f}",
                 '—' if result['along_track_m'] is None else f"{result['along_track_m']:+.3f}",
                 time.strftime('%H:%M:%S'))
        for column, text in enumerate(cells):
            item = QTableWidgetItem(text)
            if column == 1:
                item.setForeground(QColor('#086cac'))
                if len(text) > 6:
                    item.setToolTip(text)
            self.history.setItem(row, column, item)
        while self.history.rowCount() > 500:
            self.history.removeRow(0)
        self.history.scrollToBottom()

    def update_scene(self):
        filtered = self.pose_smoother.advance(time.monotonic())
        if filtered is not None:
            self.simulation.set_pose(filtered)

    def update_rate(self):
        now = time.monotonic()
        while self.rate_times and self.rate_times[0] < now - 5:
            self.rate_times.popleft()
        hz = len(self.rate_times) / min(5, max(1, now - self.rate_times[0])) if self.rate_times else None
        self.rate.setText(f'有效结果：{self.received}    最近接收速率：{hz:.1f} 次/秒' if hz is not None else f'有效结果：{self.received}    最近接收速率：—')

    def finish_smoke(self):
        from pathlib import Path
        folder = Path(self.args.smoke_output)
        folder.mkdir(parents=True, exist_ok=True)
        self.grab().save(str(folder / 'window.png'))
        (folder / 'smoke.json').write_text(json.dumps({
            'received': self.received,
            'history_rows': self.history.rowCount(),
            'last_content': self.last_content,
            'pose': self.simulation.pose,
            'status': self.status.text(),
        }, ensure_ascii=False, indent=2), encoding='utf-8')
        self.close()

    def closeEvent(self, event):
        if self.reader and self.reader.isRunning():
            self.reader.requestInterruption()
            self.reader.wait(1000)
        event.accept()


def configure_application(app):
    app.setStyle('Fusion')
    from PySide6.QtGui import QPalette
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor('#edf2f7'))
    palette.setColor(QPalette.ColorRole.WindowText, QColor('#243f54'))
    palette.setColor(QPalette.ColorRole.Base, QColor('#ffffff'))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor('#f3f6f9'))
    palette.setColor(QPalette.ColorRole.Text, QColor('#243f54'))
    palette.setColor(QPalette.ColorRole.Button, QColor('#edf2f7'))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor('#243f54'))
    app.setPalette(palette)
    app.setFont(QFont('Microsoft YaHei UI', 10))


def main():
    parser = argparse.ArgumentParser(description='P4→STM32 UART 调试、完整/精简上传与相机内参下载')
    parser.add_argument('--port')
    parser.add_argument('--auto-connect', action='store_true')
    parser.add_argument('--smoke-seconds', type=float, default=0)
    parser.add_argument('--smoke-output', default='uart_debugger_smoke')
    args = parser.parse_args()
    app = QApplication(sys.argv)
    configure_application(app)
    window = MainWindow(args)
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
