import argparse
import json
import logging
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal, QRectF, QPointF
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                               QLabel, QPushButton, QComboBox, QSplitter, QPlainTextEdit,
                               QTableWidget, QTableWidgetItem, QHeaderView, QFileDialog,
                               QMessageBox, QGraphicsView, QGraphicsScene, QFrame, QCheckBox, QDoubleSpinBox)

from geometry import load_calibration, estimate_pose, image_rotation, assumed_calibration, opposite_edge_lengths
from angle_filter import AngleAverage
from protocol import StreamParser, ScanRate, Snapshot


class SerialWorker(QThread):
    result = Signal(object)
    timing = Signal(object)
    cpu = Signal(object)
    frame = Signal(object)
    state = Signal(str)
    connected = Signal(bool)

    def __init__(self, port, upload_images=True):
        super().__init__()
        self.port = port
        self.upload_images = upload_images

    def run(self):
        parser = StreamParser()
        requested = False
        last_request = 0.0
        last_timing_request = 0.0
        timing_received = False
        previous_upload = None
        try:
            with serial.Serial(self.port, 921600, timeout=0.03, write_timeout=0.3) as connection:
                connection.reset_input_buffer()
                connection.write(b'@TIMINGON\r@GEOMETRYON\r@SUBPIXELON\r')
                last_timing_request = time.monotonic()
                self.connected.emit(True)
                self.state.emit(f'已连接 {self.port}，等待连续取图；按键固件需按设备按钮')
                try:
                    while not self.isInterruptionRequested():
                        now = time.monotonic()
                        if not timing_received and now - last_timing_request >= 1:
                            connection.write(b'@TIMINGON\r@GEOMETRYON\r@SUBPIXELON\r')
                            last_timing_request = now
                        upload = self.upload_images
                        if upload != previous_upload:
                            if not upload:
                                connection.write(b'@DEBUGOFF\r@TIMINGON\r')
                                last_timing_request = now
                                requested = False
                            previous_upload = upload
                        if requested and now - last_request > 2.5:
                            requested = False
                            parser.pending = None
                            parser.image.clear()
                            self.state.emit('图片请求未响应；请确认设备使用配套调试接口固件')
                        if upload and not requested and now - last_request >= 0.35:
                            connection.write(b'@SNAP\r')
                            requested = True
                            last_request = now
                        data = connection.read(min(max(connection.in_waiting, 1), 65536))
                        for kind, value in parser.feed(data):
                            if kind == 'result':
                                timing_received = value.get('decode_us') is not None
                                self.result.emit(value)
                            elif kind == 'timing':
                                timing_received = True
                                self.timing.emit(value)
                            elif kind == 'cpu':
                                self.cpu.emit(value)
                            elif kind == 'frame':
                                requested = False
                                self.frame.emit(value)
                            elif kind == 'waiting':
                                requested = False
                                if value == 'no_frame':
                                    self.state.emit('已连接，等待设备取图；无码帧也显示图像')
                            elif kind == 'error':
                                requested = False
                                logging.warning('Snapshot stream: %s', value)
                                self.state.emit('图片校验失败，已丢弃并重新请求')
                finally:
                    try:
                        connection.write(b'@DEBUGOFF\r@TIMINGOFF\r@GEOMETRYOFF\r@SUBPIXELOFF\r')
                    except serial.SerialException:
                        pass
        except (serial.SerialException, OSError) as error:
            logging.exception('Serial connection failed')
            self.state.emit(f'串口连接失败：{error}')
        finally:
            self.connected.emit(False)


def annotated_image(frame):
    if frame.coordinates_only:
        image = QImage(frame.width, frame.height, QImage.Format.Format_RGB32)
        image.fill(QColor('white'))
    else:
        image = QImage.fromData(frame.jpeg, 'JPEG')
    if image.isNull() or image.width() != frame.width or image.height() != frame.height:
        raise ValueError('图片尺寸或JPEG数据无效')
    image = image.convertToFormat(QImage.Format.Format_RGB32)
    if not frame.success:
        return image
    painter = QPainter(image)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    polygon = QPolygonF([QPointF(*point) for point in frame.corners])
    painter.setPen(QPen(QColor('#f59e0b' if frame.geometry_quality in (0, 1) else '#22d38a'), 4))
    painter.drawPolygon(polygon)
    for point in frame.corners:
        painter.setPen(QPen(QColor('#072b20'), 2))
        painter.setBrush(QColor('#f59e0b' if frame.geometry_quality in (0, 1) else '#35ed9c'))
        painter.drawEllipse(QPointF(*point), 6, 6)
    painter.end()
    return image


class ImageView(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.setBackgroundBrush(QColor('#111c2d'))
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setRenderHints(QPainter.RenderHint.SmoothPixmapTransform | QPainter.RenderHint.Antialiasing)
        self.item = None
        self.fitted = True
        self.setToolTip('滚轮缩放，按住鼠标拖动；“适应窗口”恢复完整图像')

    def show_image(self, image):
        if self.item is None:
            self.item = self.scene().addPixmap(QPixmap.fromImage(image))
        else:
            self.item.setPixmap(QPixmap.fromImage(image))
        self.scene().setSceneRect(QRectF(image.rect()))
        if self.fitted:
            self.fit()

    def fit(self):
        self.fitted = True
        if self.item:
            self.fitInView(self.item, Qt.AspectRatioMode.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.fitted:
            self.fit()

    def wheelEvent(self, event):
        self.fitted = False
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        current = self.transform().m11()
        if 0.1 <= current * factor <= 12:
            self.scale(factor, factor)


class DebugWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.worker = None
        self.timing_total = 0
        self.timing_success = 0
        self.rate = ScanRate()
        self.rate_end = self.rate.started
        self.frames_received = 0
        self.snapshot_success = 0
        self.snapshot_failure = 0
        self.image_size = (1280, 720)
        self.current = None
        self.latest = None
        self.latest_result = None
        self.rendered = None
        self.calibration = None
        self.pose = {}
        self.angle_average = AngleAverage()
        self.last_angle_render = 0
        self.last_cpu_at = 0
        self.records = deque(maxlen=30)
        self.last_key = None
        self.capture_folder = (Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[2] / 'dist') / 'captures'
        self.saved_frames = 0
        self.setWindowTitle('P4SCAN OV9281 扫码调试工具')
        self.resize(1460, 900)
        self.setMinimumSize(1160, 740)
        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(16, 12, 16, 12)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        outer.addWidget(splitter)
        left = QWidget()
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 10, 0)
        layout.setSpacing(6)
        title = QLabel('P4SCAN  /  OV9281 扫码调试')
        title.setObjectName('title')
        layout.addWidget(title)
        connection = QHBoxLayout()
        self.ports = QComboBox()
        self.ports.setMinimumWidth(230)
        connection.addWidget(QLabel('HS-USB 串口'))
        connection.addWidget(self.ports)
        refresh = QPushButton('刷新')
        refresh.clicked.connect(self.refresh_ports)
        connection.addWidget(refresh)
        self.connect_button = QPushButton('连接')
        self.connect_button.setObjectName('primary')
        self.connect_button.clicked.connect(self.toggle_connection)
        connection.addWidget(self.connect_button)
        self.image_button = QPushButton('关闭解码图片上传')
        self.image_button.clicked.connect(lambda: self.upload_images.toggle())
        connection.addWidget(self.image_button)
        self.status = QLabel('选择 HS-USB 串口，连接后显示 DataMatrix 结果和同帧快照')
        self.status.setWordWrap(True)
        layout.addLayout(connection)
        layout.addWidget(self.status)
        cards = QHBoxLayout()
        self.rolling = self.add_card(cards, '近60秒收到成功结果', '0')
        self.average = self.add_card(cards, '当前收到成功结果 · 次/秒', '—')
        self.total = self.add_card(cards, '本次累计成功次数', '0')
        self.decode = self.add_card(cards, '当前帧 · 解码耗时 ms', '—')
        layout.addLayout(cards)
        video_cards = QHBoxLayout()
        self.snapshot_count = self.add_card(video_cards, '同帧图片成功 / 失败', '0 / 0')
        self.timing_count = self.add_card(video_cards, 'CDC 扫描轮次 / 成功', '0 / 0')
        self.snapshot_total = self.add_card(video_cards, '本次同帧图片总数', '0')
        layout.addLayout(video_cards)
        self.rate_note = QLabel('按主机收到的成功结果计数；预览图按需采样，刷新频率不代表解码速度。')
        self.rate_note.setObjectName('muted')
        layout.addWidget(self.rate_note)
        toolbar = QHBoxLayout()
        self.upload_images = QCheckBox('上传解码帧图片（成功 / 失败）')
        self.upload_images.setChecked(not getattr(args, 'no_images', False))
        self.upload_images.setToolTip('关闭后用白底实时显示四角、解码内容和角度，继续接收时间戳。')
        self.upload_images.toggled.connect(self.upload_changed)
        toolbar.addWidget(self.upload_images)
        self.image_button.setText('关闭解码图片上传' if self.upload_images.isChecked() else '开启解码图片上传')
        toolbar.addWidget(QLabel('原图尺寸'))
        self.image_size_select = QComboBox()
        for width, height in ((1280, 720), (1024, 1024), (512, 512)):
            self.image_size_select.addItem(f'{width}×{height}', (width, height))
        self.image_size_select.setToolTip('关闭图片上传时选择固件的实际输出尺寸；收到图片后自动更新。')
        self.image_size_select.currentIndexChanged.connect(self.image_size_changed)
        toolbar.addWidget(self.image_size_select)
        self.pause = QCheckBox('冻结画面')
        self.pause.toggled.connect(self.pause_changed)
        toolbar.addWidget(self.pause)
        fit = QPushButton('适应窗口')
        fit.clicked.connect(lambda: self.view.fit())
        toolbar.addWidget(fit)
        toolbar.addStretch()
        layout.addLayout(toolbar)
        actions = QHBoxLayout()
        save = QPushButton('保存图片与结果')
        save.clicked.connect(self.save_current)
        actions.addWidget(save)
        offline = QPushButton('打开已保存样本')
        offline.clicked.connect(self.open_saved)
        actions.addWidget(offline)
        reset = QPushButton('清零统计')
        reset.clicked.connect(self.reset_rate)
        actions.addWidget(reset)
        actions.addStretch()
        layout.addLayout(actions)
        capture_bar = QHBoxLayout()
        self.auto_save = QCheckBox('自动保存收到的图片')
        self.auto_save.setChecked(False)
        self.auto_save.setToolTip('成功和失败图均保存为收到的原始JPEG及同帧JSON；冻结画面不停止保存。')
        self.auto_save.toggled.connect(self.auto_save_changed)
        capture_bar.addWidget(self.auto_save)
        capture_directory = QPushButton('选择保存目录')
        capture_directory.clicked.connect(self.choose_capture_folder)
        capture_bar.addWidget(capture_directory)
        self.capture_note = QLabel(str(self.capture_folder))
        self.capture_note.setWordWrap(True)
        capture_bar.addWidget(self.capture_note, 1)
        layout.addLayout(capture_bar)
        self.view = ImageView()
        layout.addWidget(self.view, 1)
        self.frame_info = QLabel('等待真实取图。图片与坐标来自同一个保留帧，CRC校验通过后才显示。')
        self.frame_info.setWordWrap(True)
        layout.addWidget(self.frame_info)
        splitter.addWidget(left)
        right = QWidget()
        right.setMinimumWidth(430)
        right.setMaximumWidth(570)
        panel = QVBoxLayout(right)
        panel.setContentsMargins(10, 0, 0, 0)
        panel.setSpacing(5)
        cpu_row = QHBoxLayout()
        self.cpu0 = QLabel('Core 0 占用：—')
        self.cpu1 = QLabel('Core 1 占用：—')
        for label in (self.cpu0, self.cpu1):
            label.setObjectName('cpu_usage')
            label.setToolTip('按每核空闲任务的运行时间计算，约 1 秒更新一次；包含采集、USB、ISP、解码等任务。')
            cpu_row.addWidget(label)
        panel.addLayout(cpu_row)
        panel.addWidget(QLabel('图中码制解码结果'))
        self.payload = QPlainTextEdit()
        self.payload.setReadOnly(True)
        self.payload.setMaximumHeight(74)
        panel.addWidget(self.payload)
        self.corners = QTableWidget(4, 3)
        self.corners.setHorizontalHeaderLabels(['原图角点', 'X', 'Y'])
        self.corners.verticalHeader().hide()
        self.corners.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.corners.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.corners.verticalHeader().setDefaultSectionSize(27)
        self.corners.horizontalHeader().setFixedHeight(29)
        self.corners.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.corners.setFixedHeight(142)
        for row, name in enumerate(['P0', 'P1', 'P2', 'P3']):
            self.corners.setItem(row, 0, QTableWidgetItem(name))
        panel.addWidget(self.corners)
        self.rotation = QLabel('画面内旋转角：—')
        self.front_angle = QLabel('正对偏角（3D）：未标定')
        self.agv_angle = QLabel('AGV与墙面偏航（有符号）：未标定')
        self.agv_angle.setWordWrap(True)
        self.agv_angle.setToolTip('相机水平且画面朝上，侧装90°。角度符号按相机坐标定义，转换为AGV车体偏航需校核左右安装外参。')
        self.pose_details = QLabel('正对为0°，偏斜时增大；此角度是光轴与码平面法线的夹角。')
        self.pose_details.setWordWrap(True)
        panel.addWidget(self.rotation)
        panel.addWidget(self.front_angle)
        panel.addWidget(self.agv_angle)
        panel.addWidget(self.pose_details)
        angle_options = QHBoxLayout()
        self.smooth_angles = QCheckBox('角度平滑')
        self.smooth_angles.setChecked(True)
        angle_options.addWidget(self.smooth_angles)
        self.angle_window = QDoubleSpinBox()
        self.angle_window.setRange(0.1, 2.0)
        self.angle_window.setSingleStep(0.1)
        self.angle_window.setValue(0.5)
        self.angle_window.setSuffix(' 秒平均')
        angle_options.addWidget(self.angle_window)
        panel.addLayout(angle_options)
        self.smooth_angles.setToolTip('仅平滑UI角度，约5Hz刷新；原始四角、设备结果与保存的原始姿态不变。平均会引入显示延迟。')
        self.smooth_angles.toggled.connect(self.refresh_pose)
        self.angle_window.valueChanged.connect(self.refresh_pose)
        calibration_buttons = QHBoxLayout()
        import_calibration = QPushButton('导入相机标定 JSON')
        import_calibration.clicked.connect(self.import_calibration)
        calibration_buttons.addWidget(import_calibration)
        clear_calibration = QPushButton('清除')
        clear_calibration.clicked.connect(self.clear_calibration)
        calibration_buttons.addWidget(clear_calibration)
        panel.addLayout(calibration_buttons)
        self.rough_pose = QCheckBox('未导入标定时，使用假定内参粗估')
        self.rough_pose.setChecked(True)
        panel.addWidget(self.rough_pose)
        fov_row = QHBoxLayout()
        fov_row.addWidget(QLabel('假定水平视场角'))
        self.assumed_fov = QDoubleSpinBox()
        self.assumed_fov.setRange(10, 150)
        self.assumed_fov.setValue(23)
        self.assumed_fov.setSuffix('°')
        fov_row.addWidget(self.assumed_fov)
        panel.addLayout(fov_row)
        self.rough_pose.toggled.connect(self.refresh_pose)
        self.assumed_fov.valueChanged.connect(self.refresh_pose)
        self.calibration_note = QLabel('默认粗估：假定水平视场角23°、主点居中、零畸变。导入真实标定后优先使用。')
        self.calibration_note.setWordWrap(True)
        self.calibration_note.setObjectName('muted')
        panel.addWidget(self.calibration_note)
        panel.addWidget(QLabel('最近图像记录（点击查看，最多30帧）'))
        self.history = QTableWidget(0, 2)
        self.history.setHorizontalHeaderLabels(['帧', '码制 / 状态'])
        self.history.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.history.setColumnWidth(0, 58)
        self.history.verticalHeader().hide()
        self.history.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.history.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.history.cellClicked.connect(self.select_history)
        panel.addWidget(self.history, 1)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([900, 520])
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_rate)
        self.timer.start(250)
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

    def add_card(self, layout, caption, value):
        card = QFrame()
        card.setObjectName('card')
        inner = QVBoxLayout(card)
        label = QLabel(caption)
        label.setObjectName('muted')
        number = QLabel(value)
        number.setObjectName('number')
        inner.addWidget(label)
        inner.addWidget(number)
        layout.addWidget(card)
        return number

    def refresh_ports(self):
        selected = self.ports.currentData()
        self.ports.clear()
        ports = sorted((port for port in list_ports.comports()
                        if port.vid == 0x303a and port.pid == 0x8001), key=lambda port: port.device)
        for port in ports:
            serial_number = port.serial_number or ''
            self.ports.addItem(f'{port.device}  {port.description}  {serial_number}', port.device)
        index = self.ports.findData(selected)
        if index >= 0:
            self.ports.setCurrentIndex(index)

    def toggle_connection(self):
        if self.worker:
            self.stop_worker()
            self.status.setText('已断开；最后一张图像保留供查看')
            return
        port = self.ports.currentData()
        if not port:
            self.status.setText('未发现串口，请连接扫码头的HS-USB口后刷新')
            return
        self.reset_rate()
        self.angle_average.reset()
        self.last_angle_render = 0
        self.rate_end = None
        self.worker = SerialWorker(port, self.upload_images.isChecked())
        self.worker.result.connect(self.receive_result)
        self.worker.timing.connect(self.receive_timing)
        self.worker.cpu.connect(self.receive_cpu)
        self.worker.frame.connect(self.receive_frame)
        self.worker.state.connect(self.status.setText)
        self.worker.connected.connect(self.connection_state)
        self.worker.start()
        self.connect_button.setText('断开')
        self.ports.setEnabled(False)

    def connection_state(self, connected):
        if self.sender() is not self.worker:
            return
        if not connected:
            self.cpu0.setText('Core 0 占用：—')
            self.cpu1.setText('Core 1 占用：—')
            self.last_cpu_at = 0
            self.rate_end = time.monotonic()
            self.connect_button.setText('连接')
            self.ports.setEnabled(True)
            if self.worker:
                self.worker.wait(1000)
                self.worker = None

    def stop_worker(self):
        if self.worker:
            self.worker.requestInterruption()
            if not self.worker.wait(2500):
                raise RuntimeError('串口线程未退出')
            self.worker = None
        self.rate_end = time.monotonic()
        self.cpu0.setText('Core 0 占用：—')
        self.cpu1.setText('Core 1 占用：—')
        self.last_cpu_at = 0
        self.connect_button.setText('连接')
        self.ports.setEnabled(True)

    def reset_rate(self):
        self.rate.reset()
        self.timing_total = self.timing_success = 0
        self.frames_received = 0
        self.snapshot_success = self.snapshot_failure = 0
        self.rate_end = None if self.worker else self.rate.started
        self.update_rate()

    def image_size_changed(self):
        size = self.image_size_select.currentData()
        if size is None:
            return
        self.image_size = size
        if self.current and self.current.coordinates_only:
            self.display_coordinates(self.latest_result)

    def update_rate(self):
        if self.last_cpu_at and time.monotonic() - self.last_cpu_at > 3:
            self.cpu0.setText('Core 0 占用：—')
            self.cpu1.setText('Core 1 占用：—')
            self.last_cpu_at = 0
        rolling, average, total, elapsed = self.rate.values(self.rate_end)
        self.rolling.setText(str(rolling))
        current = self.rate.current(self.rate_end)
        self.average.setText(f'{current / 60:.1f}' if current is not None else '—')
        self.total.setText(str(total))
        self.timing_count.setText(f'{self.timing_total} / {self.timing_success}')
        self.snapshot_count.setText(f'{self.snapshot_success} / {self.snapshot_failure}')
        self.snapshot_total.setText(str(self.frames_received))
        suffix = f'首分钟累计中：{min(elapsed, 60):.0f}/60秒' if elapsed < 60 else '完整滚动60秒窗口'
        self.rate_note.setText(f'{suffix} · 同帧图片按请求采样，成功和失败都显示；定位行仅在有效解码后发送。成功结果会话平均 {average / 60:.1f}次/秒。')
        self.rate_note.setWordWrap(True)

    def receive_timing(self, timing):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        self.timing_total += 1
        self.timing_success += int(timing['success'])
        if self.upload_images.isChecked() or self.pause.isChecked():
            return
        self.decode.setText(f"{timing['decode_us'] / 1000:.3f}")
        if not timing['success']:
            frame = Snapshot(timing['sequence'], timing['capture_us'], *self.image_size, False,
                             timing['decode_us'], [[0, 0]] * 4, b'', coordinates_only=True)
            self.display_frame(frame)
        self.status.setText(f"帧 #{timing['sequence']} · {'解码成功' if timing['success'] else '未解码'} · 图片上传关闭")

    def receive_cpu(self, usage):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        self.cpu0.setText(f"Core 0 占用：{usage['core0']:.1f}%")
        self.cpu1.setText(f"Core 1 占用：{usage['core1']:.1f}%")
        self.last_cpu_at = time.monotonic()

    def receive_result(self, result):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        self.rate.record()
        self.latest_result = result
        if not self.upload_images.isChecked() and not self.pause.isChecked():
            self.display_coordinates(result)
        logging.info('barcode %s corners=%s', result['payload'].decode('utf-8', errors='replace'), result['corners'])
        if 'age_us' in result:
            logging.info('barcode timing sequence=%d capture_us=%d device_age_ms=%.3f decode_us=%s',
                         result['sequence'], result['capture_us'], result['age_us'] / 1000, result.get('decode_us'))

    def receive_frame(self, frame):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        key = (frame.sequence, frame.timestamp_us)
        if key == self.last_key:
            return
        try:
            annotated_image(frame)
        except ValueError as error:
            self.status.setText(str(error))
            return
        self.last_key = key
        if self.auto_save.isChecked() and frame.jpeg and not frame.coordinates_only:
            self.save_received_frame(frame)
        self.latest = frame
        self.image_size = (frame.width, frame.height)
        index = self.image_size_select.findData((frame.width, frame.height))
        if index >= 0:
            self.image_size_select.setCurrentIndex(index)
        self.frames_received += 1
        if frame.success:
            self.snapshot_success += 1
        else:
            self.snapshot_failure += 1
        self.records.appendleft(frame)
        self.history.setRowCount(len(self.records))
        for row, record in enumerate(self.records):
            self.history.setItem(row, 0, QTableWidgetItem(str(record.sequence)))
            self.history.setItem(row, 1, QTableWidgetItem(record.text if record.success else '未解码'))
        if self.upload_images.isChecked() and not self.pause.isChecked():
            self.display_frame(frame)
        if self.upload_images.isChecked():
            self.status.setText('已收到同帧图片与结果' if frame.success else '已收到无解码结果的实际图像 · 正在继续搜索')

    def display_coordinates(self, result):
        result = result or {}
        frame = Snapshot(result.get('sequence', 0), result.get('capture_us', 0), *self.image_size,
                         bool(result), result.get('decode_us'), result.get('corners', [[0, 0]] * 4), result.get('payload', b''),
                         coordinates_only=True)
        for name in ('geometry_quality', 'geometry_edge_mask', 'geometry_support', 'geometry_residual'):
            if name in result:
                setattr(frame, name, result[name])
        self.display_frame(frame)

    def display_frame(self, frame):
        self.current = frame
        self.rendered = annotated_image(frame)
        self.view.setBackgroundBrush(QColor('white' if frame.coordinates_only else '#111c2d'))
        self.view.show_image(self.rendered)
        self.payload.setPlainText(frame.text if frame.success else '本帧无有效定位结果')
        self.decode.setText(f'{frame.decode_us / 1000:.3f}' if frame.decode_us is not None else '—')
        self.frame_info.setText(f'帧 #{frame.sequence} · {frame.width}×{frame.height} · 采集：启动后 {frame.timestamp_us / 1e6:.6f}s'
                               ' · 同帧关联 / JPEG校验通过')
        if frame.coordinates_only:
            self.frame_info.setText(f'白底定位示意 · {frame.width}×{frame.height}原图坐标 · 帧 #{frame.sequence}'
                                    f' · 采集时间戳 {frame.timestamp_us}µs · 非摄像头图像'
                                    + ('；固件未上传解码耗时' if frame.decode_us is None else '；含同帧解码耗时'))
            if not frame.success:
                self.payload.setPlainText('等待定位结果')
        for row in range(4):
            for column in range(2):
                value = str(frame.corners[row][column]) if frame.success else '—'
                self.corners.setItem(row, column + 1, QTableWidgetItem(value))
        angle = image_rotation(frame.corners) if frame.success else None
        calibration = self.calibration
        approximate = calibration is None and self.rough_pose.isChecked()
        if approximate:
            calibration = assumed_calibration(frame.width, frame.height, self.assumed_fov.value())
        pose_was_available = self.pose.get('available', False)
        self.pose = estimate_pose(frame.corners, frame.width, frame.height, calibration,
                                  max_reprojection_px=15 if approximate else 5) if frame.success else {'available': False, 'reason': '本帧无有效 DataMatrix 角点'}
        if frame.geometry_quality in (0, 1):
            self.pose = {'available': False, 'reason': '仅粗定位，精定位未通过质量检查，不用于AGV姿态估计'}
        if frame.geometry_quality is not None:
            quality = ('无定位', '粗定位', '精定位')[frame.geometry_quality]
            self.frame_info.setText(self.frame_info.text() + f' · {quality} · 边掩码 {frame.geometry_edge_mask:X} · 支持点 {frame.geometry_support} · 残差 {frame.geometry_residual:.3f}px')
        self.pose['calibration_source'] = 'assumed' if approximate else ('imported' if calibration else 'none')
        self.pose['assumed_horizontal_fov_deg'] = self.assumed_fov.value() if approximate else None
        if calibration is not None:
            self.pose['camera_matrix'] = calibration['camera_matrix'].tolist()
            self.pose['dist_coeffs'] = calibration['dist_coeffs'].tolist()
        now = time.monotonic()
        show_angles = (not self.smooth_angles.isChecked() or now - self.last_angle_render >= 0.2 or
                       not self.pose['available'] or not pose_was_available or
                       self.angle_average.identity != (frame.payload, frame.width, frame.height))
        display_angles = dict(self.pose)
        display_rotation = angle
        if self.pose['available']:
            self.pose['opposite_edges'] = opposite_edge_lengths(frame.corners)
            self.pose['agv_wall_yaw_deg'] = self.pose['horizontal_tilt_deg']
            self.pose['agv_wall_yaw_abs_deg'] = abs(self.pose['horizontal_tilt_deg'])
            values = {name: self.pose[name] for name in ('front_angle_deg', 'horizontal_tilt_deg', 'vertical_tilt_deg')}
            if angle is not None:
                values['image_rotation_deg'] = angle
            if self.smooth_angles.isChecked():
                values = self.angle_average.update((frame.payload, frame.width, frame.height),
                                                  frame.timestamp_us / 1e6 if frame.timestamp_us else now,
                                                  values, self.angle_window.value())
            display_angles.update(values)
            display_rotation = values.get('image_rotation_deg')
            self.pose['display_angles'] = values
            self.pose['display_average_seconds'] = self.angle_window.value() if self.smooth_angles.isChecked() else 0
        if show_angles:
            self.last_angle_render = now
            self.rotation.setText(f'画面内旋转角：{display_rotation:+.2f}°（顺时针为正）' if display_rotation is not None else '画面内旋转角：—')
        if self.pose['available'] and show_angles:
            self.pose['agv_assumptions'] = '相机水平且画面朝上、光轴垂直车身前进方向、DataMatrix 所在墙面竖直'
            self.agv_angle.setText(f'AGV与墙面偏航：{display_angles["horizontal_tilt_deg"]:+.2f}°（0°平行）\n法线向画面右偏为正、左偏为负；车体方向需校核。')
            self.front_angle.setText(f'正对偏角（3D大小）：{display_angles["front_angle_deg"]:.2f}°')
            if approximate:
                self.agv_angle.setText(self.agv_angle.text() + '\n【假定内参粗估，非标定测量】')
                self.front_angle.setText(self.front_angle.text() + '（粗估）')
            description = f'水平倾斜 {display_angles["horizontal_tilt_deg"]:+.2f}° / 垂直倾斜 {display_angles["vertical_tilt_deg"]:+.2f}°\n重投影误差 {self.pose["reprojection_px"]:.2f}px'
            edges = self.pose['opposite_edges']
            description += (f'\nP0–P3 {edges["p0_p3_px"]:.2f}px / P1–P2 {edges["p1_p2_px"]:.2f}px'
                            f'\n边长比 {edges["left_right_ratio"]:.5f}（原始值，非角度）')
            if approximate and self.pose['reprojection_px'] > 5:
                description += '\n低可信：假定内参与角点拟合偏差较大'
            if self.pose['optical_distance_mm'] is not None:
                description += f' / 沿光轴距离 {self.pose["optical_distance_mm"]:.1f}mm'
            if self.pose['ambiguous']:
                description += '\n平面双解接近：当前估计存在歧义，仅供调试'
            self.pose_details.setText(description)
        elif not self.pose['available']:
            self.agv_angle.setText('AGV与墙面偏航：不可计算（需要有效角点及匹配标定）')
            self.front_angle.setText('正对偏角（3D）：不可计算')
            self.pose_details.setText(self.pose['reason'])

    def upload_changed(self, enabled):
        self.image_button.setText('关闭解码图片上传' if enabled else '开启解码图片上传')
        if self.worker:
            self.worker.upload_images = enabled
        if not enabled:
            self.display_coordinates(self.latest_result)
        self.status.setText('图片上传已开启，等待新图' if enabled else '图片上传已关闭；白底实时显示四角并计算角度')

    def pause_changed(self, paused):
        if not paused:
            if not self.upload_images.isChecked():
                self.display_coordinates(self.latest_result)
            elif self.latest:
                self.display_frame(self.latest)

    def select_history(self, row, column):
        del column
        if row < len(self.records):
            self.pause.setChecked(True)
            self.angle_average.reset()
            self.last_angle_render = 0
            self.display_frame(self.records[row])

    def import_calibration(self):
        path, _ = QFileDialog.getOpenFileName(self, '导入真实相机标定', '', 'JSON (*.json)')
        if not path:
            return
        try:
            self.calibration = load_calibration(path)
            self.angle_average.reset()
            self.last_angle_render = 0
            self.calibration_note.setText(f'已加载：{Path(path).name}\n3D角度0°表示正对码面；角点和安装外参需实测校核。')
            if self.current:
                self.display_frame(self.current)
        except (ValueError, KeyError, OSError) as error:
            QMessageBox.warning(self, '标定格式错误', str(error))

    def refresh_pose(self, *_):
        self.angle_average.reset()
        self.last_angle_render = 0
        if self.current:
            self.display_frame(self.current)

    def clear_calibration(self):
        self.calibration = None
        self.angle_average.reset()
        self.last_angle_render = 0
        self.calibration_note.setText('未导入标定：勾选粗估可使用假定视场角、主点居中及零畸变；非实际相机标定。')
        if self.current:
            self.display_frame(self.current)

    def auto_save_changed(self, enabled):
        if enabled:
            self.upload_images.setChecked(True)
        self.capture_note.setText(f'已保存 {self.saved_frames} 张 · {self.capture_folder}')

    def choose_capture_folder(self):
        folder = QFileDialog.getExistingDirectory(self, '选择自动保存目录', str(self.capture_folder))
        if folder:
            self.capture_folder = Path(folder)
            self.capture_note.setText(f'已保存 {self.saved_frames} 张 · {self.capture_folder}')

    def save_received_frame(self, frame):
        try:
            self.capture_folder.mkdir(parents=True, exist_ok=True)
            received_at = datetime.now()
            base = f"{received_at:%Y%m%d_%H%M%S_%f}_frame_{frame.sequence}_{frame.timestamp_us}"
            image_file = f'{base}.jpg'
            (self.capture_folder / image_file).write_bytes(frame.jpeg)
            record = frame.metadata() | {'image_file': image_file, 'saved_at': received_at.isoformat()}
            (self.capture_folder / f'{base}.json').write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
            self.saved_frames += 1
            self.capture_note.setText(f'已保存 {self.saved_frames} 张 · {self.capture_folder}')
        except OSError as error:
            self.auto_save.setChecked(False)
            self.capture_note.setText(f'自动保存已停止：{error}')
            logging.exception('Automatic image save failed')

    def export_frame(self, folder):
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        base = f'frame_{self.current.sequence}_{self.current.timestamp_us}'
        image_file = None if self.current.coordinates_only else f'{base}.jpg'
        if image_file:
            (folder / image_file).write_bytes(self.current.jpeg)
        if not self.rendered.save(str(folder / f'{base}_annotated.png')):
            raise OSError('标注图保存失败')
        record = self.current.metadata() | {'image_file': image_file, 'pose': self.pose,
                                          'saved_at': datetime.now().isoformat(), 'rate': self.rate.values(self.rate_end)[:3]}
        (folder / f'{base}.json').write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')

    def save_current(self):
        if not self.current:
            self.status.setText('尚无同帧快照，请连接设备并开启上传图片')
            return
        folder = QFileDialog.getExistingDirectory(self, '选择保存目录')
        if folder:
            try:
                self.export_frame(folder)
                self.status.setText(f'已保存白底标注PNG和结果JSON：{folder}' if self.current.coordinates_only else
                                    f'已保存原始JPEG、标注PNG和结果JSON：{folder}')
            except OSError as error:
                QMessageBox.warning(self, '保存失败', str(error))

    def open_saved(self):
        path, _ = QFileDialog.getOpenFileName(self, '打开保存的结果JSON', '', 'JSON (*.json)')
        if not path:
            return
        try:
            record = json.loads(Path(path).read_text(encoding='utf-8-sig'))
            frame = Snapshot(record['sequence'], record['timestamp_us'], record['width'], record['height'],
                             record['success'], record['decode_us'], record['corners'], bytes.fromhex(record['payload_hex']),
                             b'' if record.get('coordinates_only', False) else (Path(path).parent / record['image_file']).read_bytes(),
                             coordinates_only=record.get('coordinates_only', False))
            for name in ('geometry_quality', 'geometry_edge_mask', 'geometry_support', 'geometry_residual'):
                if name in record:
                    setattr(frame, name, record[name])
            if len(frame.corners) != 4 or any(len(point) != 2 for point in frame.corners):
                raise ValueError('无效角点')
            self.pause.setChecked(True)
            self.angle_average.reset()
            self.last_angle_render = 0
            self.display_frame(frame)
            self.status.setText('正在查看离线样本，不计入成功速度统计')
        except (ValueError, KeyError, OSError, TypeError) as error:
            QMessageBox.warning(self, '样本读取失败', str(error))

    def finish_smoke(self):
        folder = Path(self.args.smoke_output)
        folder.mkdir(parents=True, exist_ok=True)
        self.grab().save(str(folder / 'window.png'))
        if self.current:
            self.export_frame(folder)
        (folder / 'smoke.json').write_text(json.dumps({'frames': self.frames_received, 'rate': self.rate.values(self.rate_end),
                                                      'timing_total': self.timing_total,
                                                      'snapshot_success': self.snapshot_success,
                                                      'snapshot_failure': self.snapshot_failure,
                                                      'timing_success': self.timing_success,
                                                      'decode_ms': self.decode.text(), 'coordinates_only': bool(self.current and self.current.coordinates_only),
                                                      'has_image': self.current is not None, 'pose': self.pose}, indent=2))
        self.close()

    def closeEvent(self, event):
        self.stop_worker()
        event.accept()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port')
    parser.add_argument('--auto-connect', action='store_true')
    parser.add_argument('--no-images', action='store_true')
    parser.add_argument('--smoke-seconds', type=float, default=0)
    parser.add_argument('--smoke-output', default='debugger_smoke')
    args = parser.parse_args()
    log_folder = Path.home() / 'AppData/Local/P4ScanOV9281Debugger'
    log_folder.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=log_folder / 'debugger.log', level=logging.INFO, encoding='utf-8',
                        format='%(asctime)s %(levelname)s %(message)s')
    logging.info('Starting timing-enabled debugger 2026-10-13 executable=%s', sys.executable)
    application = QApplication(sys.argv[:1])
    application.setFont(QFont('Microsoft YaHei UI', 10))
    application.setStyleSheet('''
        QWidget { background:#f3f6fb; color:#20334a; }
        QLabel#title { font-size:23px; font-weight:700; padding-bottom:8px; }
        QLabel#muted { color:#60738a; }
        QFrame#card { background:white; border:1px solid #dce5ef; border-radius:8px; }
        QFrame#card QLabel { background:white; }
        QLabel#number { font-size:29px; font-weight:700; color:#0a6db7; }
        QLabel#cpu_usage { color:#0a6db7; font-size:15px; font-weight:700; }
        QPushButton { background:white; border:1px solid #ccd8e6; border-radius:5px; padding:7px 12px; }
        QPushButton:hover { background:#e5f0ff; }
        QPushButton#primary { background:#136eb7; color:white; border-color:#136eb7; }
        QComboBox { background:white; padding:6px; border:1px solid #ccd8e6; border-radius:4px; }
        QPlainTextEdit, QTableWidget { background:white; border:1px solid #dce5ef; border-radius:4px; }
        QHeaderView::section { background:#eaf0f7; padding:5px; border:0; }
        QTableWidget::item:selected { background:#dbeaff; color:#173854; }
    ''')
    window = DebugWindow(args)
    window.show()
    return application.exec()


if __name__ == '__main__':
    raise SystemExit(main())
