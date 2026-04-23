import sys
import platform
import threading
from pathlib import Path
import yaml
import pyqtgraph
from PySide6.QtCore import QObject, QMetaObject, QThread, QTimer, Qt, Signal
from PySide6.QtGui import QIntValidator
from PySide6.QtWidgets import (
    QHBoxLayout,
    QApplication,
    QLabel,
    QLineEdit,
    QSlider,
    QVBoxLayout,
    QWidget,
)
import os

from src.read_fts3 import read_sensor_frame
from src.control_loop import HandController
from src.visualize_robot import RobotGLWidget
from src.scan_port import scan_id

UPDATE_HZ = 50
SENSOR_FREQ = 333
IR_FREQ = 333
PLOT_COLOR = (100, 100, 100)


class HandState(QObject):
    targetsChanged = Signal(list)
    presentChanged = Signal(list)
    currentChanged = Signal(list, list)
    irChanged = Signal(int)

    def __init__(self, joint_ids, is_left):
        super().__init__()
        self._joint_ids = list(joint_ids)
        self._joint_id_to_index = {
            joint_id: idx for idx, joint_id in enumerate(self._joint_ids)
        }
        self._targets = [0] * len(self._joint_ids)
        self._present_positions = [0] * len(self._joint_ids)
        self._currents = [0] * len(self._joint_ids)
        self._temperatures = [0] * len(self._joint_ids)
        self._ir_value = 0
        self._lock = threading.Lock()
        self.is_left = is_left

    def set_target(self, joint_id, target_position):
        idx = self._joint_id_to_index.get(joint_id)
        if idx is None:
            return
        with self._lock:
            self._targets[idx] = int(target_position)
            targets = list(self._targets)
        self.targetsChanged.emit(targets)

    def targets(self):
        with self._lock:
            return list(self._targets)

    def update_feedback(self, positions, currents, temperatures, ir_value):
        with self._lock:
            self._present_positions = list(positions)
            self._currents = list(currents)
            self._temperatures = list(temperatures)
            self._ir_value = int(ir_value)
            present = list(self._present_positions)
            currents_copy = list(self._currents)
            temps_copy = list(self._temperatures)
            ir_copy = self._ir_value
        self.presentChanged.emit(present)
        self.currentChanged.emit(currents_copy, temps_copy)
        self.irChanged.emit(ir_copy)


class LabeledSlider(QWidget):
    valueChanged = Signal(int)

    def __init__(self, name="Value", min_val=0, max_val=100, init_value=0):
        super().__init__()
        self.min_val = min_val
        self.max_val = max_val

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)

        self.name_label = QLabel(name)
        self.name_label.setAlignment(Qt.AlignLeft)
        self.name_label.setStyleSheet("font-weight: bold;")
        self.current_label = QLabel("current: --")
        self.temperature_label = QLabel("temp: --")

        row_layout = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(min_val, max_val)
        self.slider.setValue(init_value)

        self.input = QLineEdit(str(init_value))
        self.input.setFixedWidth(50)
        self.input.setValidator(QIntValidator(min_val, max_val))

        row_layout.addWidget(self.slider)
        row_layout.addWidget(self.input)
        row_layout.addWidget(self.current_label)
        row_layout.addWidget(self.temperature_label)

        main_layout.addWidget(self.name_label)
        main_layout.addLayout(row_layout)
        self.setLayout(main_layout)

        self.slider.valueChanged.connect(self._from_slider)
        self.input.editingFinished.connect(self._from_text)

    def update_current(self, current, temperature):
        if current is None:
            self.current_label.setText("current: --")
        else:
            self.current_label.setText(f"current: {int(current):04d}")
        if temperature is None:
            self.temperature_label.setText("temp: --")
        else:
            self.temperature_label.setText(f"temp: {int(temperature)}")

    def _from_slider(self, value):
        if self.input.text() != str(value):
            self.input.setText(str(value))
        self.valueChanged.emit(value)

    def _from_text(self):
        text = self.input.text()
        if text == "":
            return
        value = int(text)
        value = max(self.min_val, min(self.max_val, value))
        if self.slider.value() != value:
            self.slider.setValue(value)
        self.valueChanged.emit(value)


class HandControlWidget(QWidget):
    def __init__(self, config, hand_state, is_left):
        super().__init__()
        self.hand_state = hand_state
        self.layout = QVBoxLayout()
        self.sliders = []
        if is_left:
            mapping = config["l_joint_mapping"]
        else:
            mapping = config["r_joint_mapping"]
        
        
        for joint_name in mapping:
            if joint_name in ["Wrist Rotation", "Wrist Adduction", "Wrist Flexation"]:
                slider = LabeledSlider(joint_name, 0, 4095, 2048)
            else:
                slider = LabeledSlider(joint_name, 0, 4095, 0)

            joint_id = mapping[joint_name]
            slider.valueChanged.connect(
                lambda value, jid=joint_id: self.hand_state.set_target(jid, value)
            )
            self.layout.addWidget(slider)
            self.sliders.append(slider)

        self.layout.addStretch(1)
        self.setLayout(self.layout)

    def update_current(self, currents, temperatures):
        for slider, current, temperature in zip(self.sliders, currents, temperatures):
            slider.update_current(current, temperature)

    def init_hand(self):
        for slider in self.sliders:
            slider._from_text()


class IRLabel(QWidget):
    def __init__(self, name="IR"):
        super().__init__()
        self.layout = QVBoxLayout()
        self.name_label = QLabel(name)
        self.name_label.setStyleSheet("font-weight: bold;")
        self.layout.addWidget(self.name_label)
        self.plot_widget = pyqtgraph.PlotWidget()
        self.layout.addWidget(self.plot_widget)
        self.setLayout(self.layout)
        self.data = []
        self.x = []
        self.x_step = 1 / UPDATE_HZ
        self.window_points = 60 / self.x_step
        self.plot_widget.setBackground(PLOT_COLOR)
        plot_item = self.plot_widget.getPlotItem()
        plot_item.layout.setContentsMargins(0, 0, 0, 0)
        plot_item.getViewBox().setDefaultPadding(0)
        self.curve = self.plot_widget.plot(
            self.x, list(self.data), pen=pyqtgraph.mkPen("c", width=1)
        )
        self.plot_widget.setClipToView(True)
        self.plot_widget.setDownsampling(auto=True)

    def write_sensor_reading(self, value):
        self.data.append(value)
        self.x.append(len(self.data) * self.x_step)
        self.curve.setData(self.x, list(self.data))
        if self.x:
            x_max = self.x[-1]
            x_min = x_max - self.window_points * self.x_step
            view_x_min, view_x_max = self.plot_widget.getViewBox().viewRange()[0]
            if view_x_max >= x_max - (self.x_step * 2):
                self.plot_widget.setXRange(x_min, x_max, padding=0)


class SensorLabel(QWidget):
    def __init__(self, name="S"):
        super().__init__()
        self.layout = QVBoxLayout()
        self.name_label = QLabel(name)
        self.name_label.setStyleSheet("font-weight: bold;")
        self.layout.addWidget(self.name_label)
        self.x_step = SENSOR_FREQ / 1000
        self.plot_widget = pyqtgraph.PlotWidget()
        self.layout.addWidget(self.plot_widget)
        self.setLayout(self.layout)
        self.window_points = 60 / self.x_step

        self.data_x = []
        self.data_y = []
        self.data_z = []
        self.x = []

        plot_item = self.plot_widget.getPlotItem()
        plot_item.layout.setContentsMargins(0, 0, 0, 0)
        plot_item.getViewBox().setDefaultPadding(0)
        self.plot_widget.setClipToView(True)
        plot_item.addLegend()
        self.plot_widget.setBackground(PLOT_COLOR)

        self.curve_x = self.plot_widget.plot(
            self.x, list(self.data_x), pen=pyqtgraph.mkPen("c", width=1), name="Fx"
        )
        self.curve_y = self.plot_widget.plot(
            self.x, list(self.data_y), pen=pyqtgraph.mkPen("m", width=1), name="Fy"
        )
        self.curve_z = self.plot_widget.plot(
            self.x, list(self.data_z), pen=pyqtgraph.mkPen("y", width=1), name="Fz"
        )

        self.plot_widget.setDownsampling(auto=True)

    def write_sensor_reading(self, sensor):
        if sensor is None:
            return
        self.data_x.append(sensor.fx)
        self.data_y.append(sensor.fy)
        self.data_z.append(sensor.fz)
        self.x.append(len(self.data_x) * self.x_step)
        self.curve_x.setData(self.x, list(self.data_x))
        self.curve_y.setData(self.x, list(self.data_y))
        self.curve_z.setData(self.x, list(self.data_z))
        if self.x:
            x_max = self.x[-1]
            x_min = x_max - self.window_points * self.x_step
            view_x_min, view_x_max = self.plot_widget.getViewBox().viewRange()[0]
            if view_x_max >= x_max - (self.x_step * 2):
                self.plot_widget.setXRange(x_min, x_max, padding=0)


class SensorWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.sensor_labels = []
        self.layout = QHBoxLayout()
        for sensor_name in ["Thumb FTS3 Sensor", "Index FTS3 Sensor", "Middle FTS3 Sensor", "Ring FTS3 Sensor", "Little FTS3 Sensor"]:
            label = SensorLabel(sensor_name)
            self.sensor_labels.append(label)
            self.layout.addWidget(label)
        self.ir_label = IRLabel("Palm IR Sensor")
        self.layout.addWidget(self.ir_label)
        self.setLayout(self.layout)

    def update_IR_label(self, value):
        self.ir_label.write_sensor_reading(value)

    def update_from_data(self, data):
        if data and len(data) == len(self.sensor_labels):
            for idx, sensor_label in enumerate(self.sensor_labels):
                sensor_label.write_sensor_reading(data[idx])


class SensorReader(QThread):
    dataReady = Signal(list)
    def __init__(self, port, baudrate, interval_ms=100, parent=None):
        super().__init__(parent)
        self.interval_ms = max(10, int(interval_ms))
        self._running = True
        self.port = port
        self.baudrate = baudrate

    def run(self):
        while self._running:
            data = read_sensor_frame(self.port)[1]
            if data:
                self.dataReady.emit(data)
            self.msleep(self.interval_ms)

    def stop(self):
        self._running = False


class HandIOWorker(QObject):
    feedbackReady = Signal(list, list, list, int)
    error = Signal(str)
    stopRequested = Signal()

    def __init__(self, config_path, hand_state, update_hz=20, parent=None):
        super().__init__(parent)
        self._config_path = config_path
        self._hand_state = hand_state
        self._interval_ms = max(1, int(1000 / update_hz))
        self._timer = None
        self._controller = None
        self._running = False
        self.stopRequested.connect(self.stop, Qt.QueuedConnection)

    def start(self):
        try:
            self._controller = HandController(self._config_path, self._hand_state.is_left)
            self._controller.enable_torque()
        except Exception as exc:
            self.error.emit(str(exc))
            return

        self._running = True
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.PreciseTimer)
        self._timer.timeout.connect(self._tick)
        self._timer.start(self._interval_ms)

    def stop(self):
        self._running = False
        if self._timer is not None:
            self._timer.stop()
        if self._controller is not None:
            try:
                self._controller.disable_torque()
            finally:
                self._controller.close()
                self._controller = None

    def _tick(self):
        if not self._running or self._controller is None:
            return
        targets = self._hand_state.targets()
        data = self._controller.write_and_read_cycle(targets)
        self.feedbackReady.emit(
            data["present_positions"],
            data["currents"],
            data["temperatures"],
            data["ir_value"],
        )

def main():
    app = QApplication(sys.argv)

    system = platform.system()  # "Windows", "Linux", "Darwin"

    #config_path = Path(__file__).parent / "config" / "RH8D.yaml"
    #config_path = os.path.join(os.path.dirname(sys.executable), "config/RH8D.yaml")
    #with open(config_path, "r") as f:
    #    config = yaml.safe_load(f)

    def get_base_path():
        if getattr(sys, 'frozen', False):
            # Running as compiled EXE
            return os.path.dirname(sys.executable)
        else:
            # Running as normal Python script
            return os.path.dirname(os.path.abspath(__file__))

    config_path = os.path.join(get_base_path(), "RH8D.yaml")

    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if system == "Windows":
        sensor_port = config["sensor_port_windows"]
        control_port = config["control_port_windows"]
    elif system == "Linux":
        sensor_port = config["sensor_port_linux"]
        control_port = config["control_port_linux"]


    joint_ids = None
    if scan_id(control_port,config["baudrate"],config["r_mainboard"],2.0):
        joint_ids = list(config["r_joint_mapping"].values())
        is_left = False
    elif scan_id(control_port,config["baudrate"],config["l_mainboard"],2.0):
        joint_ids = list(config["l_joint_mapping"].values())
        is_left = True

    else:
        return


    #joint_ids = list(config["joint_mapping"].values())
    hand_state = HandState(joint_ids, is_left)

    left = RobotGLWidget(is_left)
    left.setMinimumWidth(300)
    hand_state.presentChanged.connect(left.map_HandState_to_q)

    right = HandControlWidget(config, hand_state, is_left)
    right.init_hand()
    hand_state.currentChanged.connect(right.update_current)

    content_row = QHBoxLayout()
    content_row.addWidget(left, 3)
    content_row.addWidget(right, 2)

    main_layout = QVBoxLayout()
    sensor_row = SensorWidget()
    main_layout.addWidget(sensor_row, 2)
    main_layout.addLayout(content_row, 4)

    window = QWidget()
    window.setLayout(main_layout)
    window.resize(1200, 800)
    window.show()

    sensor_reader = SensorReader(port = sensor_port, baudrate= config["baudrate"],interval_ms=SENSOR_FREQ)
    sensor_reader.dataReady.connect(sensor_row.update_from_data)
    sensor_reader.start()

    hand_state.irChanged.connect(sensor_row.update_IR_label)

    io_thread = QThread(window)
    worker = HandIOWorker(str(config_path), hand_state, update_hz=UPDATE_HZ)
    worker.moveToThread(io_thread)
    io_thread.started.connect(worker.start)
    worker.feedbackReady.connect(hand_state.update_feedback, Qt.QueuedConnection)
    worker.error.connect(lambda msg: print(f"Hand IO error: {msg}"))
    io_thread.start()

    def _shutdown():
        if io_thread.isRunning():
            QMetaObject.invokeMethod(worker, "stop", Qt.BlockingQueuedConnection)
            io_thread.quit()
            io_thread.wait(2000)
        sensor_reader.stop()
        sensor_reader.wait(2000)

    app.aboutToQuit.connect(_shutdown)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
