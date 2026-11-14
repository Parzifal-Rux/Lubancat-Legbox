"""双轮足机器人上位机控制系统。
主控链路：MPU6050 -> 姿态估计 -> 目标规划 -> V2串口帧 -> ESP32-S3 -> RS485 -> Go1。
默认运行仿真；真实硬件需要显式指定 --hardware 并在图形界面中解锁。
V2协议与 ESP32_Balance_PID_V2.ino 对应，配对实现Go1底层电机协议。
"""
import argparse
import cmd
import csv
import dataclasses
import enum
import json
import math
import os
import queue
import random
import shlex
import signal
import statistics
import struct
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Any
from typing import Callable
from typing import Deque
from typing import Dict
from typing import Iterable
from typing import Iterator
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple
APP_NAME = "双轮足机器人控制系统"
APP_VERSION = "1.0.0"
FRAME_HEAD = b"\xAA\x55"
FRAME_TAIL = b"\x0D\x0A"
V1_PAYLOAD_LENGTH = 12
V1_FRAME_LENGTH = 17
V2_PAYLOAD_LENGTH = 25
V2_FRAME_LENGTH = 30
V2_STRUCT = struct.Struct("<ffffffB")
DEG_PER_RAD = 180.0 / math.pi
RAD_PER_DEG = math.pi / 180.0
GRAVITY = 9.80665
MPU_DEFAULT_ADDRESS = 0x69
MPU_DEFAULT_BUS = 6
MPU_ACCEL_SENSITIVITY = 8192.0
MPU_GYRO_SENSITIVITY = 16.4
MPU_REG_SMPLRT_DIV = 0x19
MPU_REG_CONFIG = 0x1A
MPU_REG_GYRO_CONFIG = 0x1B
MPU_REG_ACCEL_CONFIG = 0x1C
MPU_REG_ACCEL_XOUT_H = 0x3B
MPU_REG_SIGNAL_PATH_RESET = 0x68
MPU_REG_PWR_MGMT_1 = 0x6B
MPU_REG_PWR_MGMT_2 = 0x6C
MPU_REG_WHO_AM_I = 0x75
SERIAL_DEFAULT_PORT = "/dev/ttyACM0"
SERIAL_DEFAULT_BAUDRATE = 115200
CONTROL_DEFAULT_HZ = 70.0
HISTORY_DEFAULT_CAPACITY = 2000
class RobotCommand(enum.IntEnum):
    """ESP32 V2支持的三个命令字。"""
    STOP = 0
    BALANCE = 1
    WALK = 2
class RunMode(enum.Enum):
    SIMULATION = "simulation"
    HARDWARE = "hardware"
class ConnectionState(enum.Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"
class SafetyLevel(enum.Enum):
    NORMAL = "normal"
    WARNING = "warning"
    LOCKED = "locked"
class FaultCode(enum.Enum):
    NONE = "none"
    MANUAL_ESTOP = "manual_estop"
    FALL_DETECTED = "fall_detected"
    SENSOR_TIMEOUT = "sensor_timeout"
    SENSOR_INVALID = "sensor_invalid"
    SENSOR_DISCONNECTED = "sensor_disconnected"
    SERIAL_FAILURE = "serial_failure"
    CONTROL_OVERRUN = "control_overrun"
    TARGET_INVALID = "target_invalid"
    CONFIG_INVALID = "config_invalid"
    INTERNAL_ERROR = "internal_error"
class EventLevel(enum.IntEnum):
    DEBUG = 10
    INFO = 20
    WARNING = 30
    ERROR = 40
    CRITICAL = 50
class EventTopic(enum.Enum):
    SYSTEM = "system"
    SENSOR = "sensor"
    CONTROL = "control"
    SAFETY = "safety"
    TRANSPORT = "transport"
    USER = "user"
    DIAGNOSTIC = "diagnostic"
@dataclass(frozen=True)
class SerialConfig:
    port: str = SERIAL_DEFAULT_PORT
    baudrate: int = SERIAL_DEFAULT_BAUDRATE
    timeout_seconds: float = 0.01
    write_timeout_seconds: float = 0.1
    reconnect_delay_seconds: float = 1.0
    stop_repeat_count: int = 5
    def validate(self) -> None:
        if not self.port.strip():
            raise ValueError("串口名称不能为空")
        if self.baudrate <= 0:
            raise ValueError("波特率必须大于零")
        if self.timeout_seconds < 0.0:
            raise ValueError("串口读超时不能为负数")
        if self.write_timeout_seconds <= 0.0:
            raise ValueError("串口写超时必须大于零")
        if self.reconnect_delay_seconds < 0.0:
            raise ValueError("重连等待不能为负数")
        if self.stop_repeat_count < 1:
            raise ValueError("停止帧重复次数至少为一次")
@dataclass(frozen=True)
class SensorConfig:
    i2c_bus: int = MPU_DEFAULT_BUS
    i2c_address: int = MPU_DEFAULT_ADDRESS
    accel_sensitivity: float = MPU_ACCEL_SENSITIVITY
    gyro_sensitivity: float = MPU_GYRO_SENSITIVITY
    complementary_alpha: float = 0.98
    calibration_samples: int = 500
    calibration_delay_seconds: float = 0.002
    sensor_timeout_seconds: float = 0.25
    maximum_dt_seconds: float = 0.1
    maximum_gyro_dps: float = 2000.0
    maximum_accel_g: float = 4.0
    simulation_noise_dps: float = 0.025
    simulation_noise_g: float = 0.002
    def validate(self) -> None:
        if self.i2c_bus < 0:
            raise ValueError("I2C总线编号不能为负数")
        if not 0x03 <= self.i2c_address <= 0x77:
            raise ValueError("I2C地址超出有效范围")
        if self.accel_sensitivity <= 0.0:
            raise ValueError("加速度计灵敏度必须大于零")
        if self.gyro_sensitivity <= 0.0:
            raise ValueError("陀螺仪灵敏度必须大于零")
        if not 0.0 <= self.complementary_alpha < 1.0:
            raise ValueError("互补滤波系数必须在[0,1)内")
        if self.calibration_samples < 10:
            raise ValueError("校准样本至少需要十个")
        if self.sensor_timeout_seconds <= 0.0:
            raise ValueError("传感器超时必须大于零")
        if self.maximum_dt_seconds <= 0.0:
            raise ValueError("最大采样周期必须大于零")
@dataclass(frozen=True)
class SafetyConfig:
    fall_angle_degrees: float = 30.0
    warning_angle_degrees: float = 20.0
    sensor_timeout_seconds: float = 0.25
    maximum_target_angle_degrees: float = 20.0
    maximum_walk_amplitude_degrees: float = 8.0
    maximum_walk_frequency_hz: float = 1.0
    maximum_control_overrun_seconds: float = 0.2
    require_explicit_unlock: bool = True
    stop_on_sensor_error: bool = True
    stop_on_serial_error: bool = True
    def validate(self) -> None:
        if self.fall_angle_degrees <= 0.0:
            raise ValueError("摔倒阈值必须大于零")
        if self.warning_angle_degrees < 0.0:
            raise ValueError("预警角度不能为负数")
        if self.warning_angle_degrees >= self.fall_angle_degrees:
            raise ValueError("预警角度必须小于摔倒阈值")
        if self.maximum_target_angle_degrees <= 0.0:
            raise ValueError("目标角上限必须大于零")
        if self.maximum_target_angle_degrees >= self.fall_angle_degrees:
            raise ValueError("目标角上限应小于摔倒阈值")
        if self.maximum_walk_amplitude_degrees <= 0.0:
            raise ValueError("步态摆幅上限必须大于零")
        if self.maximum_walk_frequency_hz <= 0.0:
            raise ValueError("步态频率上限必须大于零")
@dataclass(frozen=True)
class ControlConfig:
    frequency_hz: float = CONTROL_DEFAULT_HZ
    target_slew_dps: float = 45.0
    walk_frequency_hz: float = 0.3
    walk_amplitude_degrees: float = 8.0
    history_capacity: int = HISTORY_DEFAULT_CAPACITY
    display_decimation: int = 7
    statistics_window_seconds: float = 2.0
    def validate(self) -> None:
        if self.frequency_hz <= 0.0:
            raise ValueError("控制频率必须大于零")
        if self.frequency_hz > 1000.0:
            raise ValueError("控制频率不应超过1000Hz")
        if self.target_slew_dps <= 0.0:
            raise ValueError("目标变化速率必须大于零")
        if self.walk_frequency_hz <= 0.0:
            raise ValueError("步态频率必须大于零")
        if self.walk_amplitude_degrees < 0.0:
            raise ValueError("步态摆幅不能为负数")
        if self.history_capacity < 10:
            raise ValueError("历史缓冲区至少需要十项")
        if self.display_decimation < 1:
            raise ValueError("显示抽取因子至少为一")
@dataclass(frozen=True)
class RobotGeometry:
    body_length_m: float = 0.60
    body_width_m: float = 0.30
    nominal_height_m: float = 0.30
    hip_x_m: float = 0.25
    hip_y_m: float = 0.13
    thigh_length_m: float = 0.21
    shank_length_m: float = 0.21
    def validate(self) -> None:
        values = dataclasses.astuple(self)
        if any(value <= 0.0 for value in values):
            raise ValueError("机器人几何尺寸必须全部大于零")
@dataclass(frozen=True)
class ActuatorLimits:
    maximum_speed_rad_s: float = 30.0
    maximum_torque_nm: float = 23.7
    minimum_supply_voltage: float = 12.0
    maximum_supply_voltage: float = 30.0
    def validate(self) -> None:
        if self.maximum_speed_rad_s <= 0.0:
            raise ValueError("电机速度上限必须大于零")
        if self.maximum_torque_nm <= 0.0:
            raise ValueError("电机扭矩上限必须大于零")
        if self.minimum_supply_voltage >= self.maximum_supply_voltage:
            raise ValueError("供电电压范围无效")
@dataclass(frozen=True)
class AppConfig:
    mode: RunMode = RunMode.SIMULATION
    serial: SerialConfig = field(default_factory=SerialConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    geometry: RobotGeometry = field(default_factory=RobotGeometry)
    actuator: ActuatorLimits = field(default_factory=ActuatorLimits)
    log_directory: str = "robot_logs"
    auto_start: bool = False
    def validate(self) -> None:
        self.serial.validate()
        self.sensor.validate()
        self.safety.validate()
        self.control.validate()
        self.geometry.validate()
        self.actuator.validate()
        if self.control.walk_frequency_hz > self.safety.maximum_walk_frequency_hz:
            raise ValueError("默认步态频率超过安全限制")
        if self.control.walk_amplitude_degrees > self.safety.maximum_walk_amplitude_degrees:
            raise ValueError("默认步态摆幅超过安全限制")
    def to_dict(self) -> Dict[str, Any]:
        result = dataclasses.asdict(self)
        result["mode"] = self.mode.value
        return result
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AppConfig":
        config = cls(
            mode=RunMode(data.get("mode", RunMode.SIMULATION.value)),
            serial=SerialConfig(**data.get("serial", {})),
            sensor=SensorConfig(**data.get("sensor", {})),
            safety=SafetyConfig(**data.get("safety", {})),
            control=ControlConfig(**data.get("control", {})),
            geometry=RobotGeometry(**data.get("geometry", {})),
            actuator=ActuatorLimits(**data.get("actuator", {})),
            log_directory=data.get("log_directory", "robot_logs"),
            auto_start=bool(data.get("auto_start", False)),
        )
        config.validate()
        return config
@dataclass(frozen=True)
class Attitude:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    def is_finite(self) -> bool:
        return all(math.isfinite(value) for value in self.as_tuple())
    def as_tuple(self) -> Tuple[float, float, float]:
        return self.roll, self.pitch, self.yaw
    def as_dict(self) -> Dict[str, float]:
        return {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw}
    def limited(self, limit_degrees: float) -> "Attitude":
        return Attitude(
            clamp(self.roll, -limit_degrees, limit_degrees),
            clamp(self.pitch, -limit_degrees, limit_degrees),
            clamp(self.yaw, -limit_degrees, limit_degrees),
        )
    def distance_to(self, other: "Attitude") -> float:
        return math.sqrt(
            (self.roll - other.roll) ** 2
            + (self.pitch - other.pitch) ** 2
            + shortest_angle_delta(self.yaw, other.yaw) ** 2
        )
@dataclass(frozen=True)
class AngularRate:
    roll_dps: float = 0.0
    pitch_dps: float = 0.0
    yaw_dps: float = 0.0
    def is_finite(self) -> bool:
        return all(math.isfinite(value) for value in self.as_tuple())
    def as_tuple(self) -> Tuple[float, float, float]:
        return self.roll_dps, self.pitch_dps, self.yaw_dps
@dataclass(frozen=True)
class Acceleration:
    x_g: float = 0.0
    y_g: float = 0.0
    z_g: float = 1.0
    def is_finite(self) -> bool:
        return all(math.isfinite(value) for value in self.as_tuple())
    def as_tuple(self) -> Tuple[float, float, float]:
        return self.x_g, self.y_g, self.z_g
    def magnitude_g(self) -> float:
        return math.sqrt(self.x_g ** 2 + self.y_g ** 2 + self.z_g ** 2)
@dataclass(frozen=True)
class SensorSample:
    attitude: Attitude
    angular_rate: AngularRate
    acceleration: Acceleration
    timestamp: float
    sequence: int
    source: str
    calibrated: bool = True
    def age_seconds(self, now: Optional[float] = None) -> float:
        reference = time.monotonic() if now is None else now
        return max(0.0, reference - self.timestamp)
    def is_valid(self) -> bool:
        return self.attitude.is_finite() and self.angular_rate.is_finite() and self.acceleration.is_finite()
@dataclass(frozen=True)
class V2Packet:
    attitude: Attitude
    target: Attitude
    command: RobotCommand
    def encode(self) -> bytes:
        return encode_v2_frame(self.attitude, self.target, self.command)
@dataclass(frozen=True)
class ControlSnapshot:
    timestamp: float
    sequence: int
    command: RobotCommand
    attitude: Attitude
    target: Attitude
    filtered_target: Attitude
    sensor_age_seconds: float
    control_period_seconds: float
    frames_sent: int
    safety_level: SafetyLevel
    fault: FaultCode
    connection: ConnectionState
    armed: bool
@dataclass(frozen=True)
class SystemEvent:
    timestamp: float
    level: EventLevel
    topic: EventTopic
    message: str
    details: Dict[str, Any] = field(default_factory=dict)
    def format(self) -> str:
        local_time = time.strftime("%H:%M:%S", time.localtime(self.timestamp))
        return f"{local_time} [{self.level.name}] [{self.topic.value}] {self.message}"
@dataclass
class RuntimeStatistics:
    sensor_updates: int = 0
    control_cycles: int = 0
    frames_encoded: int = 0
    frames_sent: int = 0
    frames_failed: int = 0
    bytes_sent: int = 0
    protocol_errors: int = 0
    safety_trips: int = 0
    emergency_stops: int = 0
    started_at: float = field(default_factory=time.monotonic)
    last_cycle_duration: float = 0.0
    maximum_cycle_duration: float = 0.0
    def uptime_seconds(self) -> float:
        return max(0.0, time.monotonic() - self.started_at)
    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)
def clamp(value: float, minimum: float, maximum: float) -> float:
    if minimum > maximum:
        raise ValueError("下限不能大于上限")
    return min(maximum, max(minimum, value))
def saturate_signed(value: float, limit: float) -> float:
    if limit < 0.0:
        raise ValueError("限幅值不能为负数")
    return clamp(value, -limit, limit)
def lerp(start: float, end: float, ratio: float) -> float:
    return start + (end - start) * clamp(ratio, 0.0, 1.0)
def move_toward(current: float, target: float, maximum_delta: float) -> float:
    if maximum_delta < 0.0:
        raise ValueError("最大变化量不能为负数")
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + math.copysign(maximum_delta, delta)
def wrap_degrees(angle: float) -> float:
    wrapped = (angle + 180.0) % 360.0 - 180.0
    if wrapped == -180.0 and angle > 0.0:
        return 180.0
    return wrapped
def shortest_angle_delta(target: float, current: float) -> float:
    return wrap_degrees(target - current)
def degrees_to_radians(value: float) -> float:
    return value * RAD_PER_DEG
def radians_to_degrees(value: float) -> float:
    return value * DEG_PER_RAD
def xor_checksum(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum
def ensure_finite(values: Iterable[float], name: str = "数值") -> None:
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{name}包含NaN或无穷大")
def safe_divide(numerator: float, denominator: float, default: float = 0.0) -> float:
    if abs(denominator) < 1e-12:
        return default
    return numerator / denominator
def monotonic_timestamp() -> float:
    return time.monotonic()
def wall_timestamp() -> float:
    return time.time()
def format_duration(seconds: float) -> str:
    total = max(0, int(seconds))
    hours = total // 3600
    minutes = (total % 3600) // 60
    remainder = total % 60
    return f"{hours:02d}:{minutes:02d}:{remainder:02d}"
def atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    os.replace(str(temporary), str(path))
def load_json_file(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("配置根节点必须是JSON对象")
    return data
def save_json_file(path: Path, data: Dict[str, Any]) -> None:
    content = json.dumps(data, ensure_ascii=False, indent=2)
    atomic_write_text(path, content + "\n")
def command_name(command: RobotCommand) -> str:
    names = {RobotCommand.STOP: "停止", RobotCommand.BALANCE: "平衡", RobotCommand.WALK: "走路"}
    return names[command]
def fault_message(fault: FaultCode) -> str:
    messages = {
        FaultCode.NONE: "无故障",
        FaultCode.MANUAL_ESTOP: "人工急停已锁存",
        FaultCode.FALL_DETECTED: "姿态超过摔倒阈值",
        FaultCode.SENSOR_TIMEOUT: "姿态传感器数据超时",
        FaultCode.SENSOR_INVALID: "姿态传感器数据无效",
        FaultCode.SENSOR_DISCONNECTED: "姿态传感器已断开",
        FaultCode.SERIAL_FAILURE: "串口发送失败",
        FaultCode.CONTROL_OVERRUN: "控制循环严重超时",
        FaultCode.TARGET_INVALID: "目标角度无效",
        FaultCode.CONFIG_INVALID: "配置无效",
        FaultCode.INTERNAL_ERROR: "控制系统内部错误",
    }
    return messages[fault]
class EventBus:
    """线程安全的轻量事件总线，用于引擎、CLI和GUI解耦。"""
    def __init__(self, history_limit: int = 500) -> None:
        self._subscribers: List[Callable[[SystemEvent], None]] = []
        self._history: Deque[SystemEvent] = deque(maxlen=history_limit)
        self._lock = threading.RLock()
    def subscribe(self, callback: Callable[[SystemEvent], None]) -> Callable[[], None]:
        if not callable(callback):
            raise TypeError("事件订阅者必须可调用")
        with self._lock:
            if callback not in self._subscribers:
                self._subscribers.append(callback)
        def unsubscribe() -> None:
            self.unsubscribe(callback)
        return unsubscribe
    def unsubscribe(self, callback: Callable[[SystemEvent], None]) -> None:
        with self._lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)
    def publish(self, event: SystemEvent) -> None:
        with self._lock:
            self._history.append(event)
            subscribers = tuple(self._subscribers)
        for callback in subscribers:
            try:
                callback(event)
            except Exception:
                traceback.print_exc()
    def emit(
        self,
        level: EventLevel,
        topic: EventTopic,
        message: str,
        **details: Any,
    ) -> SystemEvent:
        event = SystemEvent(wall_timestamp(), level, topic, message, details)
        self.publish(event)
        return event
    def history(self, minimum_level: EventLevel = EventLevel.DEBUG) -> List[SystemEvent]:
        with self._lock:
            return [event for event in self._history if event.level >= minimum_level]
    def clear_history(self) -> None:
        with self._lock:
            self._history.clear()
class EventQueueAdapter:
    """将事件投递到队列，供定时轮询的界面线程消费。"""
    def __init__(self, bus: EventBus, maximum_size: int = 1000) -> None:
        self.bus = bus
        self.queue: queue.Queue[SystemEvent] = queue.Queue(maxsize=maximum_size)
        self._unsubscribe = bus.subscribe(self._on_event)
    def _on_event(self, event: SystemEvent) -> None:
        try:
            self.queue.put_nowait(event)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(event)
            except queue.Full:
                pass
    def drain(self, limit: int = 200) -> List[SystemEvent]:
        events: List[SystemEvent] = []
        for _ in range(limit):
            try:
                events.append(self.queue.get_nowait())
            except queue.Empty:
                break
        return events
    def close(self) -> None:
        self._unsubscribe()
class ProtocolError(ValueError):
    """串口帧格式、长度或校验错误。"""
class ProtocolStatistics:
    def __init__(self) -> None:
        self.v1_frames = 0
        self.v2_frames = 0
        self.bad_checksums = 0
        self.bad_lengths = 0
        self.bad_commands = 0
        self.discarded_bytes = 0
        self.resynchronizations = 0
    def as_dict(self) -> Dict[str, int]:
        return dict(self.__dict__)
    def reset(self) -> None:
        for key in self.__dict__:
            setattr(self, key, 0)
def encode_v1_frame(attitude: Attitude) -> bytes:
    """兼容旧接收程序的17字节姿态帧，不用于V2平衡控制。"""
    ensure_finite(attitude.as_tuple(), "姿态")
    payload = struct.pack("<fff", *attitude.as_tuple())
    return FRAME_HEAD + payload + bytes((xor_checksum(payload),)) + FRAME_TAIL
def decode_v1_frame(frame: bytes) -> Attitude:
    if len(frame) != V1_FRAME_LENGTH:
        raise ProtocolError(f"V1帧长度应为{V1_FRAME_LENGTH}，实际为{len(frame)}")
    if not frame.startswith(FRAME_HEAD):
        raise ProtocolError("V1帧头错误")
    if not frame.endswith(FRAME_TAIL):
        raise ProtocolError("V1帧尾错误")
    payload = frame[2:14]
    if xor_checksum(payload) != frame[14]:
        raise ProtocolError("V1异或校验失败")
    values = struct.unpack("<fff", payload)
    ensure_finite(values, "V1姿态")
    return Attitude(*values)
def encode_v2_frame(attitude: Attitude, target: Attitude, command: RobotCommand) -> bytes:
    """构造30字节V2帧，XOR覆盖六个float32与Cmd共25字节。"""
    ensure_finite(attitude.as_tuple(), "当前姿态")
    ensure_finite(target.as_tuple(), "目标姿态")
    command = RobotCommand(command)
    payload = V2_STRUCT.pack(*attitude.as_tuple(), *target.as_tuple(), int(command))
    if len(payload) != V2_PAYLOAD_LENGTH:
        raise AssertionError("V2有效载荷长度内部错误")
    checksum = xor_checksum(payload)
    frame = FRAME_HEAD + payload + bytes((checksum,)) + FRAME_TAIL
    if len(frame) != V2_FRAME_LENGTH:
        raise AssertionError("V2帧长度内部错误")
    return frame
def decode_v2_frame(frame: bytes) -> V2Packet:
    if len(frame) != V2_FRAME_LENGTH:
        raise ProtocolError(f"V2帧长度应为{V2_FRAME_LENGTH}，实际为{len(frame)}")
    if frame[0:2] != FRAME_HEAD:
        raise ProtocolError("V2帧头错误")
    if frame[28:30] != FRAME_TAIL:
        raise ProtocolError("V2帧尾错误")
    payload = frame[2:27]
    if frame[27] != xor_checksum(payload):
        raise ProtocolError("V2异或校验失败")
    values = V2_STRUCT.unpack(payload)
    ensure_finite(values[:6], "V2姿态或目标")
    try:
        command = RobotCommand(values[6])
    except ValueError as error:
        raise ProtocolError(f"未知V2命令字: {values[6]}") from error
    attitude = Attitude(values[0], values[1], values[2])
    target = Attitude(values[3], values[4], values[5])
    return V2Packet(attitude, target, command)
def packet_wire_offsets() -> Dict[str, Tuple[int, int]]:
    """返回V2线上字节区间，供诊断和协议测试使用。"""
    return {
        "header": (0, 2),
        "roll": (2, 6),
        "pitch": (6, 10),
        "yaw": (10, 14),
        "target_roll": (14, 18),
        "target_pitch": (18, 22),
        "target_yaw": (22, 26),
        "command": (26, 27),
        "xor": (27, 28),
        "tail": (28, 30),
    }
def explain_v2_frame(frame: bytes) -> Dict[str, Any]:
    packet = decode_v2_frame(frame)
    offsets = packet_wire_offsets()
    return {
        "length": len(frame),
        "header_hex": frame[slice(*offsets["header"])].hex(" ").upper(),
        "attitude": packet.attitude.as_dict(),
        "target": packet.target.as_dict(),
        "command": int(packet.command),
        "command_name": command_name(packet.command),
        "xor_calculated": xor_checksum(frame[2:27]),
        "xor_received": frame[27],
        "tail_hex": frame[slice(*offsets["tail"])].hex(" ").upper(),
        "payload_hex": frame[2:27].hex(" ").upper(),
    }
class StreamFrameParser:
    """可从任意字节流中重新同步并拆解V1/V2帧。"""
    def __init__(self, maximum_buffer_bytes: int = 4096) -> None:
        if maximum_buffer_bytes < V2_FRAME_LENGTH * 2:
            raise ValueError("流解析缓冲区太小")
        self.maximum_buffer_bytes = maximum_buffer_bytes
        self._buffer = bytearray()
        self.statistics = ProtocolStatistics()
    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)
    def reset(self) -> None:
        self._buffer.clear()
    def feed(self, data: bytes) -> List[V2Packet]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("串口输入必须是字节数据")
        self._buffer.extend(data)
        packets: List[V2Packet] = []
        while True:
            packet, consumed = self._parse_one()
            if consumed <= 0:
                break
            del self._buffer[:consumed]
            if packet is not None:
                packets.append(packet)
        if len(self._buffer) > self.maximum_buffer_bytes:
            discarded = len(self._buffer) - self.maximum_buffer_bytes
            del self._buffer[:discarded]
            self.statistics.discarded_bytes += discarded
            self.statistics.resynchronizations += 1
        return packets
    def _parse_one(self) -> Tuple[Optional[V2Packet], int]:
        head_index = self._buffer.find(FRAME_HEAD)
        if head_index < 0:
            retained = 1 if self._buffer.endswith(FRAME_HEAD[:1]) else 0
            consumed = len(self._buffer) - retained
            self.statistics.discarded_bytes += max(0, consumed)
            return None, max(0, consumed)
        if head_index > 0:
            self.statistics.discarded_bytes += head_index
            self.statistics.resynchronizations += 1
            return None, head_index
        if len(self._buffer) < V1_FRAME_LENGTH:
            return None, 0
        candidate_length = self._candidate_length()
        if candidate_length is None:
            self.statistics.discarded_bytes += 1
            self.statistics.resynchronizations += 1
            return None, 1
        if len(self._buffer) < candidate_length:
            return None, 0
        raw = bytes(self._buffer[:candidate_length])
        try:
            if candidate_length == V1_FRAME_LENGTH:
                attitude = decode_v1_frame(raw)
                self.statistics.v1_frames += 1
                return V2Packet(attitude, Attitude(), RobotCommand.STOP), candidate_length
            packet = decode_v2_frame(raw)
            self.statistics.v2_frames += 1
            return packet, candidate_length
        except ProtocolError:
            self.statistics.bad_checksums += 1
            self.statistics.resynchronizations += 1
            return None, 1
    def _candidate_length(self) -> Optional[int]:
        if len(self._buffer) >= V1_FRAME_LENGTH:
            if self._buffer[15:17] == FRAME_TAIL:
                return V1_FRAME_LENGTH
        if len(self._buffer) >= V2_FRAME_LENGTH:
            if self._buffer[28:30] == FRAME_TAIL:
                return V2_FRAME_LENGTH
            return None
        return V2_FRAME_LENGTH
class SerialTransport:
    """传输接口，具体实现只在需要时加载外部依赖。"""
    def __init__(self, config: SerialConfig, bus: EventBus) -> None:
        self.config = config
        self.bus = bus
        self.state = ConnectionState.DISCONNECTED
        self.sent_frames = 0
        self.sent_bytes = 0
        self.failures = 0
    @property
    def is_connected(self) -> bool:
        return self.state == ConnectionState.CONNECTED
    def connect(self) -> None:
        raise NotImplementedError
    def close(self) -> None:
        raise NotImplementedError
    def send(self, data: bytes) -> int:
        raise NotImplementedError
    def receive(self, maximum_bytes: int = 4096) -> bytes:
        return b""
    def _mark_connected(self) -> None:
        self.state = ConnectionState.CONNECTED
        self.bus.emit(EventLevel.INFO, EventTopic.TRANSPORT, "控制串口已连接")
    def _mark_closed(self) -> None:
        self.state = ConnectionState.DISCONNECTED
        self.bus.emit(EventLevel.INFO, EventTopic.TRANSPORT, "控制串口已关闭")
    def _record_send(self, data: bytes) -> int:
        self.sent_frames += 1
        self.sent_bytes += len(data)
        return len(data)
    def _record_failure(self, error: Exception) -> None:
        self.failures += 1
        self.state = ConnectionState.ERROR
        self.bus.emit(EventLevel.ERROR, EventTopic.TRANSPORT, f"串口操作失败: {error}")
class SimulatedSerialTransport(SerialTransport):
    """记录输出帧的可测仿真串口，不产生任何硬件动作。"""
    def __init__(self, config: SerialConfig, bus: EventBus) -> None:
        super().__init__(config, bus)
        self.frames: Deque[bytes] = deque(maxlen=500)
        self.loopback_parser = StreamFrameParser()
        self.fail_next_send = False
        self.echo_input = False
        self._incoming = bytearray()
        self._lock = threading.Lock()
    def connect(self) -> None:
        self.state = ConnectionState.CONNECTING
        self._mark_connected()
    def close(self) -> None:
        self._mark_closed()
    def send(self, data: bytes) -> int:
        if not self.is_connected:
            raise RuntimeError("仿真串口尚未连接")
        if self.fail_next_send:
            self.fail_next_send = False
            error = RuntimeError("注入的仿真串口发送故障")
            self._record_failure(error)
            raise error
        if not isinstance(data, bytes):
            raise TypeError("发送内容必须为bytes")
        with self._lock:
            self.frames.append(data)
            if self.echo_input:
                self._incoming.extend(data)
        return self._record_send(data)
    def receive(self, maximum_bytes: int = 4096) -> bytes:
        with self._lock:
            count = min(maximum_bytes, len(self._incoming))
            result = bytes(self._incoming[:count])
            del self._incoming[:count]
            return result
    def last_frame(self) -> Optional[bytes]:
        with self._lock:
            return self.frames[-1] if self.frames else None
    def inject_input(self, data: bytes) -> None:
        with self._lock:
            self._incoming.extend(data)
    def clear(self) -> None:
        with self._lock:
            self.frames.clear()
            self._incoming.clear()
class PySerialTransport(SerialTransport):
    """pyserial实现，导入和打开均延迟到真实硬件模式。"""
    def __init__(self, config: SerialConfig, bus: EventBus) -> None:
        super().__init__(config, bus)
        self._serial: Any = None
        self._lock = threading.Lock()
    def connect(self) -> None:
        if self.is_connected:
            return
        self.state = ConnectionState.CONNECTING
        try:
            import serial
        except ImportError as error:
            self.state = ConnectionState.ERROR
            raise RuntimeError("真实硬件模式需要安装pyserial: python -m pip install pyserial") from error
        try:
            self._serial = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=self.config.timeout_seconds,
                write_timeout=self.config.write_timeout_seconds,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as error:
            self._serial = None
            self._record_failure(error)
            raise RuntimeError(f"无法打开串口{self.config.port}: {error}") from error
        self._mark_connected()
    def close(self) -> None:
        with self._lock:
            serial_object = self._serial
            self._serial = None
        if serial_object is not None:
            try:
                serial_object.close()
            except Exception as error:
                self.bus.emit(EventLevel.WARNING, EventTopic.TRANSPORT, f"关闭串口时出现警告: {error}")
        self._mark_closed()
    def send(self, data: bytes) -> int:
        if not isinstance(data, bytes):
            raise TypeError("发送内容必须为bytes")
        with self._lock:
            if self._serial is None or not self._serial.is_open:
                raise RuntimeError("控制串口未打开")
            try:
                written = self._serial.write(data)
                self._serial.flush()
            except Exception as error:
                self._record_failure(error)
                raise RuntimeError(f"串口写入失败: {error}") from error
        if written != len(data):
            error = RuntimeError(f"串口短写: {written}/{len(data)}")
            self._record_failure(error)
            raise error
        return self._record_send(data)
    def receive(self, maximum_bytes: int = 4096) -> bytes:
        with self._lock:
            if self._serial is None or not self._serial.is_open:
                return b""
            try:
                waiting = int(self._serial.in_waiting)
                count = min(maximum_bytes, max(1, waiting))
                return self._serial.read(count)
            except Exception as error:
                self._record_failure(error)
                raise RuntimeError(f"串口读取失败: {error}") from error
def create_transport(config: AppConfig, bus: EventBus) -> SerialTransport:
    if config.mode == RunMode.SIMULATION:
        return SimulatedSerialTransport(config.serial, bus)
    return PySerialTransport(config.serial, bus)
def send_stop_frames(transport: SerialTransport, attitude: Optional[Attitude] = None) -> int:
    """退出和异常路径共用的停止帧发送函数。"""
    current = Attitude() if attitude is None else attitude
    if not current.is_finite():
        current = Attitude()
    frame = encode_v2_frame(current, Attitude(), RobotCommand.STOP)
    successes = 0
    for _ in range(transport.config.stop_repeat_count):
        try:
            transport.send(frame)
            successes += 1
        except Exception:
            break
        time.sleep(0.005)
    return successes
@dataclass(frozen=True)
class RawImuSample:
    acceleration: Acceleration
    angular_rate: AngularRate
    temperature_celsius: float
    timestamp: float
class MovingAverageFilter:
    def __init__(self, window_size: int = 10) -> None:
        if window_size < 1:
            raise ValueError("滑动平均窗口至少为一")
        self._values: Deque[float] = deque(maxlen=window_size)
    def update(self, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("滤波输入不是有限数")
        self._values.append(value)
        return self.value
    @property
    def value(self) -> float:
        if not self._values:
            return 0.0
        return math.fsum(self._values) / len(self._values)
    @property
    def sample_count(self) -> int:
        return len(self._values)
    def reset(self) -> None:
        self._values.clear()
class LowPassFilter:
    def __init__(self, alpha: float = 0.2, initial_value: float = 0.0) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("低通滤波alpha必须在(0,1]内")
        self.alpha = alpha
        self.value = initial_value
        self.initialized = False
    def update(self, sample: float) -> float:
        if not math.isfinite(sample):
            raise ValueError("低通滤波输入不是有限数")
        if not self.initialized:
            self.value = sample
            self.initialized = True
        else:
            self.value = self.alpha * sample + (1.0 - self.alpha) * self.value
        return self.value
    def reset(self, value: float = 0.0) -> None:
        self.value = value
        self.initialized = False
class RateLimiter:
    def __init__(self, maximum_rate_per_second: float) -> None:
        if maximum_rate_per_second <= 0.0:
            raise ValueError("变化率限制必须大于零")
        self.maximum_rate_per_second = maximum_rate_per_second
        self.previous_value: Optional[float] = None
    def update(self, target: float, dt_seconds: float) -> float:
        if dt_seconds < 0.0:
            raise ValueError("时间增量不能为负数")
        if not math.isfinite(target):
            raise ValueError("限速目标不是有限数")
        if self.previous_value is None:
            self.previous_value = target
            return target
        maximum_delta = self.maximum_rate_per_second * dt_seconds
        self.previous_value = move_toward(self.previous_value, target, maximum_delta)
        return self.previous_value
    def reset(self, value: Optional[float] = None) -> None:
        self.previous_value = value
@dataclass(frozen=True)
class GyroCalibration:
    roll_bias_dps: float
    pitch_bias_dps: float
    yaw_bias_dps: float
    roll_stddev_dps: float
    pitch_stddev_dps: float
    yaw_stddev_dps: float
    samples: int
    duration_seconds: float
    @property
    def maximum_stddev(self) -> float:
        return max(self.roll_stddev_dps, self.pitch_stddev_dps, self.yaw_stddev_dps)
    def apply(self, rate: AngularRate) -> AngularRate:
        return AngularRate(
            rate.roll_dps - self.roll_bias_dps,
            rate.pitch_dps - self.pitch_bias_dps,
            rate.yaw_dps - self.yaw_bias_dps,
        )
    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)
    @classmethod
    def zero(cls) -> "GyroCalibration":
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0)
class ComplementaryAttitudeEstimator:
    """滚转/俯仰互补滤波；偏航仅积分陀螺，因此会缓慢漂移。"""
    def __init__(self, config: SensorConfig) -> None:
        self.config = config
        self.attitude = Attitude()
        self.angular_rate = AngularRate()
        self.acceleration = Acceleration()
        self.calibration = GyroCalibration.zero()
        self.last_timestamp: Optional[float] = None
        self.updates = 0
    def reset(self, attitude: Optional[Attitude] = None) -> None:
        self.attitude = Attitude() if attitude is None else attitude
        self.angular_rate = AngularRate()
        self.acceleration = Acceleration()
        self.last_timestamp = None
        self.updates = 0
    def set_calibration(self, calibration: GyroCalibration) -> None:
        self.calibration = calibration
    def update(self, raw: RawImuSample) -> SensorSample:
        if not raw.acceleration.is_finite() or not raw.angular_rate.is_finite():
            raise ValueError("IMU原始数据包含非有限值")
        dt = self._calculate_dt(raw.timestamp)
        calibrated_rate = self.calibration.apply(raw.angular_rate)
        self.angular_rate = calibrated_rate
        self.acceleration = raw.acceleration
        acceleration_roll = self._acceleration_roll(raw.acceleration)
        acceleration_pitch = self._acceleration_pitch(raw.acceleration)
        if self.updates == 0 or dt <= 0.0:
            roll = acceleration_roll
            pitch = acceleration_pitch
            yaw = self.attitude.yaw
        else:
            alpha = self.config.complementary_alpha
            roll = self.attitude.roll + alpha * calibrated_rate.roll_dps * dt
            roll += (1.0 - alpha) * shortest_angle_delta(acceleration_roll, self.attitude.roll)
            pitch = alpha * (self.attitude.pitch + calibrated_rate.pitch_dps * dt)
            pitch += (1.0 - alpha) * (acceleration_pitch - self.attitude.pitch)
            yaw = self.attitude.yaw + calibrated_rate.yaw_dps * dt
        self.attitude = Attitude(wrap_degrees(roll), clamp(pitch, -90.0, 90.0), wrap_degrees(yaw))
        self.updates += 1
        return SensorSample(
            attitude=self.attitude,
            angular_rate=calibrated_rate,
            acceleration=raw.acceleration,
            timestamp=raw.timestamp,
            sequence=self.updates,
            source="mpu6050",
            calibrated=self.calibration.samples > 0,
        )
    def _calculate_dt(self, timestamp: float) -> float:
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return 0.0
        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp
        if dt < 0.0:
            return 0.0
        return min(dt, self.config.maximum_dt_seconds)
    @staticmethod
    def _acceleration_roll(acceleration: Acceleration) -> float:
        return math.degrees(math.atan2(acceleration.y_g, acceleration.z_g))
    @staticmethod
    def _acceleration_pitch(acceleration: Acceleration) -> float:
        lateral = math.hypot(acceleration.y_g, acceleration.z_g)
        return math.degrees(math.atan2(-acceleration.x_g, lateral))
class SensorSource:
    """统一真实IMU、仿真IMU与回放数据源接口。"""
    def __init__(self, bus: EventBus) -> None:
        self.bus = bus
        self.connected = False
    def connect(self) -> None:
        raise NotImplementedError
    def close(self) -> None:
        raise NotImplementedError
    def read(self) -> SensorSample:
        raise NotImplementedError
    def calibrate(self) -> GyroCalibration:
        return GyroCalibration.zero()
class MPU6050Sensor(SensorSource):
    """MPU6050：i2c-6、0x69、±4g、±2000dps、14字节burst读取。"""
    def __init__(self, config: SensorConfig, bus: EventBus, i2c_bus: Any = None) -> None:
        super().__init__(bus)
        self.config = config
        self.estimator = ComplementaryAttitudeEstimator(config)
        self._bus = i2c_bus
        self._owns_bus = i2c_bus is None
        self._sequence = 0
    def connect(self) -> None:
        if self.connected:
            return
        if self._bus is None:
            try:
                from smbus2 import SMBus
            except ImportError as error:
                raise RuntimeError("真实硬件模式需要安装smbus2: python -m pip install smbus2") from error
            try:
                self._bus = SMBus(self.config.i2c_bus)
            except Exception as error:
                raise RuntimeError(f"无法打开I2C总线{self.config.i2c_bus}: {error}") from error
        try:
            identity = self.read_register(MPU_REG_WHO_AM_I)
            if identity not in (0x68, 0x69):
                raise RuntimeError(f"MPU6050 WHO_AM_I异常: 0x{identity:02X}")
            self.write_register(MPU_REG_PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            self.write_register(MPU_REG_PWR_MGMT_1, 0x01)
            time.sleep(0.1)
            self.write_register(MPU_REG_SMPLRT_DIV, 0x04)
            self.write_register(MPU_REG_CONFIG, 0x03)
            self.write_register(MPU_REG_GYRO_CONFIG, 0x18)
            self.write_register(MPU_REG_ACCEL_CONFIG, 0x08)
            self.write_register(MPU_REG_PWR_MGMT_2, 0x00)
            time.sleep(0.05)
        except Exception:
            self.close()
            raise
        self.connected = True
        self.bus.emit(EventLevel.INFO, EventTopic.SENSOR, "MPU6050初始化完成")
    def close(self) -> None:
        self.connected = False
        if self._bus is not None and self._owns_bus:
            try:
                self._bus.close()
            except Exception as error:
                self.bus.emit(EventLevel.WARNING, EventTopic.SENSOR, f"关闭I2C时出现警告: {error}")
            self._bus = None
    def read_register(self, register: int) -> int:
        if self._bus is None:
            raise RuntimeError("I2C总线尚未打开")
        return int(self._bus.read_byte_data(self.config.i2c_address, register))
    def write_register(self, register: int, value: int) -> None:
        if self._bus is None:
            raise RuntimeError("I2C总线尚未打开")
        self._bus.write_byte_data(self.config.i2c_address, register, value)
    def read_raw(self) -> RawImuSample:
        if self._bus is None:
            raise RuntimeError("I2C总线尚未打开")
        data = self._bus.read_i2c_block_data(self.config.i2c_address, MPU_REG_ACCEL_XOUT_H, 14)
        if len(data) != 14:
            raise RuntimeError(f"MPU6050 burst读取长度异常: {len(data)}")
        words = [self._signed_word(data[index], data[index + 1]) for index in range(0, 14, 2)]
        acceleration = Acceleration(
            words[0] / self.config.accel_sensitivity,
            words[1] / self.config.accel_sensitivity,
            words[2] / self.config.accel_sensitivity,
        )
        temperature = words[3] / 340.0 + 36.53
        angular_rate = AngularRate(
            words[4] / self.config.gyro_sensitivity,
            words[5] / self.config.gyro_sensitivity,
            words[6] / self.config.gyro_sensitivity,
        )
        self._validate_raw_ranges(acceleration, angular_rate)
        return RawImuSample(acceleration, angular_rate, temperature, monotonic_timestamp())
    @staticmethod
    def _signed_word(high: int, low: int) -> int:
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value
    def _validate_raw_ranges(self, acceleration: Acceleration, rate: AngularRate) -> None:
        if max(abs(value) for value in acceleration.as_tuple()) > self.config.maximum_accel_g * 1.1:
            raise RuntimeError("加速度计读数超出配置量程")
        if max(abs(value) for value in rate.as_tuple()) > self.config.maximum_gyro_dps * 1.1:
            raise RuntimeError("陀螺仪读数超出配置量程")
    def read(self) -> SensorSample:
        if not self.connected:
            raise RuntimeError("MPU6050尚未连接")
        try:
            sample = self.estimator.update(self.read_raw())
        except Exception as error:
            self.bus.emit(EventLevel.ERROR, EventTopic.SENSOR, f"MPU6050读取失败: {error}")
            raise
        self._sequence += 1
        return dataclasses.replace(sample, sequence=self._sequence)
    def calibrate(self) -> GyroCalibration:
        if not self.connected:
            raise RuntimeError("校准前必须连接MPU6050")
        self.bus.emit(EventLevel.INFO, EventTopic.SENSOR, "陀螺仪校准开始，请保持机器人静止")
        rates: List[AngularRate] = []
        started = monotonic_timestamp()
        for _ in range(self.config.calibration_samples):
            raw = self.read_raw()
            rates.append(raw.angular_rate)
            time.sleep(self.config.calibration_delay_seconds)
        duration = monotonic_timestamp() - started
        roll_values = [rate.roll_dps for rate in rates]
        pitch_values = [rate.pitch_dps for rate in rates]
        yaw_values = [rate.yaw_dps for rate in rates]
        calibration = GyroCalibration(
            statistics.fmean(roll_values),
            statistics.fmean(pitch_values),
            statistics.fmean(yaw_values),
            statistics.pstdev(roll_values),
            statistics.pstdev(pitch_values),
            statistics.pstdev(yaw_values),
            len(rates),
            duration,
        )
        self.estimator.set_calibration(calibration)
        self.estimator.reset()
        self.bus.emit(
            EventLevel.INFO,
            EventTopic.SENSOR,
            f"陀螺仪校准完成，样本={calibration.samples}，最大标准差={calibration.maximum_stddev:.4f}dps",
        )
        return calibration
class SimulatedImuModel:
    """用于验证上位机逻辑的姿态对象，不代表真实Go1动力学。"""
    def __init__(self, geometry: RobotGeometry) -> None:
        self.geometry = geometry
        self.attitude = Attitude()
        self.rate = AngularRate()
        self.acceleration = Acceleration()
        self.external_roll_dps = 0.0
        self.external_pitch_dps = 0.0
        self.external_yaw_dps = 0.0
        self.forced_attitude: Optional[Attitude] = None
        self.response_per_second = 7.0
        self.walk_coupling = 0.15
    def set_target_response(self, target: Attitude, command: RobotCommand, dt: float) -> None:
        if self.forced_attitude is not None:
            self._apply_forced_attitude(dt)
            return
        if command == RobotCommand.STOP:
            response_target = self.attitude
            damping = 0.0
        else:
            response_target = target
            damping = self.response_per_second
        desired_roll_rate = (response_target.roll - self.attitude.roll) * damping + self.external_roll_dps
        desired_pitch_rate = (response_target.pitch - self.attitude.pitch) * damping + self.external_pitch_dps
        desired_yaw_rate = shortest_angle_delta(response_target.yaw, self.attitude.yaw) * damping + self.external_yaw_dps
        if command == RobotCommand.WALK:
            desired_roll_rate += target.roll * self.walk_coupling
        self.rate = AngularRate(desired_roll_rate, desired_pitch_rate, desired_yaw_rate)
        self._integrate(dt)
    def _apply_forced_attitude(self, dt: float) -> None:
        assert self.forced_attitude is not None
        if dt > 0.0:
            self.rate = AngularRate(
                shortest_angle_delta(self.forced_attitude.roll, self.attitude.roll) / dt,
                (self.forced_attitude.pitch - self.attitude.pitch) / dt,
                shortest_angle_delta(self.forced_attitude.yaw, self.attitude.yaw) / dt,
            )
        self.attitude = self.forced_attitude
    def _integrate(self, dt: float) -> None:
        if dt <= 0.0:
            return
        roll = self.attitude.roll + self.rate.roll_dps * dt
        pitch = self.attitude.pitch + self.rate.pitch_dps * dt
        yaw = self.attitude.yaw + self.rate.yaw_dps * dt
        self.attitude = Attitude(wrap_degrees(roll), clamp(pitch, -89.9, 89.9), wrap_degrees(yaw))
    def acceleration_from_attitude(self) -> Acceleration:
        roll = math.radians(self.attitude.roll)
        pitch = math.radians(self.attitude.pitch)
        x_g = -math.sin(pitch)
        y_g = math.cos(pitch) * math.sin(roll)
        z_g = math.cos(pitch) * math.cos(roll)
        self.acceleration = Acceleration(x_g, y_g, z_g)
        return self.acceleration
    def reset(self) -> None:
        self.attitude = Attitude()
        self.rate = AngularRate()
        self.acceleration = Acceleration()
        self.external_roll_dps = 0.0
        self.external_pitch_dps = 0.0
        self.external_yaw_dps = 0.0
        self.forced_attitude = None
class SimulatedSensor(SensorSource):
    def __init__(self, config: SensorConfig, geometry: RobotGeometry, bus: EventBus) -> None:
        super().__init__(bus)
        self.config = config
        self.model = SimulatedImuModel(geometry)
        self.command = RobotCommand.STOP
        self.target = Attitude()
        self.last_timestamp: Optional[float] = None
        self.sequence = 0
        self.random = random.Random(20260304)
        self.inject_invalid_sample = False
        self.suspended = False
        self.calibration = GyroCalibration.zero()
    def connect(self) -> None:
        self.connected = True
        self.last_timestamp = monotonic_timestamp()
        self.bus.emit(EventLevel.INFO, EventTopic.SENSOR, "仿真姿态传感器已连接")
    def close(self) -> None:
        self.connected = False
        self.bus.emit(EventLevel.INFO, EventTopic.SENSOR, "仿真姿态传感器已关闭")
    def set_control_state(self, command: RobotCommand, target: Attitude) -> None:
        self.command = RobotCommand(command)
        self.target = target
    def read(self) -> SensorSample:
        if not self.connected:
            raise RuntimeError("仿真传感器尚未连接")
        if self.suspended:
            raise TimeoutError("仿真传感器被设置为暂停")
        now = monotonic_timestamp()
        dt = 0.0 if self.last_timestamp is None else now - self.last_timestamp
        dt = min(max(dt, 0.0), self.config.maximum_dt_seconds)
        self.last_timestamp = now
        self.model.set_target_response(self.target, self.command, dt)
        acceleration = self.model.acceleration_from_attitude()
        noisy_acceleration = Acceleration(
            acceleration.x_g + self.random.gauss(0.0, self.config.simulation_noise_g),
            acceleration.y_g + self.random.gauss(0.0, self.config.simulation_noise_g),
            acceleration.z_g + self.random.gauss(0.0, self.config.simulation_noise_g),
        )
        noisy_rate = AngularRate(
            self.model.rate.roll_dps + self.random.gauss(0.0, self.config.simulation_noise_dps),
            self.model.rate.pitch_dps + self.random.gauss(0.0, self.config.simulation_noise_dps),
            self.model.rate.yaw_dps + self.random.gauss(0.0, self.config.simulation_noise_dps),
        )
        self.sequence += 1
        if self.inject_invalid_sample:
            self.inject_invalid_sample = False
            return SensorSample(
                Attitude(float("nan"), 0.0, 0.0),
                noisy_rate,
                noisy_acceleration,
                now,
                self.sequence,
                "simulation",
            )
        return SensorSample(
            self.model.attitude,
            noisy_rate,
            noisy_acceleration,
            now,
            self.sequence,
            "simulation",
        )
    def calibrate(self) -> GyroCalibration:
        self.calibration = GyroCalibration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.config.calibration_samples, 0.0)
        self.bus.emit(EventLevel.INFO, EventTopic.SENSOR, "仿真陀螺仪零偏已清零")
        return self.calibration
    def force_attitude(self, attitude: Optional[Attitude]) -> None:
        self.model.forced_attitude = attitude
    def disturb(self, roll_dps: float = 0.0, pitch_dps: float = 0.0, yaw_dps: float = 0.0) -> None:
        self.model.external_roll_dps = roll_dps
        self.model.external_pitch_dps = pitch_dps
        self.model.external_yaw_dps = yaw_dps
class ReplaySensor(SensorSource):
    """按记录时间戳回放姿态，用于离线验证界面与安全逻辑。"""
    def __init__(self, samples: Sequence[SensorSample], bus: EventBus, real_time: bool = False) -> None:
        super().__init__(bus)
        if not samples:
            raise ValueError("回放样本不能为空")
        self.samples = list(samples)
        self.real_time = real_time
        self.index = 0
        self.started_at = 0.0
        self.first_sample_time = samples[0].timestamp
    def connect(self) -> None:
        self.connected = True
        self.index = 0
        self.started_at = monotonic_timestamp()
        self.bus.emit(EventLevel.INFO, EventTopic.SENSOR, f"姿态回放已开始，共{len(self.samples)}个样本")
    def close(self) -> None:
        self.connected = False
    def read(self) -> SensorSample:
        if not self.connected:
            raise RuntimeError("回放传感器尚未连接")
        if self.index >= len(self.samples):
            raise EOFError("姿态回放已结束")
        sample = self.samples[self.index]
        if self.real_time:
            elapsed = monotonic_timestamp() - self.started_at
            sample_elapsed = max(0.0, sample.timestamp - self.first_sample_time)
            if elapsed < sample_elapsed:
                time.sleep(min(sample_elapsed - elapsed, 0.01))
                return self.read()
        self.index += 1
        return dataclasses.replace(sample, timestamp=monotonic_timestamp(), sequence=self.index, source="replay")
    @property
    def progress(self) -> float:
        return self.index / len(self.samples)
class SensorHealthMonitor:
    def __init__(self, config: SensorConfig) -> None:
        self.config = config
        self.last_sequence = -1
        self.last_timestamp = 0.0
        self.consecutive_errors = 0
        self.total_errors = 0
        self.maximum_gap_seconds = 0.0
    def evaluate(self, sample: SensorSample, now: Optional[float] = None) -> Tuple[bool, Optional[FaultCode]]:
        current = monotonic_timestamp() if now is None else now
        if not sample.is_valid():
            self.total_errors += 1
            self.consecutive_errors += 1
            return False, FaultCode.SENSOR_INVALID
        age = current - sample.timestamp
        if age > self.config.sensor_timeout_seconds:
            self.total_errors += 1
            self.consecutive_errors += 1
            return False, FaultCode.SENSOR_TIMEOUT
        if self.last_timestamp > 0.0:
            gap = sample.timestamp - self.last_timestamp
            self.maximum_gap_seconds = max(self.maximum_gap_seconds, gap)
        self.last_timestamp = sample.timestamp
        self.last_sequence = sample.sequence
        self.consecutive_errors = 0
        return True, None
    def record_exception(self, error: Exception) -> FaultCode:
        self.total_errors += 1
        self.consecutive_errors += 1
        if isinstance(error, TimeoutError):
            return FaultCode.SENSOR_TIMEOUT
        return FaultCode.SENSOR_DISCONNECTED
def create_sensor(config: AppConfig, bus: EventBus) -> SensorSource:
    if config.mode == RunMode.SIMULATION:
        return SimulatedSensor(config.sensor, config.geometry, bus)
    return MPU6050Sensor(config.sensor, bus)
@dataclass(frozen=True)
class SafetyDecision:
    command: RobotCommand
    level: SafetyLevel
    fault: FaultCode
    reason: str
    may_operate: bool
class SafetySupervisor:
    """默认停止、显式解锁、故障锁存；任何模式都不自动解除急停。"""
    def __init__(self, config: SafetyConfig, bus: EventBus) -> None:
        self.config = config
        self.bus = bus
        self._lock = threading.RLock()
        self._armed = not config.require_explicit_unlock
        self._latched_fault = FaultCode.NONE
        self._fault_reason = ""
        self._trip_count = 0
        self._last_decision = SafetyDecision(RobotCommand.STOP, SafetyLevel.NORMAL, FaultCode.NONE, "默认停止", False)
        self._current_requested_command = RobotCommand.STOP
    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed
    @property
    def fault(self) -> FaultCode:
        with self._lock:
            return self._latched_fault
    @property
    def locked(self) -> bool:
        with self._lock:
            return self._latched_fault != FaultCode.NONE
    @property
    def trip_count(self) -> int:
        with self._lock:
            return self._trip_count
    @property
    def last_decision(self) -> SafetyDecision:
        with self._lock:
            return self._last_decision
    def arm(self, sample: Optional[SensorSample] = None) -> Tuple[bool, str]:
        with self._lock:
            if self._latched_fault != FaultCode.NONE:
                return False, f"故障尚未复位: {fault_message(self._latched_fault)}"
            if sample is not None:
                if not sample.is_valid():
                    return False, "传感器数据无效，不能解锁"
                if self._is_fall_attitude(sample.attitude):
                    return False, "当前姿态超过安全角度，不能解锁"
                if sample.age_seconds() > self.config.sensor_timeout_seconds:
                    return False, "传感器数据过旧，不能解锁"
            self._armed = True
            self.bus.emit(EventLevel.WARNING, EventTopic.SAFETY, "控制输出已由用户显式解锁")
            return True, "控制输出已解锁"
    def disarm(self, reason: str = "用户锁定输出") -> None:
        with self._lock:
            changed = self._armed
            self._armed = False
            if changed:
                self.bus.emit(EventLevel.WARNING, EventTopic.SAFETY, reason)
    def emergency_stop(self, reason: str = "用户按下急停") -> None:
        with self._lock:
            self._armed = False
            self._latch_locked(FaultCode.MANUAL_ESTOP, reason)
            self.bus.emit(EventLevel.CRITICAL, EventTopic.SAFETY, reason)
    def trip(self, fault: FaultCode, reason: str) -> None:
        if fault == FaultCode.NONE:
            raise ValueError("不能锁存NONE故障")
        with self._lock:
            self._armed = False
            self._latch_locked(fault, reason)
    def clear_fault(self, sample: Optional[SensorSample] = None, confirmed: bool = False) -> Tuple[bool, str]:
        with self._lock:
            if not confirmed:
                return False, "清除锁存故障需要用户确认"
            if sample is not None:
                if not sample.is_valid():
                    return False, "传感器仍无效，不能复位"
                if sample.age_seconds() > self.config.sensor_timeout_seconds:
                    return False, "传感器仍超时，不能复位"
                if self._is_fall_attitude(sample.attitude):
                    return False, "机器人仍处于危险姿态，不能复位"
            previous = self._latched_fault
            self._latched_fault = FaultCode.NONE
            self._fault_reason = ""
            self._armed = False
            self.bus.emit(EventLevel.WARNING, EventTopic.SAFETY, f"故障{previous.value}已复位，输出保持锁定")
            return True, "故障已复位，请检查姿态后重新解锁"
    def evaluate(
        self,
        sample: Optional[SensorSample],
        target: Attitude,
        requested_command: RobotCommand,
        now: Optional[float] = None,
    ) -> SafetyDecision:
        current = monotonic_timestamp() if now is None else now
        with self._lock:
            self._current_requested_command = requested_command
            if self._latched_fault != FaultCode.NONE:
                return self._decision(SafetyLevel.LOCKED, self._latched_fault, self._fault_reason)
            if not target.is_finite():
                self._armed = False
                self._latch_locked(FaultCode.TARGET_INVALID, "目标姿态包含NaN或无穷大")
                return self._decision(SafetyLevel.LOCKED, FaultCode.TARGET_INVALID, self._fault_reason)
            target_limit = self.config.maximum_target_angle_degrees
            if max(abs(target.roll), abs(target.pitch)) > target_limit + 1e-6:
                self._armed = False
                self._latch_locked(FaultCode.TARGET_INVALID, f"目标横滚或俯仰超过{target_limit:.1f}度")
                return self._decision(SafetyLevel.LOCKED, FaultCode.TARGET_INVALID, self._fault_reason)
            if requested_command != RobotCommand.STOP and not self._armed:
                return self._decision(SafetyLevel.NORMAL, FaultCode.NONE, "输出未解锁，保持停止")
            if sample is None:
                if requested_command != RobotCommand.STOP:
                    self._armed = False
                    self._latch_locked(FaultCode.SENSOR_DISCONNECTED, "没有可用的姿态样本")
                    return self._decision(SafetyLevel.LOCKED, FaultCode.SENSOR_DISCONNECTED, self._fault_reason)
                return self._decision(SafetyLevel.WARNING, FaultCode.NONE, "等待姿态样本")
            if not sample.is_valid():
                self._armed = False
                self._latch_locked(FaultCode.SENSOR_INVALID, "姿态样本包含NaN或无穷大")
                return self._decision(SafetyLevel.LOCKED, FaultCode.SENSOR_INVALID, self._fault_reason)
            age = max(0.0, current - sample.timestamp)
            if age > self.config.sensor_timeout_seconds and requested_command != RobotCommand.STOP:
                self._armed = False
                self._latch_locked(FaultCode.SENSOR_TIMEOUT, f"姿态数据已{age:.3f}秒未更新")
                return self._decision(SafetyLevel.LOCKED, FaultCode.SENSOR_TIMEOUT, self._fault_reason)
            if self._is_fall_attitude(sample.attitude):
                self._armed = False
                reason = f"姿态达到Roll={sample.attitude.roll:.2f}°, Pitch={sample.attitude.pitch:.2f}°"
                self._latch_locked(FaultCode.FALL_DETECTED, reason)
                return self._decision(SafetyLevel.LOCKED, FaultCode.FALL_DETECTED, reason)
            if self._is_warning_attitude(sample.attitude):
                return self._decision(SafetyLevel.WARNING, FaultCode.NONE, "姿态接近安全阈值")
            return self._decision(SafetyLevel.NORMAL, FaultCode.NONE, "安全监控正常")
    def _decision(self, level: SafetyLevel, fault: FaultCode, reason: str) -> SafetyDecision:
        may_operate = self._armed and fault == FaultCode.NONE
        command = self._current_requested_command if may_operate else RobotCommand.STOP
        self._last_decision = SafetyDecision(command, level, fault, reason, may_operate)
        return self._last_decision
    def _latch_locked(self, fault: FaultCode, reason: str) -> None:
        if self._latched_fault == FaultCode.NONE:
            self._latched_fault = fault
            self._fault_reason = reason
            self._trip_count += 1
        elif self._latched_fault == FaultCode.MANUAL_ESTOP:
            self._fault_reason = reason
    def _is_fall_attitude(self, attitude: Attitude) -> bool:
        threshold = self.config.fall_angle_degrees
        return abs(attitude.roll) > threshold or abs(attitude.pitch) > threshold
    def _is_warning_attitude(self, attitude: Attitude) -> bool:
        threshold = self.config.warning_angle_degrees
        return abs(attitude.roll) > threshold or abs(attitude.pitch) > threshold
@dataclass(frozen=True)
class GaitParameters:
    frequency_hz: float = 0.3
    pitch_amplitude_degrees: float = 8.0
    roll_amplitude_degrees: float = 0.0
    yaw_amplitude_degrees: float = 0.0
    phase_offset_radians: float = 0.0
    duty_factor: float = 0.65
    step_length_m: float = 0.06
    step_height_m: float = 0.03
    def validate(self, safety: SafetyConfig) -> None:
        if not 0.0 < self.frequency_hz <= safety.maximum_walk_frequency_hz:
            raise ValueError("步态频率超出安全范围")
        if abs(self.pitch_amplitude_degrees) > safety.maximum_walk_amplitude_degrees:
            raise ValueError("俯仰步态摆幅超出安全范围")
        if abs(self.roll_amplitude_degrees) > safety.maximum_walk_amplitude_degrees:
            raise ValueError("横滚步态摆幅超出安全范围")
        if not 0.5 <= self.duty_factor < 1.0:
            raise ValueError("四足占空比应在[0.5,1.0)内")
        if self.step_length_m < 0.0 or self.step_height_m < 0.0:
            raise ValueError("步长和步高不能为负数")
class SineWalkGait:
    """仅生成ESP32所需姿态目标，不生成Go1电机位置指令。"""
    def __init__(self, parameters: GaitParameters) -> None:
        self.parameters = parameters
        self.phase_radians = parameters.phase_offset_radians
        self.enabled = False
        self.cycles = 0
    def start(self, reset_phase: bool = True) -> None:
        self.enabled = True
        if reset_phase:
            self.phase_radians = self.parameters.phase_offset_radians
            self.cycles = 0
    def stop(self) -> None:
        self.enabled = False
    def update(self, dt_seconds: float) -> Attitude:
        if dt_seconds < 0.0:
            raise ValueError("步态时间增量不能为负数")
        if not self.enabled:
            return Attitude()
        previous_phase = self.phase_radians
        phase_increment = 2.0 * math.pi * self.parameters.frequency_hz * dt_seconds
        self.phase_radians = (self.phase_radians + phase_increment) % (2.0 * math.pi)
        if self.phase_radians < previous_phase:
            self.cycles += 1
        sine = math.sin(self.phase_radians)
        cosine = math.cos(self.phase_radians)
        return Attitude(
            self.parameters.roll_amplitude_degrees * cosine,
            self.parameters.pitch_amplitude_degrees * sine,
            self.parameters.yaw_amplitude_degrees * math.sin(self.phase_radians * 0.5),
        )
    def normalized_phase(self) -> float:
        return self.phase_radians / (2.0 * math.pi)
@dataclass(frozen=True)
class FootReference:
    leg_name: str
    x_m: float
    y_m: float
    z_m: float
    stance: bool
    phase: float
class FourLegReferenceGenerator:
    """生成供界面观察的四足末端参考，不直接输出关节或电机命令。"""
    LEG_PHASES = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
    def __init__(self, geometry: RobotGeometry, parameters: GaitParameters) -> None:
        self.geometry = geometry
        self.parameters = parameters
    def foot_position(self, leg_name: str, phase: float) -> FootReference:
        if leg_name not in self.LEG_PHASES:
            raise ValueError(f"未知腿名称: {leg_name}")
        local_phase = (phase + self.LEG_PHASES[leg_name]) % 1.0
        sign_x = 1.0 if leg_name.startswith("F") else -1.0
        sign_y = 1.0 if leg_name.endswith("L") else -1.0
        hip_x = sign_x * self.geometry.hip_x_m
        hip_y = sign_y * self.geometry.hip_y_m
        stance = local_phase < self.parameters.duty_factor
        if stance:
            stance_ratio = local_phase / self.parameters.duty_factor
            x_offset = self.parameters.step_length_m * (0.5 - stance_ratio)
            z_offset = 0.0
        else:
            swing_ratio = (local_phase - self.parameters.duty_factor) / (1.0 - self.parameters.duty_factor)
            x_offset = self.parameters.step_length_m * (swing_ratio - 0.5)
            z_offset = self.parameters.step_height_m * math.sin(math.pi * swing_ratio)
        return FootReference(leg_name, hip_x + x_offset, hip_y, -self.geometry.nominal_height_m + z_offset, stance, local_phase)
    def update(self, phase: float) -> Dict[str, FootReference]:
        return {name: self.foot_position(name, phase) for name in self.LEG_PHASES}
class TargetPlanner:
    """管理手动目标、走路正弦目标和目标变化率限制。"""
    def __init__(self, control: ControlConfig, safety: SafetyConfig, bus: EventBus) -> None:
        self.control = control
        self.safety = safety
        self.bus = bus
        self.manual_target = Attitude()
        self.filtered_target = Attitude()
        self.last_output = Attitude()
        parameters = GaitParameters(control.walk_frequency_hz, control.walk_amplitude_degrees)
        parameters.validate(safety)
        self.walk = SineWalkGait(parameters)
        self.last_timestamp: Optional[float] = None
    def reset(self) -> None:
        self.manual_target = Attitude()
        self.filtered_target = Attitude()
        self.last_output = Attitude()
        self.walk.stop()
        self.last_timestamp = None
    def set_manual_target(self, target: Attitude) -> Attitude:
        if not target.is_finite():
            raise ValueError("目标姿态必须全部为有限数")
        limit = self.safety.maximum_target_angle_degrees
        limited = Attitude(
            saturate_signed(target.roll, limit),
            saturate_signed(target.pitch, limit),
            wrap_degrees(target.yaw),
        )
        self.manual_target = limited
        return limited
    def set_manual_axis(self, axis: str, value: float) -> Attitude:
        if axis not in ("roll", "pitch", "yaw"):
            raise ValueError("目标轴必须为roll、pitch或yaw")
        values = self.manual_target.as_dict()
        values[axis] = value
        return self.set_manual_target(Attitude(**values))
    def adjust_manual_axis(self, axis: str, delta: float) -> Attitude:
        values = self.manual_target.as_dict()
        if axis not in values:
            raise ValueError("目标轴必须为roll、pitch或yaw")
        values[axis] += delta
        return self.set_manual_target(Attitude(**values))
    def set_walk_parameters(self, frequency_hz: float, amplitude_degrees: float) -> None:
        parameters = dataclasses.replace(
            self.walk.parameters,
            frequency_hz=frequency_hz,
            pitch_amplitude_degrees=amplitude_degrees,
        )
        parameters.validate(self.safety)
        phase = self.walk.phase_radians
        enabled = self.walk.enabled
        self.walk = SineWalkGait(parameters)
        self.walk.phase_radians = phase
        self.walk.enabled = enabled
        self.bus.emit(EventLevel.INFO, EventTopic.CONTROL, f"步态参数更新: {frequency_hz:.2f}Hz, ±{amplitude_degrees:.2f}°")
    def update(self, command: RobotCommand, now: Optional[float] = None) -> Attitude:
        current = monotonic_timestamp() if now is None else now
        dt = self._dt(current)
        if command == RobotCommand.WALK:
            if not self.walk.enabled:
                self.walk.start()
            gait_offset = self.walk.update(dt)
            raw_target = Attitude(
                self.manual_target.roll + gait_offset.roll,
                self.manual_target.pitch + gait_offset.pitch,
                wrap_degrees(self.manual_target.yaw + gait_offset.yaw),
            )
        else:
            self.walk.stop()
            raw_target = self.manual_target if command == RobotCommand.BALANCE else Attitude()
        raw_target = raw_target.limited(self.safety.maximum_target_angle_degrees)
        maximum_delta = self.control.target_slew_dps * dt
        if self.last_timestamp is None:
            filtered = raw_target
        else:
            filtered = Attitude(
                move_toward(self.filtered_target.roll, raw_target.roll, maximum_delta),
                move_toward(self.filtered_target.pitch, raw_target.pitch, maximum_delta),
                move_toward(self.filtered_target.yaw, raw_target.yaw, maximum_delta),
            )
        self.filtered_target = filtered
        self.last_output = raw_target
        return filtered
    def _dt(self, current: float) -> float:
        if self.last_timestamp is None:
            self.last_timestamp = current
            return 0.0
        dt = max(0.0, current - self.last_timestamp)
        self.last_timestamp = current
        return dt
    def zero_manual_target(self) -> None:
        self.manual_target = Attitude()
        self.bus.emit(EventLevel.INFO, EventTopic.USER, "手动目标姿态已归零")
@dataclass(frozen=True)
class AttitudeError:
    roll_error: float
    pitch_error: float
    yaw_error: float
    roll_rms: float
    pitch_rms: float
    maximum_absolute_tilt: float
class AttitudeMetrics:
    def __init__(self, window_size: int = 200) -> None:
        self.roll_errors: Deque[float] = deque(maxlen=window_size)
        self.pitch_errors: Deque[float] = deque(maxlen=window_size)
        self.yaw_errors: Deque[float] = deque(maxlen=window_size)
        self.tilt_values: Deque[float] = deque(maxlen=window_size)
    def update(self, attitude: Attitude, target: Attitude) -> AttitudeError:
        roll_error = target.roll - attitude.roll
        pitch_error = target.pitch - attitude.pitch
        yaw_error = shortest_angle_delta(target.yaw, attitude.yaw)
        tilt = max(abs(attitude.roll), abs(attitude.pitch))
        self.roll_errors.append(roll_error)
        self.pitch_errors.append(pitch_error)
        self.yaw_errors.append(yaw_error)
        self.tilt_values.append(tilt)
        return self.current()
    def current(self) -> AttitudeError:
        return AttitudeError(
            self.roll_errors[-1] if self.roll_errors else 0.0,
            self.pitch_errors[-1] if self.pitch_errors else 0.0,
            self.yaw_errors[-1] if self.yaw_errors else 0.0,
            self._rms(self.roll_errors),
            self._rms(self.pitch_errors),
            max(self.tilt_values) if self.tilt_values else 0.0,
        )
    @staticmethod
    def _rms(values: Iterable[float]) -> float:
        materialized = list(values)
        if not materialized:
            return 0.0
        return math.sqrt(math.fsum(value * value for value in materialized) / len(materialized))
    def reset(self) -> None:
        self.roll_errors.clear()
        self.pitch_errors.clear()
        self.yaw_errors.clear()
        self.tilt_values.clear()
class SnapshotHistory:
    def __init__(self, capacity: int) -> None:
        self._items: Deque[ControlSnapshot] = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
    def append(self, snapshot: ControlSnapshot) -> None:
        with self._condition:
            self._items.append(snapshot)
            self._condition.notify_all()
    def latest(self) -> Optional[ControlSnapshot]:
        with self._lock:
            return self._items[-1] if self._items else None
    def recent(self, count: int = 100) -> List[ControlSnapshot]:
        if count < 0:
            raise ValueError("历史数量不能为负数")
        with self._lock:
            if count == 0:
                return []
            return list(self._items)[-count:]
    def since_sequence(self, sequence: int) -> List[ControlSnapshot]:
        with self._lock:
            return [item for item in self._items if item.sequence > sequence]
    def wait_for_new(self, sequence: int, timeout: float = 1.0) -> Optional[ControlSnapshot]:
        deadline = monotonic_timestamp() + timeout
        with self._condition:
            while True:
                latest = self._items[-1] if self._items else None
                if latest is not None and latest.sequence > sequence:
                    return latest
                remaining = deadline - monotonic_timestamp()
                if remaining <= 0.0:
                    return latest
                self._condition.wait(remaining)
    def clear(self) -> None:
        with self._lock:
            self._items.clear()
    def __len__(self) -> int:
        with self._lock:
            return len(self._items)
class CsvSnapshotRecorder:
    """控制快照CSV记录器，所有文件操作集中在引擎之外显式启停。"""
    FIELDNAMES = [
        "wall_time",
        "monotonic_time",
        "sequence",
        "command",
        "command_name",
        "roll",
        "pitch",
        "yaw",
        "target_roll",
        "target_pitch",
        "target_yaw",
        "filtered_roll",
        "filtered_pitch",
        "filtered_yaw",
        "sensor_age_seconds",
        "control_period_seconds",
        "frames_sent",
        "safety_level",
        "fault",
        "connection",
        "armed",
    ]
    def __init__(self, bus: EventBus) -> None:
        self.bus = bus
        self.path: Optional[Path] = None
        self._file: Any = None
        self._writer: Optional[csv.DictWriter] = None
        self._lock = threading.Lock()
        self.rows_written = 0
    @property
    def active(self) -> bool:
        return self._file is not None
    def start(self, path: Path) -> None:
        with self._lock:
            if self._file is not None:
                raise RuntimeError("CSV记录已经启动")
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = path.open("w", newline="", encoding="utf-8-sig")
            self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
            self._writer.writeheader()
            self.path = path
            self.rows_written = 0
        self.bus.emit(EventLevel.INFO, EventTopic.SYSTEM, f"CSV记录已开始: {path}")
    def record(self, snapshot: ControlSnapshot) -> None:
        with self._lock:
            if self._writer is None:
                return
            self._writer.writerow(self._row(snapshot))
            self.rows_written += 1
    def _row(self, snapshot: ControlSnapshot) -> Dict[str, Any]:
        return {
            "wall_time": wall_timestamp(),
            "monotonic_time": f"{snapshot.timestamp:.9f}",
            "sequence": snapshot.sequence,
            "command": int(snapshot.command),
            "command_name": command_name(snapshot.command),
            "roll": f"{snapshot.attitude.roll:.6f}",
            "pitch": f"{snapshot.attitude.pitch:.6f}",
            "yaw": f"{snapshot.attitude.yaw:.6f}",
            "target_roll": f"{snapshot.target.roll:.6f}",
            "target_pitch": f"{snapshot.target.pitch:.6f}",
            "target_yaw": f"{snapshot.target.yaw:.6f}",
            "filtered_roll": f"{snapshot.filtered_target.roll:.6f}",
            "filtered_pitch": f"{snapshot.filtered_target.pitch:.6f}",
            "filtered_yaw": f"{snapshot.filtered_target.yaw:.6f}",
            "sensor_age_seconds": f"{snapshot.sensor_age_seconds:.6f}",
            "control_period_seconds": f"{snapshot.control_period_seconds:.9f}",
            "frames_sent": snapshot.frames_sent,
            "safety_level": snapshot.safety_level.value,
            "fault": snapshot.fault.value,
            "connection": snapshot.connection.value,
            "armed": int(snapshot.armed),
        }
    def stop(self) -> None:
        with self._lock:
            if self._file is None:
                return
            self._file.flush()
            self._file.close()
            path = self.path
            rows = self.rows_written
            self._file = None
            self._writer = None
            self.path = None
        self.bus.emit(EventLevel.INFO, EventTopic.SYSTEM, f"CSV记录已停止: {path}，共{rows}行")
def read_snapshot_csv(path: Path) -> List[ControlSnapshot]:
    snapshots: List[ControlSnapshot] = []
    with path.open("r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        missing = set(CsvSnapshotRecorder.FIELDNAMES) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV缺少字段: {sorted(missing)}")
        for row in reader:
            snapshot = ControlSnapshot(
                timestamp=float(row["monotonic_time"]),
                sequence=int(row["sequence"]),
                command=RobotCommand(int(row["command"])),
                attitude=Attitude(float(row["roll"]), float(row["pitch"]), float(row["yaw"])),
                target=Attitude(float(row["target_roll"]), float(row["target_pitch"]), float(row["target_yaw"])),
                filtered_target=Attitude(float(row["filtered_roll"]), float(row["filtered_pitch"]), float(row["filtered_yaw"])),
                sensor_age_seconds=float(row["sensor_age_seconds"]),
                control_period_seconds=float(row["control_period_seconds"]),
                frames_sent=int(row["frames_sent"]),
                safety_level=SafetyLevel(row["safety_level"]),
                fault=FaultCode(row["fault"]),
                connection=ConnectionState(row["connection"]),
                armed=bool(int(row["armed"])),
            )
            snapshots.append(snapshot)
    return snapshots
def snapshots_to_sensor_samples(snapshots: Sequence[ControlSnapshot]) -> List[SensorSample]:
    samples: List[SensorSample] = []
    for snapshot in snapshots:
        sample = SensorSample(
            attitude=snapshot.attitude,
            angular_rate=AngularRate(),
            acceleration=Acceleration(),
            timestamp=snapshot.timestamp,
            sequence=snapshot.sequence,
            source="csv",
        )
        samples.append(sample)
    return samples
class ControlEngine:
    """70Hz主控引擎：只向ESP32发送V2姿态、目标和命令字。"""
    def __init__(
        self,
        config: AppConfig,
        bus: Optional[EventBus] = None,
        sensor: Optional[SensorSource] = None,
        transport: Optional[SerialTransport] = None,
    ) -> None:
        config.validate()
        self.config = config
        self.bus = EventBus() if bus is None else bus
        self.sensor = create_sensor(config, self.bus) if sensor is None else sensor
        self.transport = create_transport(config, self.bus) if transport is None else transport
        self.safety = SafetySupervisor(config.safety, self.bus)
        self.planner = TargetPlanner(config.control, config.safety, self.bus)
        self.health = SensorHealthMonitor(config.sensor)
        self.metrics = AttitudeMetrics()
        self.history = SnapshotHistory(config.control.history_capacity)
        self.recorder = CsvSnapshotRecorder(self.bus)
        self.statistics = RuntimeStatistics()
        self.receive_parser = StreamFrameParser()
        self._requested_command = RobotCommand.STOP
        self._last_sample: Optional[SensorSample] = None
        self._last_snapshot: Optional[ControlSnapshot] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._state_lock = threading.RLock()
        self._cycle_lock = threading.Lock()
        self._sequence = 0
        self._last_cycle_timestamp: Optional[float] = None
        self._last_error: Optional[str] = None
        self._closed = False
    @property
    def running(self) -> bool:
        with self._state_lock:
            return self._running
    @property
    def requested_command(self) -> RobotCommand:
        with self._state_lock:
            return self._requested_command
    @property
    def last_sample(self) -> Optional[SensorSample]:
        with self._state_lock:
            return self._last_sample
    @property
    def last_snapshot(self) -> Optional[ControlSnapshot]:
        with self._state_lock:
            return self._last_snapshot
    @property
    def last_error(self) -> Optional[str]:
        with self._state_lock:
            return self._last_error
    def start(self, calibrate_hardware: bool = False) -> None:
        with self._state_lock:
            if self._running:
                return
            if self._closed:
                raise RuntimeError("控制引擎已关闭，不能重复启动")
            self._stop_event.clear()
            self._last_error = None
        try:
            self.transport.connect()
            self.sensor.connect()
            if calibrate_hardware and self.config.mode == RunMode.HARDWARE:
                self.sensor.calibrate()
            initial_sample = self.sensor.read()
            self._accept_sample(initial_sample)
            self._send_immediate(RobotCommand.STOP, initial_sample.attitude, Attitude())
        except Exception as error:
            self._last_error = str(error)
            self.bus.emit(EventLevel.ERROR, EventTopic.SYSTEM, f"控制引擎启动失败: {error}")
            self._safe_shutdown_hardware()
            raise
        with self._state_lock:
            self._running = True
            self._thread = threading.Thread(target=self._run_loop, name="RobotControl70Hz", daemon=True)
            self._thread.start()
        self.bus.emit(EventLevel.INFO, EventTopic.SYSTEM, f"控制引擎已启动，目标频率{self.config.control.frequency_hz:.1f}Hz")
    def _run_loop(self) -> None:
        period = 1.0 / self.config.control.frequency_hz
        next_cycle = monotonic_timestamp()
        while not self._stop_event.is_set():
            cycle_started = monotonic_timestamp()
            next_cycle += period
            try:
                self.run_cycle(cycle_started)
            except Exception as error:
                self._last_error = f"{type(error).__name__}: {error}"
                self.safety.trip(FaultCode.INTERNAL_ERROR, self._last_error)
                self.bus.emit(EventLevel.CRITICAL, EventTopic.SYSTEM, f"控制循环异常: {error}")
                self.bus.emit(EventLevel.DEBUG, EventTopic.DIAGNOSTIC, traceback.format_exc())
                try:
                    self._send_immediate(RobotCommand.STOP, Attitude(), Attitude())
                except Exception:
                    pass
                break
            cycle_duration = monotonic_timestamp() - cycle_started
            self.statistics.last_cycle_duration = cycle_duration
            self.statistics.maximum_cycle_duration = max(self.statistics.maximum_cycle_duration, cycle_duration)
            if cycle_duration > self.config.safety.maximum_control_overrun_seconds:
                self.safety.trip(FaultCode.CONTROL_OVERRUN, f"单周期执行耗时{cycle_duration:.3f}秒")
            delay = next_cycle - monotonic_timestamp()
            if delay > 0.0:
                self._stop_event.wait(delay)
            else:
                missed_cycles = math.floor(-delay / period)
                next_cycle += missed_cycles * period
        with self._state_lock:
            self._running = False
        self.bus.emit(EventLevel.INFO, EventTopic.SYSTEM, "控制循环线程已退出")
    def run_cycle(self, now: Optional[float] = None) -> ControlSnapshot:
        """执行一次控制周期，独立公开便于离线单元测试。"""
        with self._cycle_lock:
            current = monotonic_timestamp() if now is None else now
            period = self._cycle_period(current)
            sample = self._read_sample_safely()
            requested = self.requested_command
            planning_command = requested if self.safety.armed else RobotCommand.STOP
            filtered_target = self.planner.update(planning_command, current)
            decision = self.safety.evaluate(sample, filtered_target, requested, current)
            effective_command = requested if decision.may_operate else RobotCommand.STOP
            if effective_command == RobotCommand.STOP and not self.safety.armed:
                filtered_target = Attitude()
            if isinstance(self.sensor, SimulatedSensor):
                self.sensor.set_control_state(effective_command, filtered_target)
            attitude = sample.attitude if sample is not None and sample.is_valid() else Attitude()
            sensor_age = sample.age_seconds(current) if sample is not None else float("inf")
            try:
                self._send_immediate(effective_command, attitude, filtered_target)
            except Exception as error:
                self.statistics.frames_failed += 1
                self._last_error = str(error)
                if self.config.safety.stop_on_serial_error:
                    self.safety.trip(FaultCode.SERIAL_FAILURE, str(error))
                    effective_command = RobotCommand.STOP
            if sample is not None and sample.is_valid():
                self.metrics.update(sample.attitude, filtered_target)
            self._sequence += 1
            snapshot = ControlSnapshot(
                timestamp=current,
                sequence=self._sequence,
                command=effective_command,
                attitude=attitude,
                target=self.planner.last_output if effective_command != RobotCommand.STOP else Attitude(),
                filtered_target=filtered_target,
                sensor_age_seconds=sensor_age,
                control_period_seconds=period,
                frames_sent=self.statistics.frames_sent,
                safety_level=SafetyLevel.LOCKED if self.safety.fault != FaultCode.NONE else self.safety.last_decision.level,
                fault=self.safety.fault,
                connection=self.transport.state,
                armed=self.safety.armed,
            )
            with self._state_lock:
                self._last_snapshot = snapshot
            self.history.append(snapshot)
            self.recorder.record(snapshot)
            self.statistics.control_cycles += 1
            return snapshot
    def _read_sample_safely(self) -> Optional[SensorSample]:
        try:
            sample = self.sensor.read()
        except EOFError as error:
            self.safety.trip(FaultCode.SENSOR_DISCONNECTED, str(error))
            return self._last_sample
        except Exception as error:
            fault = self.health.record_exception(error)
            self._last_error = str(error)
            if self.config.safety.stop_on_sensor_error:
                self.safety.trip(fault, f"传感器读取异常: {error}")
            return self._last_sample
        valid, fault = self.health.evaluate(sample)
        if not valid and fault is not None:
            if self.config.safety.stop_on_sensor_error:
                self.safety.trip(fault, fault_message(fault))
        self._accept_sample(sample)
        return sample
    def _accept_sample(self, sample: SensorSample) -> None:
        with self._state_lock:
            self._last_sample = sample
        self.statistics.sensor_updates += 1
    def _cycle_period(self, current: float) -> float:
        if self._last_cycle_timestamp is None:
            self._last_cycle_timestamp = current
            return 1.0 / self.config.control.frequency_hz
        period = max(0.0, current - self._last_cycle_timestamp)
        self._last_cycle_timestamp = current
        return period
    def _send_immediate(self, command: RobotCommand, attitude: Attitude, target: Attitude) -> bytes:
        frame = encode_v2_frame(attitude, target, command)
        self.statistics.frames_encoded += 1
        self.transport.send(frame)
        self.statistics.frames_sent += 1
        self.statistics.bytes_sent += len(frame)
        return frame
    def request_command(self, command: RobotCommand) -> Tuple[bool, str]:
        command = RobotCommand(command)
        if command != RobotCommand.STOP and not self.safety.armed:
            return False, "请先检查姿态并显式解锁控制输出"
        if self.safety.locked and command != RobotCommand.STOP:
            return False, f"请先复位故障: {fault_message(self.safety.fault)}"
        with self._state_lock:
            self._requested_command = command
        self.bus.emit(EventLevel.INFO, EventTopic.USER, f"请求控制命令: {command_name(command)}")
        return True, f"已请求{command_name(command)}模式"
    def arm(self) -> Tuple[bool, str]:
        result = self.safety.arm(self.last_sample)
        return result
    def disarm(self) -> None:
        with self._state_lock:
            self._requested_command = RobotCommand.STOP
        self.safety.disarm()
    def emergency_stop(self, reason: str = "用户按下急停") -> None:
        with self._state_lock:
            self._requested_command = RobotCommand.STOP
        self.safety.emergency_stop(reason)
        self.statistics.emergency_stops += 1
        attitude = self.last_sample.attitude if self.last_sample is not None else Attitude()
        try:
            self._send_immediate(RobotCommand.STOP, attitude, Attitude())
        except Exception as error:
            self.bus.emit(EventLevel.ERROR, EventTopic.SAFETY, f"急停帧发送失败: {error}")
    def clear_fault(self, confirmed: bool = False) -> Tuple[bool, str]:
        result = self.safety.clear_fault(self.last_sample, confirmed)
        if result[0]:
            self.planner.reset()
        return result
    def zero_target(self) -> None:
        self.planner.zero_manual_target()
    def set_target(self, roll: float, pitch: float, yaw: float) -> Attitude:
        return self.planner.set_manual_target(Attitude(roll, pitch, yaw))
    def set_target_axis(self, axis: str, value: float) -> Attitude:
        return self.planner.set_manual_axis(axis, value)
    def adjust_target(self, axis: str, delta: float) -> Attitude:
        return self.planner.adjust_manual_axis(axis, delta)
    def set_walk_parameters(self, frequency_hz: float, amplitude_degrees: float) -> None:
        self.planner.set_walk_parameters(frequency_hz, amplitude_degrees)
    def calibrate_sensor(self) -> GyroCalibration:
        if self.running and self.requested_command != RobotCommand.STOP:
            raise RuntimeError("校准前必须停止机器人")
        return self.sensor.calibrate()
    def start_recording(self, path: Optional[Path] = None) -> Path:
        if path is None:
            directory = Path(self.config.log_directory)
            filename = time.strftime("robot_%Y%m%d_%H%M%S.csv")
            path = directory / filename
        self.recorder.start(path)
        return path
    def stop_recording(self) -> None:
        self.recorder.stop()
    def poll_received_packets(self) -> List[V2Packet]:
        data = self.transport.receive()
        if not data:
            return []
        return self.receive_parser.feed(data)
    def run_for(self, duration_seconds: float) -> None:
        if duration_seconds < 0.0:
            raise ValueError("运行时长不能为负数")
        self.start()
        try:
            deadline = monotonic_timestamp() + duration_seconds
            while monotonic_timestamp() < deadline and self.running:
                time.sleep(min(0.05, max(0.0, deadline - monotonic_timestamp())))
        finally:
            self.stop()
    def stop(self) -> None:
        with self._state_lock:
            if self._closed:
                return
            thread = self._thread
        self._stop_event.set()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._state_lock:
            self._running = False
            self._requested_command = RobotCommand.STOP
        self.safety.disarm("控制引擎停止，输出锁定")
        self._safe_shutdown_hardware()
        self.recorder.stop()
        with self._state_lock:
            self._closed = True
        self.bus.emit(EventLevel.INFO, EventTopic.SYSTEM, "控制引擎已安全停止")
    def _safe_shutdown_hardware(self) -> None:
        attitude = self.last_sample.attitude if self.last_sample is not None else Attitude()
        if self.transport.state != ConnectionState.DISCONNECTED:
            successes = send_stop_frames(self.transport, attitude)
            self.bus.emit(EventLevel.INFO, EventTopic.SAFETY, f"退出前已发送{successes}个STOP帧")
        try:
            self.sensor.close()
        except Exception as error:
            self.bus.emit(EventLevel.WARNING, EventTopic.SENSOR, f"关闭传感器时出现警告: {error}")
        try:
            self.transport.close()
        except Exception as error:
            self.bus.emit(EventLevel.WARNING, EventTopic.TRANSPORT, f"关闭传输时出现警告: {error}")
    def status_dictionary(self) -> Dict[str, Any]:
        snapshot = self.last_snapshot
        return {
            "application": APP_NAME,
            "version": APP_VERSION,
            "mode": self.config.mode.value,
            "running": self.running,
            "requested_command": command_name(self.requested_command),
            "effective_command": command_name(snapshot.command) if snapshot else command_name(RobotCommand.STOP),
            "armed": self.safety.armed,
            "fault": self.safety.fault.value,
            "fault_message": fault_message(self.safety.fault),
            "attitude": snapshot.attitude.as_dict() if snapshot else Attitude().as_dict(),
            "target": snapshot.filtered_target.as_dict() if snapshot else Attitude().as_dict(),
            "frames_sent": self.statistics.frames_sent,
            "control_cycles": self.statistics.control_cycles,
            "uptime_seconds": self.statistics.uptime_seconds(),
            "recording": self.recorder.active,
            "last_error": self.last_error,
        }
    def __enter__(self) -> "ControlEngine":
        self.start()
        return self
    def __exit__(self, exception_type: Any, exception: Any, traceback_object: Any) -> None:
        self.stop()
@dataclass(frozen=True)
class DiagnosticItem:
    name: str
    passed: bool
    detail: str
    level: EventLevel = EventLevel.INFO
@dataclass(frozen=True)
class DiagnosticReport:
    created_at: float
    items: Tuple[DiagnosticItem, ...]
    @property
    def passed(self) -> bool:
        return all(item.passed for item in self.items)
    def format(self) -> str:
        lines = [f"诊断时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.created_at))}"]
        for item in self.items:
            marker = "通过" if item.passed else "失败"
            lines.append(f"[{marker}] {item.name}: {item.detail}")
        lines.append(f"总体结果: {'通过' if self.passed else '存在问题'}")
        return "\n".join(lines)
class SystemDiagnostics:
    def __init__(self, engine: ControlEngine) -> None:
        self.engine = engine
    def run(self) -> DiagnosticReport:
        items: List[DiagnosticItem] = []
        items.append(self._check_configuration())
        items.append(self._check_sensor())
        items.append(self._check_transport())
        items.append(self._check_safety())
        items.append(self._check_control_rate())
        items.append(self._check_protocol())
        items.append(self._check_recorder())
        return DiagnosticReport(wall_timestamp(), tuple(items))
    def _check_configuration(self) -> DiagnosticItem:
        try:
            self.engine.config.validate()
            return DiagnosticItem("配置", True, "所有配置范围有效")
        except Exception as error:
            return DiagnosticItem("配置", False, str(error), EventLevel.ERROR)
    def _check_sensor(self) -> DiagnosticItem:
        sample = self.engine.last_sample
        if sample is None:
            return DiagnosticItem("姿态传感器", False, "尚无有效样本", EventLevel.WARNING)
        if not sample.is_valid():
            return DiagnosticItem("姿态传感器", False, "最新样本包含非有限值", EventLevel.ERROR)
        age = sample.age_seconds()
        limit = self.engine.config.sensor.sensor_timeout_seconds
        passed = age <= limit
        return DiagnosticItem("姿态传感器", passed, f"来源={sample.source}，数据年龄={age:.4f}秒")
    def _check_transport(self) -> DiagnosticItem:
        transport = self.engine.transport
        passed = transport.is_connected
        detail = f"状态={transport.state.value}，发送={transport.sent_frames}，失败={transport.failures}"
        return DiagnosticItem("控制串口", passed, detail)
    def _check_safety(self) -> DiagnosticItem:
        fault = self.engine.safety.fault
        passed = fault == FaultCode.NONE
        detail = fault_message(fault)
        return DiagnosticItem("安全监控", passed, detail, EventLevel.ERROR if not passed else EventLevel.INFO)
    def _check_control_rate(self) -> DiagnosticItem:
        snapshot = self.engine.last_snapshot
        if snapshot is None:
            return DiagnosticItem("控制频率", False, "尚无控制快照", EventLevel.WARNING)
        actual = safe_divide(1.0, snapshot.control_period_seconds)
        expected = self.engine.config.control.frequency_hz
        tolerance = expected * 0.2
        passed = abs(actual - expected) <= tolerance or self.engine.statistics.control_cycles < 3
        return DiagnosticItem("控制频率", passed, f"最近周期频率={actual:.2f}Hz，目标={expected:.2f}Hz")
    def _check_protocol(self) -> DiagnosticItem:
        frame = encode_v2_frame(Attitude(1.0, 2.0, 3.0), Attitude(4.0, 5.0, 6.0), RobotCommand.BALANCE)
        packet = decode_v2_frame(frame)
        passed = len(frame) == 30 and packet.command == RobotCommand.BALANCE
        return DiagnosticItem("V2协议", passed, "30字节帧、25字节XOR和命令字自检")
    def _check_recorder(self) -> DiagnosticItem:
        recorder = self.engine.recorder
        detail = f"记录中，已写{recorder.rows_written}行" if recorder.active else "当前未记录"
        return DiagnosticItem("数据记录", True, detail)
@dataclass(frozen=True)
class SelfTestResult:
    name: str
    passed: bool
    duration_seconds: float
    detail: str
class SelfTestRunner:
    """不依赖第三方包的功能回归测试，默认全部在仿真对象上运行。"""
    def __init__(self) -> None:
        self.results: List[SelfTestResult] = []
    def run_all(self) -> List[SelfTestResult]:
        self.results.clear()
        tests = [getattr(self, name) for name in dir(self) if name.startswith("test_")]
        for test in sorted(tests, key=lambda function: function.__name__):
            started = monotonic_timestamp()
            try:
                detail = test()
                result = SelfTestResult(test.__name__, True, monotonic_timestamp() - started, detail or "通过")
            except Exception as error:
                detail = f"{type(error).__name__}: {error}"
                result = SelfTestResult(test.__name__, False, monotonic_timestamp() - started, detail)
            self.results.append(result)
        return self.results
    def format_results(self) -> str:
        lines: List[str] = []
        for result in self.results:
            status = "PASS" if result.passed else "FAIL"
            lines.append(f"{status:4s} {result.name:42s} {result.duration_seconds * 1000.0:8.2f}ms  {result.detail}")
        passed = sum(1 for result in self.results if result.passed)
        lines.append(f"合计: {passed}/{len(self.results)}项通过")
        return "\n".join(lines)
    @staticmethod
    def assert_close(actual: float, expected: float, tolerance: float = 1e-5) -> None:
        if abs(actual - expected) > tolerance:
            raise AssertionError(f"{actual}与{expected}不一致，容差{tolerance}")
    def test_01_numeric_helpers(self) -> str:
        self.assert_close(clamp(12.0, -1.0, 1.0), 1.0)
        self.assert_close(clamp(-12.0, -1.0, 1.0), -1.0)
        self.assert_close(wrap_degrees(190.0), -170.0)
        self.assert_close(shortest_angle_delta(-170.0, 170.0), 20.0)
        self.assert_close(move_toward(0.0, 10.0, 2.0), 2.0)
        self.assert_close(move_toward(0.0, 1.0, 2.0), 1.0)
        return "限幅、角度环绕和变化率辅助函数正确"
    def test_02_v1_protocol(self) -> str:
        expected = Attitude(1.25, -2.5, 3.75)
        frame = encode_v1_frame(expected)
        if len(frame) != 17:
            raise AssertionError("V1帧不是17字节")
        actual = decode_v1_frame(frame)
        for left, right in zip(actual.as_tuple(), expected.as_tuple()):
            self.assert_close(left, right)
        return "V1编解码与17字节长度正确"
    def test_03_v2_protocol_layout(self) -> str:
        attitude = Attitude(1.0, 2.0, 3.0)
        target = Attitude(4.0, 5.0, 6.0)
        frame = encode_v2_frame(attitude, target, RobotCommand.WALK)
        if len(frame) != 30:
            raise AssertionError("V2帧不是30字节")
        if frame[:2] != FRAME_HEAD or frame[-2:] != FRAME_TAIL:
            raise AssertionError("V2帧头帧尾错误")
        if frame[26] != 2:
            raise AssertionError("Cmd不在字节26")
        if frame[27] != xor_checksum(frame[2:27]):
            raise AssertionError("XOR未覆盖25字节有效载荷")
        values = struct.unpack("<fff", frame[2:14])
        self.assert_close(values[0], 1.0)
        self.assert_close(values[1], 2.0)
        self.assert_close(values[2], 3.0)
        targets = struct.unpack("<fff", frame[14:26])
        self.assert_close(targets[0], 4.0)
        self.assert_close(targets[1], 5.0)
        self.assert_close(targets[2], 6.0)
        packet = decode_v2_frame(frame)
        if packet.command != RobotCommand.WALK:
            raise AssertionError("V2命令字解码错误")
        return "30字节布局、偏移、六个float32、Cmd和XOR正确"
    def test_04_v2_rejects_corruption(self) -> str:
        frame = bytearray(encode_v2_frame(Attitude(), Attitude(), RobotCommand.BALANCE))
        frame[10] ^= 0x01
        try:
            decode_v2_frame(bytes(frame))
        except ProtocolError:
            return "损坏帧被正确拒绝"
        raise AssertionError("损坏帧未被拒绝")
    def test_05_stream_parser_fragmentation(self) -> str:
        first = encode_v2_frame(Attitude(1.0, 2.0, 3.0), Attitude(), RobotCommand.BALANCE)
        second = encode_v2_frame(Attitude(4.0, 5.0, 6.0), Attitude(), RobotCommand.WALK)
        stream = b"noise" + first + b"\x00" + second
        parser = StreamFrameParser()
        packets: List[V2Packet] = []
        for value in stream:
            packets.extend(parser.feed(bytes((value,))))
        if len(packets) != 2:
            raise AssertionError(f"逐字节拆包应得到2帧，实际{len(packets)}")
        if packets[0].command != RobotCommand.BALANCE or packets[1].command != RobotCommand.WALK:
            raise AssertionError("拆包后的命令字错误")
        return "噪声重同步和逐字节拆包正确"
    def test_06_complementary_estimator(self) -> str:
        config = SensorConfig()
        estimator = ComplementaryAttitudeEstimator(config)
        now = monotonic_timestamp()
        raw = RawImuSample(Acceleration(), AngularRate(), 25.0, now)
        first = estimator.update(raw)
        self.assert_close(first.attitude.roll, 0.0)
        self.assert_close(first.attitude.pitch, 0.0)
        raw_second = RawImuSample(Acceleration(), AngularRate(10.0, 0.0, 2.0), 25.0, now + 0.01)
        second = estimator.update(raw_second)
        if second.attitude.roll <= 0.0 or second.attitude.yaw <= 0.0:
            raise AssertionError("陀螺仪积分未生效")
        return "静止初始化与陀螺积分正确"
    def test_07_safety_default_and_estop(self) -> str:
        bus = EventBus()
        safety = SafetySupervisor(SafetyConfig(), bus)
        sample = SensorSample(Attitude(), AngularRate(), Acceleration(), monotonic_timestamp(), 1, "test")
        decision = safety.evaluate(sample, Attitude(), RobotCommand.BALANCE)
        if decision.may_operate:
            raise AssertionError("默认状态不应允许输出")
        if not safety.arm(sample)[0]:
            raise AssertionError("安全姿态下解锁失败")
        decision = safety.evaluate(sample, Attitude(), RobotCommand.BALANCE)
        if not decision.may_operate:
            raise AssertionError("解锁后未允许输出")
        safety.emergency_stop("自检急停")
        decision = safety.evaluate(sample, Attitude(), RobotCommand.BALANCE)
        if decision.may_operate or safety.fault != FaultCode.MANUAL_ESTOP:
            raise AssertionError("急停未锁存")
        if safety.clear_fault(sample, confirmed=False)[0]:
            raise AssertionError("未确认也能清除故障")
        if not safety.clear_fault(sample, confirmed=True)[0]:
            raise AssertionError("确认后无法清除故障")
        if safety.armed:
            raise AssertionError("故障复位后不应自动解锁")
        return "默认停止、急停锁存和确认复位正确"
    def test_08_safety_fall_latch(self) -> str:
        safety = SafetySupervisor(SafetyConfig(), EventBus())
        safe = SensorSample(Attitude(), AngularRate(), Acceleration(), monotonic_timestamp(), 1, "test")
        safety.arm(safe)
        fallen = dataclasses.replace(safe, attitude=Attitude(31.0, 0.0, 0.0))
        decision = safety.evaluate(fallen, Attitude(), RobotCommand.WALK)
        if decision.may_operate or safety.fault != FaultCode.FALL_DETECTED:
            raise AssertionError("超过30度未触发摔倒锁存")
        if safety.clear_fault(fallen, confirmed=True)[0]:
            raise AssertionError("危险姿态下故障被错误复位")
        return "摔倒阈值和危险姿态复位拒绝正确"
    def test_09_target_planner_and_gait(self) -> str:
        bus = EventBus()
        planner = TargetPlanner(ControlConfig(), SafetyConfig(), bus)
        limited = planner.set_manual_target(Attitude(100.0, -100.0, 370.0))
        self.assert_close(limited.roll, 20.0)
        self.assert_close(limited.pitch, -20.0)
        self.assert_close(limited.yaw, 10.0)
        now = monotonic_timestamp()
        planner.update(RobotCommand.WALK, now)
        planner.update(RobotCommand.WALK, now + 0.1)
        if planner.walk.normalized_phase() <= 0.0:
            raise AssertionError("步态相位未推进")
        planner.update(RobotCommand.STOP, now + 0.2)
        if planner.walk.enabled:
            raise AssertionError("停止后步态仍启用")
        return "目标限幅、偏航环绕和步态相位正确"
    def test_10_engine_requires_arm(self) -> str:
        config = AppConfig()
        bus = EventBus()
        engine = ControlEngine(config, bus)
        engine.transport.connect()
        engine.sensor.connect()
        try:
            accepted, _ = engine.request_command(RobotCommand.BALANCE)
            if accepted:
                raise AssertionError("未解锁却接受平衡命令")
            snapshot = engine.run_cycle()
            if snapshot.command != RobotCommand.STOP:
                raise AssertionError("未解锁时输出不是STOP")
            if not engine.arm()[0]:
                raise AssertionError("仿真引擎解锁失败")
            engine.request_command(RobotCommand.BALANCE)
            snapshot = engine.run_cycle()
            if snapshot.command != RobotCommand.BALANCE:
                raise AssertionError("解锁后未输出BALANCE")
            engine.request_command(RobotCommand.WALK)
            snapshot = engine.run_cycle()
            if snapshot.command != RobotCommand.WALK:
                raise AssertionError("走路命令未保留为WALK")
        finally:
            engine.stop()
        return "引擎默认STOP、解锁后BALANCE/WALK正确"
    def test_11_engine_emergency_stop(self) -> str:
        engine = ControlEngine(AppConfig(), EventBus())
        engine.transport.connect()
        engine.sensor.connect()
        try:
            engine._accept_sample(engine.sensor.read())
            engine.arm()
            engine.request_command(RobotCommand.BALANCE)
            engine.run_cycle()
            engine.emergency_stop("自检急停")
            snapshot = engine.run_cycle()
            if snapshot.command != RobotCommand.STOP:
                raise AssertionError("急停后仍发送非STOP命令")
            transport = engine.transport
            assert isinstance(transport, SimulatedSerialTransport)
            last_packet = decode_v2_frame(transport.last_frame() or b"")
            if last_packet.command != RobotCommand.STOP:
                raise AssertionError("最后线上帧不是STOP")
        finally:
            engine.stop()
        return "急停后线上命令与锁存状态均为STOP"
    def test_12_engine_fall_detection(self) -> str:
        engine = ControlEngine(AppConfig(), EventBus())
        engine.transport.connect()
        engine.sensor.connect()
        try:
            engine._accept_sample(engine.sensor.read())
            engine.arm()
            engine.request_command(RobotCommand.BALANCE)
            sensor = engine.sensor
            assert isinstance(sensor, SimulatedSensor)
            sensor.force_attitude(Attitude(35.0, 0.0, 0.0))
            snapshot = engine.run_cycle()
            if snapshot.fault != FaultCode.FALL_DETECTED or snapshot.command != RobotCommand.STOP:
                raise AssertionError("仿真摔倒未锁存STOP")
        finally:
            engine.stop()
        return "仿真摔倒触发故障锁存和STOP"
    def test_13_serial_failure_trip(self) -> str:
        engine = ControlEngine(AppConfig(), EventBus())
        transport = engine.transport
        assert isinstance(transport, SimulatedSerialTransport)
        transport.connect()
        engine.sensor.connect()
        try:
            engine._accept_sample(engine.sensor.read())
            engine.arm()
            engine.request_command(RobotCommand.BALANCE)
            transport.fail_next_send = True
            snapshot = engine.run_cycle()
            if snapshot.fault != FaultCode.SERIAL_FAILURE or snapshot.command != RobotCommand.STOP:
                raise AssertionError("串口故障未锁存STOP")
        finally:
            engine.stop()
        return "注入串口故障后锁存SERIAL_FAILURE"
    def test_14_csv_round_trip(self) -> str:
        import tempfile
        engine = ControlEngine(AppConfig(), EventBus())
        engine.transport.connect()
        engine.sensor.connect()
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "snapshot.csv"
                engine.start_recording(path)
                engine.run_cycle()
                engine.stop_recording()
                snapshots = read_snapshot_csv(path)
                if len(snapshots) != 1:
                    raise AssertionError("CSV回放快照数量错误")
                if snapshots[0].command != RobotCommand.STOP:
                    raise AssertionError("CSV命令字段错误")
        finally:
            engine.stop()
        return "CSV写入、读取和字段转换正确"
    def test_15_configuration_validation(self) -> str:
        config = AppConfig()
        config.validate()
        invalid = dataclasses.replace(config, control=ControlConfig(frequency_hz=0.0))
        try:
            invalid.validate()
        except ValueError:
            return "有效配置通过，无效频率被拒绝"
        raise AssertionError("无效控制频率未拒绝")
    def test_16_threaded_simulation(self) -> str:
        engine = ControlEngine(AppConfig(), EventBus())
        engine.start()
        try:
            time.sleep(0.08)
            if engine.statistics.control_cycles < 2:
                raise AssertionError("后台控制循环未运行")
            if not engine.running:
                raise AssertionError("后台线程提前退出")
        finally:
            engine.stop()
        transport = engine.transport
        assert isinstance(transport, SimulatedSerialTransport)
        if transport.last_frame() is None:
            raise AssertionError("退出时未发送停止帧")
        packet = decode_v2_frame(transport.last_frame() or b"")
        if packet.command != RobotCommand.STOP:
            raise AssertionError("退出最后一帧不是STOP")
        return "短时70Hz仿真与退出STOP正确"
class RobotShell(cmd.Cmd):
    """交互式终端，所有命令均调用同一控制引擎。"""
    intro = "输入help查看命令；默认STOP，balance/walk前必须先arm。"
    prompt = "robot> "
    def __init__(self, engine: ControlEngine) -> None:
        super().__init__()
        self.engine = engine
        self._event_subscription = engine.bus.subscribe(self._print_event)
    def _print_event(self, event: SystemEvent) -> None:
        if event.level >= EventLevel.WARNING:
            self.stdout.write(event.format() + "\n")
            self.stdout.flush()
    def emptyline(self) -> None:
        return
    def _print_result(self, result: Tuple[bool, str]) -> None:
        self.stdout.write(result[1] + "\n")
    def _guard(self, action: Callable[[], Any]) -> None:
        try:
            action()
        except Exception as error:
            self.stdout.write(f"错误: {error}\n")
    def do_arm(self, argument: str) -> None:
        """arm：检查当前姿态并显式解锁控制输出。"""
        self._print_result(self.engine.arm())
    def do_disarm(self, argument: str) -> None:
        """disarm：锁定输出并回到STOP。"""
        self.engine.disarm()
        self.stdout.write("输出已锁定\n")
    def do_stop(self, argument: str) -> None:
        """stop：请求普通停止，不清除解锁状态。"""
        self._print_result(self.engine.request_command(RobotCommand.STOP))
    def do_balance(self, argument: str) -> None:
        """balance：请求平衡模式，需先arm。"""
        self._print_result(self.engine.request_command(RobotCommand.BALANCE))
    def do_walk(self, argument: str) -> None:
        """walk：请求正弦姿态步态模式，需先arm。"""
        self._print_result(self.engine.request_command(RobotCommand.WALK))
    def do_estop(self, argument: str) -> None:
        """estop：立即急停并锁存，复位必须显式确认。"""
        self.engine.emergency_stop("终端用户急停")
        self.stdout.write("急停已锁存\n")
    def do_reset(self, argument: str) -> None:
        """reset：确认清除故障；不会自动解锁。"""
        self._print_result(self.engine.clear_fault(confirmed=True))
    def do_target(self, argument: str) -> None:
        """target ROLL PITCH YAW：设置目标角度（度）。"""
        def action() -> None:
            parts = shlex.split(argument)
            if len(parts) != 3:
                raise ValueError("用法: target ROLL PITCH YAW")
            values = [float(part) for part in parts]
            target = self.engine.set_target(*values)
            self.stdout.write(json.dumps(target.as_dict(), ensure_ascii=False) + "\n")
        self._guard(action)
    def do_nudge(self, argument: str) -> None:
        """nudge AXIS DELTA：相对调整目标，例如nudge pitch 2。"""
        def action() -> None:
            parts = shlex.split(argument)
            if len(parts) != 2:
                raise ValueError("用法: nudge roll|pitch|yaw DELTA")
            target = self.engine.adjust_target(parts[0], float(parts[1]))
            self.stdout.write(json.dumps(target.as_dict(), ensure_ascii=False) + "\n")
        self._guard(action)
    def do_zero(self, argument: str) -> None:
        """zero：目标姿态归零。"""
        self.engine.zero_target()
        self.stdout.write("目标已归零\n")
    def do_walkcfg(self, argument: str) -> None:
        """walkcfg HZ AMPLITUDE：设置走路频率和俯仰摆幅。"""
        def action() -> None:
            parts = shlex.split(argument)
            if len(parts) != 2:
                raise ValueError("用法: walkcfg HZ AMPLITUDE_DEG")
            self.engine.set_walk_parameters(float(parts[0]), float(parts[1]))
        self._guard(action)
    def do_status(self, argument: str) -> None:
        """status：显示JSON运行状态。"""
        status = self.engine.status_dictionary()
        self.stdout.write(json.dumps(status, ensure_ascii=False, indent=2) + "\n")
    def do_diag(self, argument: str) -> None:
        """diag：运行在线诊断。"""
        self.stdout.write(SystemDiagnostics(self.engine).run().format() + "\n")
    def do_protocol(self, argument: str) -> None:
        """protocol：显示最近发送帧的字段解释。"""
        def action() -> None:
            transport = self.engine.transport
            if not isinstance(transport, SimulatedSerialTransport):
                raise RuntimeError("该诊断命令仅在仿真串口提供最近帧")
            frame = transport.last_frame()
            if frame is None:
                raise RuntimeError("尚未发送任何帧")
            explanation = explain_v2_frame(frame)
            self.stdout.write(json.dumps(explanation, ensure_ascii=False, indent=2) + "\n")
        self._guard(action)
    def do_record(self, argument: str) -> None:
        """record start [PATH] | record stop：控制CSV记录。"""
        def action() -> None:
            parts = shlex.split(argument)
            operation = parts[0] if parts else "start"
            if operation == "stop":
                self.engine.stop_recording()
                return
            if operation != "start":
                raise ValueError("用法: record start [PATH] | record stop")
            path = Path(parts[1]) if len(parts) > 1 else None
            actual = self.engine.start_recording(path)
            self.stdout.write(f"记录文件: {actual}\n")
        self._guard(action)
    def do_calibrate(self, argument: str) -> None:
        """calibrate：校准时必须保持机器人静止且处于STOP。"""
        def action() -> None:
            calibration = self.engine.calibrate_sensor()
            self.stdout.write(json.dumps(calibration.as_dict(), ensure_ascii=False, indent=2) + "\n")
        self._guard(action)
    def do_simfall(self, argument: str) -> None:
        """simfall ROLL PITCH YAW：仿真强制姿态，用于验证摔倒保护。"""
        def action() -> None:
            if not isinstance(self.engine.sensor, SimulatedSensor):
                raise RuntimeError("仅仿真模式支持simfall")
            parts = shlex.split(argument)
            if len(parts) != 3:
                raise ValueError("用法: simfall ROLL PITCH YAW")
            attitude = Attitude(*(float(value) for value in parts))
            self.engine.sensor.force_attitude(attitude)
        self._guard(action)
    def do_simclear(self, argument: str) -> None:
        """simclear：取消仿真强制姿态和外部扰动。"""
        if isinstance(self.engine.sensor, SimulatedSensor):
            self.engine.sensor.force_attitude(None)
            self.engine.sensor.disturb()
            self.engine.sensor.model.reset()
            self.stdout.write("仿真扰动已清除\n")
    def do_simdisturb(self, argument: str) -> None:
        """simdisturb ROLL_DPS PITCH_DPS YAW_DPS：添加仿真角速度扰动。"""
        def action() -> None:
            if not isinstance(self.engine.sensor, SimulatedSensor):
                raise RuntimeError("仅仿真模式支持simdisturb")
            parts = shlex.split(argument)
            if len(parts) != 3:
                raise ValueError("用法: simdisturb ROLL_DPS PITCH_DPS YAW_DPS")
            self.engine.sensor.disturb(*(float(value) for value in parts))
        self._guard(action)
    def do_events(self, argument: str) -> None:
        """events [COUNT]：显示最近事件。"""
        try:
            count = int(argument) if argument.strip() else 20
        except ValueError:
            self.stdout.write("数量必须为整数\n")
            return
        for event in self.engine.bus.history()[-count:]:
            self.stdout.write(event.format() + "\n")
    def do_quit(self, argument: str) -> bool:
        """quit：发送停止帧并退出。"""
        self._event_subscription()
        self.engine.stop()
        self.stdout.write("已安全退出\n")
        return True
    def do_exit(self, argument: str) -> bool:
        """exit：同quit。"""
        return self.do_quit(argument)
    def do_EOF(self, argument: str) -> bool:
        self.stdout.write("\n")
        return self.do_quit(argument)
def print_event_to_console(event: SystemEvent) -> None:
    print(event.format(), flush=True)
def run_headless(engine: ControlEngine, duration: float, json_status: bool = False) -> int:
    subscription = engine.bus.subscribe(print_event_to_console)
    engine.start()
    try:
        if duration > 0.0:
            deadline = monotonic_timestamp() + duration
            while monotonic_timestamp() < deadline and engine.running:
                time.sleep(min(0.1, deadline - monotonic_timestamp()))
        else:
            while engine.running:
                time.sleep(0.2)
        status = engine.status_dictionary()
        if json_status:
            print(json.dumps(status, ensure_ascii=False, indent=2))
        else:
            print(f"运行{status['uptime_seconds']:.2f}s，周期{status['control_cycles']}，发送{status['frames_sent']}帧")
        return 0 if engine.safety.fault == FaultCode.NONE else 2
    except KeyboardInterrupt:
        print("收到Ctrl+C，正在停止")
        return 0
    finally:
        engine.stop()
        subscription()
class AttitudeCanvas:
    """Tkinter人工姿态仪，仅显示姿态，不执行控制计算。"""
    def __init__(self, tk_module: Any, parent: Any, width: int = 390, height: int = 300) -> None:
        self.tk = tk_module
        self.width = width
        self.height = height
        self.canvas = tk_module.Canvas(parent, width=width, height=height, background="#111827", highlightthickness=0)
        self.attitude = Attitude()
    def pack(self, **options: Any) -> None:
        self.canvas.pack(**options)
    def update_attitude(self, attitude: Attitude) -> None:
        self.attitude = attitude
        self.draw()
    def draw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        center_x = self.width / 2.0
        center_y = self.height / 2.0
        radius = min(self.width, self.height) * 0.39
        canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, fill="#1f2937", outline="#94a3b8", width=2)
        pitch_offset = clamp(self.attitude.pitch, -45.0, 45.0) * 2.0
        roll_radians = math.radians(self.attitude.roll)
        self._draw_horizon(center_x, center_y + pitch_offset, roll_radians, radius)
        for pitch_mark in range(-30, 31, 10):
            y = center_y + pitch_mark * 2.0
            half_width = 28 if pitch_mark == 0 else 18
            canvas.create_line(center_x - half_width, y, center_x + half_width, y, fill="#cbd5e1")
            if pitch_mark != 0:
                canvas.create_text(center_x + half_width + 18, y, text=str(-pitch_mark), fill="#cbd5e1", font=("Arial", 8))
        canvas.create_line(center_x - 65, center_y, center_x - 18, center_y, fill="#fbbf24", width=3)
        canvas.create_line(center_x + 18, center_y, center_x + 65, center_y, fill="#fbbf24", width=3)
        canvas.create_line(center_x - 18, center_y, center_x, center_y + 12, fill="#fbbf24", width=3)
        canvas.create_line(center_x + 18, center_y, center_x, center_y + 12, fill="#fbbf24", width=3)
        canvas.create_text(center_x, 22, text=f"R {self.attitude.roll:+.2f}°  P {self.attitude.pitch:+.2f}°  Y {self.attitude.yaw:+.2f}°", fill="#f8fafc", font=("Arial", 11, "bold"))
    def _draw_horizon(self, center_x: float, center_y: float, roll: float, radius: float) -> None:
        canvas = self.canvas
        length = radius * 1.5
        cosine = math.cos(roll)
        sine = math.sin(roll)
        left_x = center_x - length * cosine
        left_y = center_y - length * sine
        right_x = center_x + length * cosine
        right_y = center_y + length * sine
        canvas.create_line(left_x, left_y, right_x, right_y, fill="#e2e8f0", width=2)
        for offset in range(-80, 81, 20):
            normal_x = -sine * offset
            normal_y = cosine * offset
            x1 = left_x + normal_x
            y1 = left_y + normal_y
            x2 = right_x + normal_x
            y2 = right_y + normal_y
            color = "#38bdf8" if offset < 0 else "#b45309"
            canvas.create_line(x1, y1, x2, y2, fill=color, width=1)
class HistoryPlotCanvas:
    """无第三方绘图依赖的三轴姿态历史曲线。"""
    COLORS = {"roll": "#ef4444", "pitch": "#22c55e", "yaw": "#3b82f6", "target": "#fbbf24"}
    def __init__(self, tk_module: Any, parent: Any, width: int = 900, height: int = 330) -> None:
        self.width = width
        self.height = height
        self.canvas = tk_module.Canvas(parent, width=width, height=height, background="#0f172a", highlightthickness=0)
        self.maximum_abs_angle = 30.0
    def pack(self, **options: Any) -> None:
        self.canvas.pack(**options)
    def update_history(self, snapshots: Sequence[ControlSnapshot]) -> None:
        self.canvas.delete("all")
        self._draw_grid()
        if len(snapshots) < 2:
            self.canvas.create_text(self.width / 2, self.height / 2, text="等待姿态历史数据", fill="#94a3b8", font=("Arial", 14))
            return
        maximum = max(
            abs(value)
            for snapshot in snapshots
            for value in (*snapshot.attitude.as_tuple(), *snapshot.filtered_target.as_tuple())
        )
        self.maximum_abs_angle = max(10.0, maximum * 1.15)
        self._draw_series(snapshots, "roll", lambda item: item.attitude.roll)
        self._draw_series(snapshots, "pitch", lambda item: item.attitude.pitch)
        self._draw_series(snapshots, "yaw", lambda item: item.attitude.yaw)
        self._draw_series(snapshots, "target", lambda item: item.filtered_target.pitch, dashed=True)
        self._draw_legend()
    def _draw_grid(self) -> None:
        for index in range(7):
            y = 25 + index * (self.height - 50) / 6.0
            self.canvas.create_line(45, y, self.width - 15, y, fill="#334155")
            angle = self.maximum_abs_angle * (1.0 - index / 3.0)
            self.canvas.create_text(34, y, text=f"{angle:.0f}", fill="#94a3b8", anchor="e", font=("Arial", 8))
        for index in range(11):
            x = 45 + index * (self.width - 60) / 10.0
            self.canvas.create_line(x, 25, x, self.height - 25, fill="#1e293b")
    def _draw_series(self, snapshots: Sequence[ControlSnapshot], name: str, selector: Callable[[ControlSnapshot], float], dashed: bool = False) -> None:
        points: List[float] = []
        usable_width = self.width - 60
        usable_height = self.height - 50
        for index, snapshot in enumerate(snapshots):
            x = 45 + usable_width * index / max(1, len(snapshots) - 1)
            normalized = clamp(selector(snapshot) / self.maximum_abs_angle, -1.0, 1.0)
            y = 25 + usable_height * (1.0 - (normalized + 1.0) / 2.0)
            points.extend((x, y))
        options: Dict[str, Any] = {"fill": self.COLORS[name], "width": 2}
        if dashed:
            options["dash"] = (5, 3)
        self.canvas.create_line(*points, **options)
    def _draw_legend(self) -> None:
        labels = (("roll", "横滚"), ("pitch", "俯仰"), ("yaw", "偏航"), ("target", "目标俯仰"))
        for index, (key, label) in enumerate(labels):
            x = 70 + index * 120
            self.canvas.create_line(x, 12, x + 22, 12, fill=self.COLORS[key], width=2)
            self.canvas.create_text(x + 28, 12, text=label, fill="#e2e8f0", anchor="w")
class RobotGui:
    """完整桌面控制台，所有Tkinter调用留在GUI线程。"""
    def __init__(self, engine: ControlEngine) -> None:
        try:
            import tkinter as tk
            from tkinter import messagebox
            from tkinter import ttk
        except ImportError as error:
            raise RuntimeError("当前Python未安装Tkinter，请改用--interface cli") from error
        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.engine = engine
        self.root = tk.Tk()
        self.root.title(f"{APP_NAME} v{APP_VERSION}")
        self.root.geometry("1220x820")
        self.root.minsize(980, 700)
        self.root.configure(background="#e2e8f0")
        self.events = EventQueueAdapter(engine.bus)
        self.closed = False
        self.status_text = tk.StringVar(value="系统启动中")
        self.mode_text = tk.StringVar(value=f"模式: {engine.config.mode.value}")
        self.command_text = tk.StringVar(value="命令: 停止")
        self.arm_text = tk.StringVar(value="输出: 锁定")
        self.fault_text = tk.StringVar(value="故障: 无")
        self.counter_text = tk.StringVar(value="周期: 0 | 帧: 0 | 频率: 0.0Hz")
        self.target_text = tk.StringVar(value="目标 R +0.0° P +0.0° Y +0.0°")
        self.roll_variable = tk.DoubleVar(value=0.0)
        self.pitch_variable = tk.DoubleVar(value=0.0)
        self.yaw_variable = tk.DoubleVar(value=0.0)
        self.walk_frequency_variable = tk.DoubleVar(value=engine.config.control.walk_frequency_hz)
        self.walk_amplitude_variable = tk.DoubleVar(value=engine.config.control.walk_amplitude_degrees)
        self._configure_styles()
        self._build_layout()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
    def _configure_styles(self) -> None:
        style = self.ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TFrame", background="#e2e8f0")
        style.configure("Card.TFrame", background="#f8fafc", relief="flat")
        style.configure("TLabel", background="#e2e8f0", foreground="#0f172a", font=("Microsoft YaHei", 10))
        style.configure("Card.TLabel", background="#f8fafc", foreground="#0f172a", font=("Microsoft YaHei", 10))
        style.configure("Title.TLabel", font=("Microsoft YaHei", 18, "bold"), foreground="#0f172a")
        style.configure("Status.TLabel", background="#0f172a", foreground="#e2e8f0", font=("Microsoft YaHei", 10))
        style.configure("TButton", font=("Microsoft YaHei", 10), padding=6)
        style.configure("Danger.TButton", foreground="#ffffff", background="#dc2626")
        style.map("Danger.TButton", background=[("active", "#b91c1c")])
        style.configure("Safe.TButton", foreground="#ffffff", background="#15803d")
        style.map("Safe.TButton", background=[("active", "#166534")])
    def _build_layout(self) -> None:
        header = self.ttk.Frame(self.root, padding=12)
        header.pack(fill="x")
        self.ttk.Label(header, text=APP_NAME, style="Title.TLabel").pack(side="left")
        self.ttk.Label(header, textvariable=self.mode_text).pack(side="right")
        status_bar = self.tk.Frame(self.root, background="#0f172a", padx=10, pady=8)
        status_bar.pack(fill="x", padx=12)
        for variable in (self.status_text, self.command_text, self.arm_text, self.fault_text, self.counter_text):
            label = self.tk.Label(status_bar, textvariable=variable, background="#0f172a", foreground="#e2e8f0", font=("Microsoft YaHei", 9))
            label.pack(side="left", padx=10)
        notebook = self.ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=12, pady=10)
        control_tab = self.ttk.Frame(notebook, padding=8)
        plot_tab = self.ttk.Frame(notebook, padding=8)
        diagnostic_tab = self.ttk.Frame(notebook, padding=8)
        notebook.add(control_tab, text="机器人控制")
        notebook.add(plot_tab, text="姿态曲线")
        notebook.add(diagnostic_tab, text="系统信息")
        self._build_control_tab(control_tab)
        self._build_plot_tab(plot_tab)
        self._build_diagnostic_tab(diagnostic_tab)
        log_frame = self.ttk.Frame(self.root, padding=4)
        log_frame.pack(fill="both", expand=False, padx=12, pady=(0, 10))
        self.log_text = self.tk.Text(log_frame, height=8, background="#0f172a", foreground="#d1d5db", insertbackground="#ffffff", font=("Consolas", 9), state="disabled")
        scrollbar = self.ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    def _build_control_tab(self, parent: Any) -> None:
        left = self.ttk.Frame(parent, style="Card.TFrame", padding=10)
        left.pack(side="left", fill="both", expand=True)
        right = self.ttk.Frame(parent, style="Card.TFrame", padding=12)
        right.pack(side="right", fill="y", padx=(10, 0))
        self.ttk.Label(left, text="实时姿态仪", style="Card.TLabel", font=("Microsoft YaHei", 12, "bold")).pack()
        self.attitude_canvas = AttitudeCanvas(self.tk, left)
        self.attitude_canvas.pack(pady=8)
        self.ttk.Label(left, textvariable=self.target_text, style="Card.TLabel").pack(pady=4)
        self.ttk.Label(right, text="运行控制", style="Card.TLabel", font=("Microsoft YaHei", 12, "bold")).grid(row=0, column=0, columnspan=3, pady=(0, 8))
        self._button(right, "解锁输出", self.arm, "Safe.TButton").grid(row=1, column=0, padx=4, pady=4, sticky="ew")
        self._button(right, "锁定输出", self.disarm).grid(row=1, column=1, padx=4, pady=4, sticky="ew")
        self._button(right, "故障复位", self.reset_fault).grid(row=1, column=2, padx=4, pady=4, sticky="ew")
        self._button(right, "平衡", lambda: self.request_command(RobotCommand.BALANCE)).grid(row=2, column=0, padx=4, pady=4, sticky="ew")
        self._button(right, "走路", lambda: self.request_command(RobotCommand.WALK)).grid(row=2, column=1, padx=4, pady=4, sticky="ew")
        self._button(right, "停止", lambda: self.request_command(RobotCommand.STOP)).grid(row=2, column=2, padx=4, pady=4, sticky="ew")
        emergency = self._button(right, "急 停\nSpace", self.emergency_stop, "Danger.TButton")
        emergency.grid(row=3, column=0, columnspan=3, padx=4, pady=8, sticky="ew", ipady=8)
        self._build_target_controls(right)
        self._build_walk_controls(right)
        self._build_recording_controls(right)
        self._build_simulation_controls(right)
        for column in range(3):
            right.columnconfigure(column, weight=1)
    def _build_target_controls(self, parent: Any) -> None:
        frame = self.ttk.LabelFrame(parent, text="目标姿态（度）", padding=8)
        frame.grid(row=4, column=0, columnspan=3, sticky="ew", pady=5)
        specifications = (("横滚", "roll", self.roll_variable, -20.0, 20.0), ("俯仰", "pitch", self.pitch_variable, -20.0, 20.0), ("偏航", "yaw", self.yaw_variable, -180.0, 180.0))
        for row, (label, axis, variable, minimum, maximum) in enumerate(specifications):
            self.ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w")
            scale = self.ttk.Scale(frame, from_=minimum, to=maximum, variable=variable, command=lambda value, name=axis: self.on_target_scale(name, float(value)))
            scale.grid(row=row, column=1, sticky="ew", padx=6)
            value_label = self.ttk.Label(frame, textvariable=variable, width=7)
            value_label.grid(row=row, column=2)
        frame.columnconfigure(1, weight=1)
        self.ttk.Button(frame, text="目标归零", command=self.zero_target).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 0))
    def _build_walk_controls(self, parent: Any) -> None:
        frame = self.ttk.LabelFrame(parent, text="走路参数", padding=8)
        frame.grid(row=5, column=0, columnspan=3, sticky="ew", pady=5)
        self.ttk.Label(frame, text="频率Hz").grid(row=0, column=0)
        self.ttk.Spinbox(frame, from_=0.05, to=1.0, increment=0.05, textvariable=self.walk_frequency_variable, width=8).grid(row=0, column=1)
        self.ttk.Label(frame, text="摆幅°").grid(row=1, column=0)
        self.ttk.Spinbox(frame, from_=0.0, to=8.0, increment=0.5, textvariable=self.walk_amplitude_variable, width=8).grid(row=1, column=1)
        self.ttk.Button(frame, text="应用步态参数", command=self.apply_walk_parameters).grid(row=0, column=2, rowspan=2, padx=6, sticky="ns")
    def _build_recording_controls(self, parent: Any) -> None:
        frame = self.ttk.LabelFrame(parent, text="数据记录与校准", padding=8)
        frame.grid(row=6, column=0, columnspan=3, sticky="ew", pady=5)
        self.ttk.Button(frame, text="开始CSV", command=self.start_recording).grid(row=0, column=0, sticky="ew", padx=2)
        self.ttk.Button(frame, text="停止CSV", command=self.engine.stop_recording).grid(row=0, column=1, sticky="ew", padx=2)
        self.ttk.Button(frame, text="陀螺仪校准", command=self.calibrate).grid(row=0, column=2, sticky="ew", padx=2)
    def _build_simulation_controls(self, parent: Any) -> None:
        if self.engine.config.mode != RunMode.SIMULATION:
            return
        frame = self.ttk.LabelFrame(parent, text="仿真故障注入", padding=8)
        frame.grid(row=7, column=0, columnspan=3, sticky="ew", pady=5)
        self.ttk.Button(frame, text="模拟35°摔倒", command=self.simulate_fall).grid(row=0, column=0, columnspan=2, sticky="ew", padx=2)
        self.ttk.Button(frame, text="清除仿真扰动", command=self.clear_simulation).grid(row=0, column=2, sticky="ew", padx=2)
    def _build_plot_tab(self, parent: Any) -> None:
        self.ttk.Label(parent, text="姿态与目标历史（最近200个控制快照）").pack(anchor="w")
        self.plot_canvas = HistoryPlotCanvas(self.tk, parent)
        self.plot_canvas.pack(fill="both", expand=True, pady=8)
        self.ttk.Label(parent, text="红=Roll，绿=Pitch，蓝=Yaw，黄色虚线=目标Pitch").pack(anchor="w")
    def _build_diagnostic_tab(self, parent: Any) -> None:
        top = self.ttk.Frame(parent)
        top.pack(fill="x")
        self.ttk.Button(top, text="刷新诊断", command=self.refresh_diagnostics).pack(side="left")
        self.ttk.Button(top, text="复制状态JSON", command=self.copy_status).pack(side="left", padx=6)
        self.diagnostic_text = self.tk.Text(parent, background="#f8fafc", foreground="#0f172a", font=("Consolas", 10), wrap="word")
        self.diagnostic_text.pack(fill="both", expand=True, pady=8)
        boundary = "控制边界：本软件输出V2姿态目标，由ESP32执行Pitch PID；不直接生成Go1电机帧。\n真实电机底层协议、定标和反馈必须另行实测，本软件不会假装获得缺失的反馈。\nYaw仅由陀螺积分得到，会漂移。"
        self.diagnostic_text.insert("1.0", boundary)
        self.diagnostic_text.configure(state="disabled")
    @staticmethod
    def _button(parent: Any, text: str, command: Callable[[], None], style: str = "TButton") -> Any:
        # ttk实例通过parent的tk路径创建，样式在调用侧保持统一。
        import tkinter.ttk as ttk
        return ttk.Button(parent, text=text, command=command, style=style)
    def _bind_keys(self) -> None:
        self.root.bind("<space>", lambda event: self.emergency_stop())
        self.root.bind("<Escape>", lambda event: self.request_command(RobotCommand.STOP))
        self.root.bind("<Control-b>", lambda event: self.request_command(RobotCommand.BALANCE))
        self.root.bind("<Control-w>", lambda event: self.request_command(RobotCommand.WALK))
        self.root.bind("<Control-z>", lambda event: self.zero_target())
    def start(self) -> None:
        try:
            self.engine.start()
        except Exception as error:
            self.messagebox.showerror("启动失败", str(error), parent=self.root)
            self.close(send_emergency=False)
            return
        self._append_log(f"{APP_NAME}已启动，默认STOP")
        self.root.after(100, self.refresh)
        self.root.mainloop()
    def refresh(self) -> None:
        if self.closed:
            return
        for event in self.events.drain():
            self._append_log(event.format())
        snapshot = self.engine.last_snapshot
        if snapshot is not None:
            self._refresh_status(snapshot)
            self.attitude_canvas.update_attitude(snapshot.attitude)
            self.plot_canvas.update_history(self.engine.history.recent(200))
        self.root.after(100, self.refresh)
    def _refresh_status(self, snapshot: ControlSnapshot) -> None:
        frequency = safe_divide(1.0, snapshot.control_period_seconds)
        self.status_text.set(f"状态: {'运行' if self.engine.running else '停止'}")
        self.command_text.set(f"命令: {command_name(snapshot.command)}")
        self.arm_text.set(f"输出: {'解锁' if snapshot.armed else '锁定'}")
        self.fault_text.set(f"故障: {fault_message(snapshot.fault)}")
        self.counter_text.set(f"周期: {snapshot.sequence} | 帧: {snapshot.frames_sent} | {frequency:.1f}Hz")
        target = snapshot.filtered_target
        self.target_text.set(f"目标 R {target.roll:+.2f}° P {target.pitch:+.2f}° Y {target.yaw:+.2f}°")
    def _append_log(self, message: str) -> None:
        if self.closed:
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 500:
            self.log_text.delete("1.0", "100.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")
    def arm(self) -> None:
        if self.engine.config.mode == RunMode.HARDWARE:
            message = "确认机器人已支撑稳定、周边无人员障碍物，并确认ESP32 V2固件已烧录。是否解锁？"
            if not self.messagebox.askyesno("真实硬件解锁确认", message, parent=self.root):
                return
        result = self.engine.arm()
        self._append_log(result[1])
    def disarm(self) -> None:
        self.engine.disarm()
    def request_command(self, command: RobotCommand) -> None:
        result = self.engine.request_command(command)
        self._append_log(result[1])
    def emergency_stop(self) -> None:
        self.engine.emergency_stop("图形界面急停")
    def reset_fault(self) -> None:
        if not self.messagebox.askyesno("故障复位", "确认机器人已恢复安全姿态且故障原因已排除？", parent=self.root):
            return
        result = self.engine.clear_fault(confirmed=True)
        self._append_log(result[1])
    def on_target_scale(self, axis: str, value: float) -> None:
        try:
            self.engine.set_target_axis(axis, value)
        except Exception as error:
            self._append_log(f"目标设置失败: {error}")
    def zero_target(self) -> None:
        self.roll_variable.set(0.0)
        self.pitch_variable.set(0.0)
        self.yaw_variable.set(0.0)
        self.engine.zero_target()
    def apply_walk_parameters(self) -> None:
        try:
            self.engine.set_walk_parameters(self.walk_frequency_variable.get(), self.walk_amplitude_variable.get())
        except Exception as error:
            self.messagebox.showerror("步态参数无效", str(error), parent=self.root)
    def start_recording(self) -> None:
        try:
            path = self.engine.start_recording()
            self._append_log(f"CSV记录: {path}")
        except Exception as error:
            self.messagebox.showerror("记录启动失败", str(error), parent=self.root)
    def calibrate(self) -> None:
        if not self.messagebox.askyesno("校准", "请保持机器人完全静止。是否开始校准？", parent=self.root):
            return
        try:
            calibration = self.engine.calibrate_sensor()
            self._append_log(f"校准完成: {calibration.samples}个样本")
        except Exception as error:
            self.messagebox.showerror("校准失败", str(error), parent=self.root)
    def simulate_fall(self) -> None:
        if isinstance(self.engine.sensor, SimulatedSensor):
            self.engine.sensor.force_attitude(Attitude(35.0, 0.0, 0.0))
    def clear_simulation(self) -> None:
        if isinstance(self.engine.sensor, SimulatedSensor):
            self.engine.sensor.force_attitude(None)
            self.engine.sensor.model.reset()
            self.engine.sensor.disturb()
    def refresh_diagnostics(self) -> None:
        report = SystemDiagnostics(self.engine).run()
        self.diagnostic_text.configure(state="normal")
        self.diagnostic_text.delete("1.0", "end")
        self.diagnostic_text.insert("1.0", report.format())
        self.diagnostic_text.insert("end", "\n\n" + json.dumps(self.engine.status_dictionary(), ensure_ascii=False, indent=2))
        self.diagnostic_text.configure(state="disabled")
    def copy_status(self) -> None:
        content = json.dumps(self.engine.status_dictionary(), ensure_ascii=False, indent=2)
        self.root.clipboard_clear()
        self.root.clipboard_append(content)
        self._append_log("状态JSON已复制到剪贴板")
    def close(self, send_emergency: bool = True) -> None:
        if self.closed:
            return
        if send_emergency:
            self.engine.emergency_stop("图形界面关闭")
        self.closed = True
        self.events.close()
        self.engine.stop()
        self.root.destroy()
def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=f"{APP_NAME}：默认仿真，真实硬件需显式--hardware")
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--hardware", action="store_true", help="使用MPU6050与真实串口，默认仍是仿真")
    mode_group.add_argument("--simulation", action="store_true", help="强制使用内置仿真")
    parser.add_argument("--interface", choices=("gui", "cli", "headless"), default="gui", help="交互界面，默认gui")
    parser.add_argument("--duration", type=float, default=0.0, help="headless运行秒数，0表示持续运行")
    parser.add_argument("--port", default=None, help="串口，例如/dev/ttyACM0或COM3")
    parser.add_argument("--baudrate", type=int, default=None, help="串口波特率，默认115200")
    parser.add_argument("--i2c-bus", type=int, default=None, help="MPU6050 I2C总线，默认6")
    parser.add_argument("--i2c-address", type=lambda value: int(value, 0), default=None, help="I2C地址，默认0x69")
    parser.add_argument("--hz", type=float, default=None, help="控制循环频率，默认70Hz")
    parser.add_argument("--config", type=Path, default=None, help="读取JSON配置文件")
    parser.add_argument("--save-config", type=Path, default=None, help="保存当前配置后退出")
    parser.add_argument("--self-test", action="store_true", help="运行内置仿真回归测试后退出")
    parser.add_argument("--calibrate", action="store_true", help="真实硬件启动时校准陀螺仪")
    parser.add_argument("--record", type=Path, default=None, help="启动后立即记录CSV到指定文件")
    parser.add_argument("--json", action="store_true", help="headless结束时输出JSON状态")
    parser.add_argument("--version", action="version", version=f"%(prog)s {APP_VERSION}")
    return parser
def configuration_from_arguments(arguments: argparse.Namespace) -> AppConfig:
    if arguments.config is not None:
        config = AppConfig.from_dict(load_json_file(arguments.config))
    else:
        config = AppConfig()
    mode = RunMode.HARDWARE if arguments.hardware else RunMode.SIMULATION
    if not arguments.hardware and not arguments.simulation and arguments.config is not None:
        mode = config.mode
    serial = dataclasses.replace(
        config.serial,
        port=arguments.port if arguments.port is not None else config.serial.port,
        baudrate=arguments.baudrate if arguments.baudrate is not None else config.serial.baudrate,
    )
    sensor = dataclasses.replace(
        config.sensor,
        i2c_bus=arguments.i2c_bus if arguments.i2c_bus is not None else config.sensor.i2c_bus,
        i2c_address=arguments.i2c_address if arguments.i2c_address is not None else config.sensor.i2c_address,
    )
    control = dataclasses.replace(
        config.control,
        frequency_hz=arguments.hz if arguments.hz is not None else config.control.frequency_hz,
    )
    result = dataclasses.replace(config, mode=mode, serial=serial, sensor=sensor, control=control)
    result.validate()
    return result
def launch_gui(engine: ControlEngine) -> int:
    interface = RobotGui(engine)
    interface.start()
    return 0
def launch_cli(engine: ControlEngine, calibrate: bool = False) -> int:
    engine.start(calibrate_hardware=calibrate)
    shell = RobotShell(engine)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        engine.emergency_stop("终端Ctrl+C")
    finally:
        engine.stop()
    return 0
def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    arguments = parser.parse_args(argv)
    if arguments.self_test:
        runner = SelfTestRunner()
        runner.run_all()
        print(runner.format_results())
        return 0 if all(result.passed for result in runner.results) else 1
    try:
        config = configuration_from_arguments(arguments)
    except Exception as error:
        parser.error(str(error))
        return 2
    if arguments.save_config is not None:
        save_json_file(arguments.save_config, config.to_dict())
        print(f"配置已保存: {arguments.save_config}")
        return 0
    if arguments.duration < 0.0:
        parser.error("--duration不能为负数")
    engine = ControlEngine(config)
    try:
        if arguments.record is not None:
            engine.start_recording(arguments.record)
        if arguments.interface == "gui":
            return launch_gui(engine)
        if arguments.interface == "cli":
            return launch_cli(engine, arguments.calibrate)
        return run_headless(engine, arguments.duration, arguments.json)
    except KeyboardInterrupt:
        engine.emergency_stop("主程序Ctrl+C")
        return 0
    except Exception as error:
        print(f"程序错误: {error}", file=sys.stderr)
        return 2
    finally:
        engine.stop()
if __name__ == "__main__":
    raise SystemExit(main())
