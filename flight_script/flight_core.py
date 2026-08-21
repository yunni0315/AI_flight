# flight_core_library.py
import time
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Type
from pymavlink import mavutil

# 비행 안전 기본 상수
MIN_SAFE_FLOOR_ALT_M = 15.0  # 하한 안전 고도 (이하 하강 절대 금지)
DEFAULT_TARGET_ALT_M = 30.0  # 기본 정찰 순항 고도
DEFAULT_ABORT_ALT_M = 45.0   # 복행 안전 고도

@dataclass(frozen=True)
class TelemetryState:
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    ground_speed_mps: float = 0.0

    def is_valid(self) -> bool:
        """텔레메트리 유효성 검사 (GPS 수신 및 비정상 NaN/Inf 필터링)"""
        if math.isnan(self.lat) or math.isnan(self.lon) or math.isnan(self.alt_m):
            return False
        if math.isinf(self.lat) or math.isinf(self.lon) or math.isinf(self.alt_m):
            return False
        # 유효한 GPS 좌표 범위 체크 (초기화 0,0 제외)
        if abs(self.lat) > 90.0 or abs(self.lon) > 180.0:
            return False
        return True

@dataclass(frozen=True)
class TargetDetection:
    class_name: str
    confidence: float
    pixel_x: int            # 화면 내 타겟 중심 x 좌표
    pixel_y: int            # 화면 내 타겟 중심 y 좌표
    pixel_width: int
    slant_range_m: float
    ground_dist_m: float

@dataclass(frozen=True)
class FlightContext:
    telemetry: TelemetryState
    target: TargetDetection | None
    is_guided_mode: bool = True  # FC가 GUIDED 모드인지 여부

@dataclass(frozen=True)
class AttitudeControlEvent:
    roll_deg: float
    pitch_deg: float = 0.0
    yaw_rate_deg_s: float = 0.0
    throttle: float = 0.5

@dataclass(frozen=True)
class TargetAlignedEvent:
    pixel_x: int
    pixel_y: int
    alt_m: float
    timestamp: float

class EventBus:
    def __init__(self):
        self._subscribers: Dict[Type, List[Callable]] = {}

    def subscribe(self, event_type: Type, handler: Callable):
        if event_type not in self._subscribers:
            self._subscribers[event_type] = []
        self._subscribers[event_type].append(handler)

    def publish(self, event):
        event_type = type(event)
        if event_type in self._subscribers:
            for handler in self._subscribers[event_type]:
                handler(event)

def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float = 0.0) -> list[float]:
    """오일러 각(Roll, Pitch, Yaw in radians)을 쿼터니언 [w, x, y, z]으로 변환"""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]

class MavlinkActuator:
    def __init__(self, master):
        self.master = master
        self._start_time = time.time()

    def handle_attitude_control(self, event: AttitudeControlEvent):
        if self.master is None:
            return

        roll_rad = math.radians(event.roll_deg)
        pitch_rad = math.radians(event.pitch_deg)
        yaw_rate_rad = math.radians(event.yaw_rate_deg_s)
        
        # MAVLink time_boot_ms는 32비트 부호없는 정수(0~4294967295)이므로 오버플로우 방지
        time_boot_ms = int((time.time() - self._start_time) * 1000) & 0xFFFFFFFF
        
        # 오일러 각을 쿼터니언으로 변환
        q = euler_to_quaternion(roll_rad, pitch_rad, 0.0)

        # MAVLink SET_ATTITUDE_TARGET
        # type_mask = 0b00000111 (7): Roll/Pitch/Yaw Rate 무시하고 Quaternion 자세 및 Throttle 적용
        self.master.mav.set_attitude_target_send(
            time_boot_ms,
            self.master.target_system,
            self.master.target_component,
            0b00000111,
            q,
            0.0,
            0.0,
            yaw_rate_rad,
            event.throttle
        )

    def handle_target_aligned(self, event: TargetAlignedEvent):
        print(f"[Target Locked] 좌표 픽셀: ({event.pixel_x}, {event.pixel_y}), 고도: {event.alt_m:.1f}m")