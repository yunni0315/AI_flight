# flight_core_library.py
import time
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Type
from pymavlink import mavutil

@dataclass(frozen=True)
class TelemetryState:
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    ground_speed_mps: float = 0.0

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

class MavlinkActuator:
    def __init__(self, master):
        self.master = master

    def handle_attitude_control(self, event: AttitudeControlEvent):
        roll_rad = math.radians(event.roll_deg)
        pitch_rad = math.radians(event.pitch_deg)
        yaw_rate_rad = math.radians(event.yaw_rate_deg_s)
        
        # MAVLink SET_ATTITUDE_TARGET (0b10000011: roll/pitch/yaw_rate 활성화)
        self.master.mav.set_attitude_target_send(
            int(time.time() * 1000),
            self.master.target_system,
            self.master.target_component,
            0b10000011,
            [1, 0, 0, 0],
            roll_rad,
            pitch_rad,
            yaw_rate_rad,
            event.throttle
        )

    def handle_target_aligned(self, event: TargetAlignedEvent):
        print(f"[Target Locked] 좌표 픽셀: ({event.pixel_x}, {event.pixel_y}), 고도: {event.alt_m:.1f}m")