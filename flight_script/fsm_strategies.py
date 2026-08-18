# fsm_strategies.py
import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from flight_core import FlightContext, AttitudeControlEvent

@dataclass
class StrategyResult:
    control_event: AttitudeControlEvent
    is_completed: bool  # 해당 단계 완료 여부 (True 시 다음 FSM 상태로 전이)

# ==========================================
# 1. RE_ENTRY_CIRCUIT (복행 선회 전략 규격)
# ==========================================
class ReEntryStrategy(ABC):
    @abstractmethod
    def reset(self):
        """전략 상태 초기화"""
        pass

    @abstractmethod
    def execute(self, context: FlightContext, target_gps: tuple[float, float] | None) -> StrategyResult:
        """복행 선회 중 조종 신호 및 진입 완료 여부 반환"""
        pass

class TeardropReEntryStrategy(ReEntryStrategy):
    """티어드롭 회항: 외측 30도 이탈 비행 후 반대 방향 180도 반전 선회"""
    def __init__(self, outbound_time_sec: float = 8.0, bank_angle: float = 20.0):
        self.outbound_time_sec = outbound_time_sec
        self.bank_angle = bank_angle
        self.start_time = 0.0

    def reset(self):
        self.start_time = time.time()

    def execute(self, context: FlightContext, target_gps: tuple[float, float] | None) -> StrategyResult:
        elapsed = time.time() - self.start_time
        
        # 1구간: 타겟 외측으로 완만한 뱅크 유지하며 거리 확보
        if elapsed < self.outbound_time_sec:
            cmd = AttitudeControlEvent(roll_deg=10.0, pitch_deg=2.0, throttle=0.55)
            return StrategyResult(control_event=cmd, is_completed=False)
        
        # 2구간: 180도 반전 선회 (티어드롭 턴)
        elif elapsed < (self.outbound_time_sec + 10.0):
            cmd = AttitudeControlEvent(roll_deg=-self.bank_angle, pitch_deg=3.0, throttle=0.6)
            return StrategyResult(control_event=cmd, is_completed=False)
        
        # 선회 완료 후 재진입 단계로 전환 신호 발생
        else:
            cmd = AttitudeControlEvent(roll_deg=0.0, pitch_deg=0.0, throttle=0.5)
            return StrategyResult(control_event=cmd, is_completed=True)


class OrbitReEntryStrategy(ReEntryStrategy):
    """오빗 선회: 기억된 타겟 좌표 주위를 지정 반경으로 선회하다가 진입 축선 정렬 시 완료"""
    def __init__(self, orbit_radius_m: float = 50.0):
        self.orbit_radius_m = orbit_radius_m
        self.roll_cmd = 22.0

    def reset(self):
        pass

    def execute(self, context: FlightContext, target_gps: tuple[float, float] | None) -> StrategyResult:
        # 타겟 기준 정렬 조건(예: 특정 진입 헤딩각 ±10도) 만족 시 완료 처리
        # 여기서는 기본 정속 선회 제어 신호 유지
        cmd = AttitudeControlEvent(roll_deg=self.roll_cmd, pitch_deg=0.0, throttle=0.55)
        
        # 가령 텔레메트리 방위각이 진입 각도에 도달했는지 검사
        aligned_with_approach_axis = False # 실제 방위각 비교 로직 적용 영역
        
        return StrategyResult(control_event=cmd, is_completed=aligned_with_approach_axis)


# ==========================================
# 2. RECOVERY_SEARCH (재탐색/접근 전략 규격)
# ==========================================
class RecoveryStrategy(ABC):
    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def execute(self, context: FlightContext, target_gps: tuple[float, float] | None) -> StrategyResult:
        """기억된 좌표를 향해 접근하며 비전 재포착 대기"""
        pass

class DirectGPSRecoveryStrategy(RecoveryStrategy):
    """기억된 타겟 절대 좌표(Lat, Lon)를 향해 직선 L1/Heading으로 유도"""
    def __init__(self, target_alt_m: float = 30.0):
        self.target_alt_m = target_alt_m

    def reset(self):
        pass

    def execute(self, context: FlightContext, target_gps: tuple[float, float] | None) -> StrategyResult:
        if target_gps is None:
            cmd = AttitudeControlEvent(roll_deg=0.0, pitch_deg=0.0, throttle=0.5)
            return StrategyResult(control_event=cmd, is_completed=False)

        target_lat, target_lon = target_gps
        # GPS 기반 목표 방위각 계산 (Bearing)
        d_lat = math.radians(target_lat - context.telemetry.lat)
        d_lon = math.radians(target_lon - context.telemetry.lon)
        y = math.sin(d_lon) * math.cos(math.radians(target_lat))
        x = math.cos(math.radians(context.telemetry.lat)) * math.sin(math.radians(target_lat)) - \
            math.sin(math.radians(context.telemetry.lat)) * math.cos(math.radians(target_lat)) * math.cos(d_lon)
        target_bearing_deg = (math.degrees(math.atan2(y, x)) + 360) % 360

        # 기체 현재 Heading과 목표 Bearing 사이의 오차
        heading_error = (target_bearing_deg - context.telemetry.yaw_deg + 180) % 360 - 180
        roll_cmd = max(-25.0, min(25.0, heading_error * 0.8))

        alt_error = self.target_alt_m - context.telemetry.alt_m
        pitch_cmd = max(-10.0, min(10.0, alt_error * 1.2))

        cmd = AttitudeControlEvent(roll_deg=roll_cmd, pitch_deg=pitch_cmd, throttle=0.5)
        
        # 타겟이 비전(YOLO)에 다시 감지되면 Recovery 성공 완료
        is_reacquired = (context.target is not None)
        return StrategyResult(control_event=cmd, is_completed=is_reacquired)