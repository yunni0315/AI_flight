# fsm_coordinator.py
import math
import time
from enum import Enum, auto
from flight_core import EventBus, FlightContext, AttitudeControlEvent, TargetAlignedEvent
from fsm_strategies import ReEntryStrategy, RecoveryStrategy

class FlightState(Enum):
    SEARCH = auto()           # 기본 탐색 비행
    APPROACH = auto()         # 타겟 통과 시도 (PPN 등)
    ABORT_CLIMB = auto()      # 복행 안전 상승
    RE_ENTRY_CIRCUIT = auto() # 장주/선회 궤도 (전략 모듈 위임)
    RECOVERY_SEARCH = auto()  # 기억 좌표 기반 재진입 (전략 모듈 위임)

class FlyoverFSMCoordinator:
    """
    상태 전이 및 예외 상황 관리, 전략 모듈(Circuit/Recovery)을 통합 조율하는 FSM 트리거
    """
    def __init__(
        self, 
        bus: EventBus, 
        re_entry_strategy: ReEntryStrategy,
        recovery_strategy: RecoveryStrategy,
        target_alt_m: float = 30.0,
        abort_alt_m: float = 45.0
    ):
        self.bus = bus
        self.re_entry_strategy = re_entry_strategy
        self.recovery_strategy = recovery_strategy
        self.target_alt_m = target_alt_m
        self.abort_alt_m = abort_alt_m

        self.state = FlightState.SEARCH
        self.remembered_target_gps: tuple[float, float] | None = None
        self.last_target_seen_time = 0.0
        self.state_enter_time = time.time()

    def _transition_to(self, new_state: FlightState):
        print(f"[FSM Transition] {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_enter_time = time.time()

        # 진입 시 해당 전략 모듈 초기화
        if new_state == FlightState.RE_ENTRY_CIRCUIT:
            self.re_entry_strategy.reset()
        elif new_state == FlightState.RECOVERY_SEARCH:
            self.recovery_strategy.reset()

    def _update_target_memory(self, context: FlightContext):
        """비전 정보와 FC 텔레메트리로 타겟의 지상 GPS 좌표 추정 및 갱신"""
        if context.target is not None:
            self.last_target_seen_time = time.time()
            R_EARTH = 6378137.0
            telem = context.telemetry
            heading_rad = math.radians(telem.yaw_deg) + math.atan2(context.target.pixel_x - 320, 600)
            dist = context.target.ground_dist_m

            d_north = dist * math.cos(heading_rad)
            d_east = dist * math.sin(heading_rad)

            lat = telem.lat + (d_north / R_EARTH) * (180.0 / math.pi)
            lon = telem.lon + (d_east / (R_EARTH * math.cos(math.radians(telem.lat)))) * (180.0 / math.pi)
            self.remembered_target_gps = (lat, lon)

    def _is_unreachable(self, context: FlightContext) -> bool:
        """물리적으로 안정 통과가 불가능한 기하 조건 판단 (예외 감지)"""
        if context.target is None:
            return False
        # 거리가 너무 가까운데(15m 미만) 화면 중심 오프셋이 너무 큼(100px 초과)
        if context.target.ground_dist_m < 15.0 and abs(context.target.pixel_x - 320) > 100:
            return True
        return False

    def evaluate(self, context: FlightContext):
        self._update_target_memory(context)

        # -------------------------------------------------------------
        # 1. SEARCH: 초기 탐색 상태
        # -------------------------------------------------------------
        if self.state == FlightState.SEARCH:
            if context.target is not None:
                # 도달 가능 영역 검사 후 통과 시도 또는 즉시 복행 결정
                if self._is_unreachable(context):
                    self._transition_to(FlightState.ABORT_CLIMB)
                else:
                    self._transition_to(FlightState.APPROACH)
            else:
                self.bus.publish(AttitudeControlEvent(roll_deg=0.0, pitch_deg=0.0, throttle=0.5))

        # -------------------------------------------------------------
        # 2. APPROACH: 최종 통과 시도 (PPN 또는 비전 정렬)
        # -------------------------------------------------------------
        elif self.state == FlightState.APPROACH:
            # 예외 1: 시야 상실 (LOS Lost)
            if time.time() - self.last_target_seen_time > 0.6:
                print("[예외 발생] 접근 중 타겟 시야 상실 -> 복행")
                self._transition_to(FlightState.ABORT_CLIMB)
                return

            # 예외 2: 접근 도중 도달 불능 궤적으로 이탈
            if self._is_unreachable(context):
                print("[예외 발생] 통과 오차 한계 초과 -> 복행")
                self._transition_to(FlightState.ABORT_CLIMB)
                return

            # 정상 통과 판정 (5m 이내 진입)
            if context.target and context.target.ground_dist_m <= 5.0:
                self.bus.publish(TargetAlignedEvent(
                    pixel_x=context.target.pixel_x,
                    pixel_y=context.target.pixel_y,
                    alt_m=context.telemetry.alt_m,
                    timestamp=time.time()
                ))

            # 기본 롤/피치 접근 신호 (간이 PPN)
            error_x = (context.target.pixel_x - 320) if context.target else 0
            roll_cmd = max(-25.0, min(25.0, error_x * 0.08))
            self.bus.publish(AttitudeControlEvent(roll_deg=roll_cmd, pitch_deg=0.0, throttle=0.55))

        # -------------------------------------------------------------
        # 3. ABORT_CLIMB: 안전 고도 상승
        # -------------------------------------------------------------
        elif self.state == FlightState.ABORT_CLIMB:
            # 날개를 펴고(Wings level) 복행 안전 고도까지 상승
            alt_error = self.abort_alt_m - context.telemetry.alt_m
            pitch_cmd = max(5.0, min(15.0, alt_error * 1.5))

            self.bus.publish(AttitudeControlEvent(roll_deg=0.0, pitch_deg=pitch_cmd, throttle=0.75))

            # 안전 고도 도달 시 장주 선회 전략 단계로 전이
            if context.telemetry.alt_m >= (self.abort_alt_m - 2.0):
                self._transition_to(FlightState.RE_ENTRY_CIRCUIT)

        # -------------------------------------------------------------
        # 4. RE_ENTRY_CIRCUIT: 선회 및 진입로 정렬 (전략 위임)
        # -------------------------------------------------------------
        elif self.state == FlightState.RE_ENTRY_CIRCUIT:
            result = self.re_entry_strategy.execute(context, self.remembered_target_gps)
            self.bus.publish(result.control_event)

            if result.is_completed:
                self._transition_to(FlightState.RECOVERY_SEARCH)

        # -------------------------------------------------------------
        # 5. RECOVERY_SEARCH: 기억된 좌표 기반 재탐색/접근 (전략 위임)
        # -------------------------------------------------------------
        elif self.state == FlightState.RECOVERY_SEARCH:
            result = self.recovery_strategy.execute(context, self.remembered_target_gps)
            self.bus.publish(result.control_event)

            # 타겟을 다시 시각적으로 포착하면 최종 APPROACH로 전이
            if result.is_completed:
                self._transition_to(FlightState.APPROACH)