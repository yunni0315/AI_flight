# fsm_coordinator.py
import math
import time
from enum import Enum, auto
from flight_core import (
    EventBus, 
    FlightContext, 
    AttitudeControlEvent, 
    TargetAlignedEvent,
    MIN_SAFE_FLOOR_ALT_M,
    DEFAULT_TARGET_ALT_M,
    DEFAULT_ABORT_ALT_M
)
from fsm_strategies import ReEntryStrategy, RecoveryStrategy

class FlightState(Enum):
    SEARCH = auto()           # 기본 탐색 비행
    APPROACH = auto()         # 타겟 통과 시도 (에너지 관리 및 PPN/정렬)
    ABORT_CLIMB = auto()      # 복행 안전 상승
    RE_ENTRY_CIRCUIT = auto() # 장주/선회 궤도 (전략 모듈 위임)
    RECOVERY_SEARCH = auto()  # 기억 좌표 기반 재진입 (전략 모듈 위임)

class FlyoverFSMCoordinator:
    """
    상태 전이 및 결함/예외 상황 관리, 전략 모듈(Circuit/Recovery)을 통합 조율하는 실전형 FSM 코디네이터
    """
    def __init__(
        self, 
        bus: EventBus, 
        re_entry_strategy: ReEntryStrategy,
        recovery_strategy: RecoveryStrategy,
        target_alt_m: float = DEFAULT_TARGET_ALT_M,
        abort_alt_m: float = DEFAULT_ABORT_ALT_M,
        min_floor_alt_m: float = MIN_SAFE_FLOOR_ALT_M,
        max_search_time_sec: float = 180.0,
        conf_threshold: float = 0.5,
        debounce_count: int = 1,
        require_reentry_after_fix: bool = True
    ):
        self.bus = bus
        self.re_entry_strategy = re_entry_strategy
        self.recovery_strategy = recovery_strategy
        self.target_alt_m = target_alt_m
        self.abort_alt_m = abort_alt_m
        self.min_floor_alt_m = min_floor_alt_m
        self.max_search_time_sec = max_search_time_sec
        self.conf_threshold = conf_threshold
        self.debounce_count = debounce_count
        self.require_reentry_after_fix = require_reentry_after_fix

        self.state = FlightState.SEARCH
        self.remembered_target_gps: tuple[float, float] | None = None
        self.last_target_seen_time = 0.0
        self.state_enter_time = time.time()
        
        # 결함 방어 내부 상태
        self.consecutive_detections = 0
        self.is_terminal_phase = False
        self.terminal_pass_completed = False
        self.locked_approach_roll = 0.0
        self.gps_fix_completed = False

    def _transition_to(self, new_state: FlightState):
        print(f"[FSM Transition] {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_enter_time = time.time()
        self.is_terminal_phase = False
        self.terminal_pass_completed = False
        self.consecutive_detections = 0

        # 진입 시 해당 전략 모듈 초기화
        if new_state == FlightState.RE_ENTRY_CIRCUIT:
            self.re_entry_strategy.reset()
        elif new_state == FlightState.RECOVERY_SEARCH:
            self.recovery_strategy.reset()

    def _update_target_memory(self, context: FlightContext):
        """비전 정보와 FC 텔레메트리로 타겟의 지상 GPS 좌표 추정 및 갱신 (GPS Glitch 방어)"""
        if context.target is not None and context.target.confidence >= self.conf_threshold:
            telem = context.telemetry
            if telem.is_valid() and telem.lat != 0.0 and telem.lon != 0.0:
                self.last_target_seen_time = time.time()
                R_EARTH = 6378137.0
                offset_x = context.target.pixel_x - 320
                heading_rad = math.radians(telem.yaw_deg) + math.atan2(offset_x, 600.0)
                dist = context.target.ground_dist_m

                d_north = dist * math.cos(heading_rad)
                d_east = dist * math.sin(heading_rad)

                lat = telem.lat + (d_north / R_EARTH) * (180.0 / math.pi)
                lon = telem.lon + (d_east / (R_EARTH * math.cos(math.radians(telem.lat)))) * (180.0 / math.pi)
                
                # 현실적인 좌표 범위 체크
                if abs(lat) <= 90.0 and abs(lon) <= 180.0:
                    self.remembered_target_gps = (lat, lon)

    def _is_unreachable(self, context: FlightContext) -> bool:
        """물리적으로 안정 통과가 불가능한 기하 조건 판단 (진입 오차 한계 초과 감지)"""
        if context.target is None or self.is_terminal_phase:
            return False
        # 거리가 너무 가까운데(15m 미만) 화면 중심 오프셋이 너무 큼(100px 초과)
        if context.target.ground_dist_m < 15.0 and abs(context.target.pixel_x - 320) > 100:
            return True
        return False

    def evaluate(self, context: FlightContext):
        now = time.time()

        # =============================================================
        # 0. 최우선 페일세이프: 최저 안전 고도 (Safety Floor) 하한 방어
        # =============================================================
        if context.telemetry.alt_m > 0.0 and context.telemetry.alt_m < self.min_floor_alt_m:
            print(f"[긴급 안전] 최저 안전 고도 하한 침범 ({context.telemetry.alt_m:.1f}m < {self.min_floor_alt_m}m) -> 긴급 상승 복행")
            if self.state != FlightState.ABORT_CLIMB:
                self._transition_to(FlightState.ABORT_CLIMB)
            # 날개 수평 + 최대 상승각 + 최대 추력
            self.bus.publish(AttitudeControlEvent(roll_deg=0.0, pitch_deg=12.0, throttle=0.85))
            return

        self._update_target_memory(context)

        # -------------------------------------------------------------
        # 1. SEARCH: 초기 탐색 상태
        # -------------------------------------------------------------
        if self.state == FlightState.SEARCH:
            # 디바운스 및 신뢰도 필터링
            if context.target is not None and context.target.confidence >= self.conf_threshold:
                self.consecutive_detections += 1
            else:
                self.consecutive_detections = 0

            # 탐색 시간 초과 페일세이프 (배터리/구역 이탈 방지)
            if (now - self.state_enter_time) > self.max_search_time_sec:
                print(f"[경고] 탐색 제한시간({self.max_search_time_sec}s) 초과 -> 안전 고도 복행 대기")
                self._transition_to(FlightState.ABORT_CLIMB)
            elif self.consecutive_detections >= self.debounce_count:
                # 좌표 추정(GPS Fix) 완료 확인
                if self.remembered_target_gps is not None:
                    self.gps_fix_completed = True
                    if self.require_reentry_after_fix:
                        print(f"[타겟 GPS 좌표 획득 완료] Target GPS: {self.remembered_target_gps} -> 정밀 정렬 진입로 형성을 위해 장주 선회(ABORT_CLIMB) 진입")
                        self._transition_to(FlightState.ABORT_CLIMB)
                    else:
                        if self._is_unreachable(context):
                            self._transition_to(FlightState.ABORT_CLIMB)
                        else:
                            self._transition_to(FlightState.APPROACH)
                else:
                    # GPS가 아직 유효하지 않으면 탐색 유지
                    alt_error = self.target_alt_m - context.telemetry.alt_m
                    pitch_cmd = max(-8.0, min(8.0, alt_error * 1.0))
                    self.bus.publish(AttitudeControlEvent(roll_deg=0.0, pitch_deg=pitch_cmd, throttle=0.5))
            else:
                # 탐색 순항 고도 유지 제어
                alt_error = self.target_alt_m - context.telemetry.alt_m
                pitch_cmd = max(-8.0, min(8.0, alt_error * 1.0))
                self.bus.publish(AttitudeControlEvent(roll_deg=0.0, pitch_deg=pitch_cmd, throttle=0.5))

        # -------------------------------------------------------------
        # 2. APPROACH: 최종 통과 시도 (에너지 관리형 피치/스로틀 연동)
        # -------------------------------------------------------------
        if self.state == FlightState.APPROACH:
            target = context.target
            
            # 말기 접근 단계(Terminal Phase, 8m 이내) 진입 체크
            if target is not None and target.ground_dist_m <= 8.0:
                self.is_terminal_phase = True

            # 예외 1: 시야 상실 (LOS Lost) - 단, 말기 단계(Terminal Phase)에서는 관성 통과 보장
            if not self.is_terminal_phase and (now - self.last_target_seen_time > 0.6):
                print("[예외 발생] 접근 중 타겟 시야 상실 -> 복행")
                self._transition_to(FlightState.ABORT_CLIMB)
                return

            # 예외 2: 접근 도중 도달 불능 궤적으로 이탈
            if self._is_unreachable(context):
                print("[예외 발생] 통과 오차 한계 초과 -> 복행")
                self._transition_to(FlightState.ABORT_CLIMB)
                return

            # 정상 통과 판정 (5m 이내 진입 시 TargetAlignedEvent 1회 발행)
            if target is not None and target.ground_dist_m <= 5.0 and not self.terminal_pass_completed:
                self.terminal_pass_completed = True
                self.bus.publish(TargetAlignedEvent(
                    pixel_x=target.pixel_x,
                    pixel_y=target.pixel_y,
                    alt_m=context.telemetry.alt_m,
                    timestamp=now
                ))

            # 롤 조종각 계산 (말기에는 Wings-level 수렴)
            if self.is_terminal_phase:
                roll_cmd = self.locked_approach_roll * 0.3  # 날개 수평 복원
            else:
                error_x = (target.pixel_x - 320) if target else 0
                roll_cmd = max(-25.0, min(25.0, error_x * 0.08))
                self.locked_approach_roll = roll_cmd

            # 고도 오차 피치 제어 (고정 0도가 아닌 목표 순항 고도 유지)
            alt_error = self.target_alt_m - context.telemetry.alt_m
            pitch_cmd = max(-10.0, min(10.0, alt_error * 1.2))

            # 롤 선회 시 양력 손실 보상 스로틀 계산 (실속 방지)
            roll_rad = math.radians(roll_cmd)
            cos_roll = max(math.cos(roll_rad), 0.5)
            lift_comp = 0.55 * (1.0 / cos_roll - 1.0)
            pitch_comp = max(0.0, (pitch_cmd / 10.0) * 0.1)
            final_throttle = max(0.4, min(0.80, 0.55 + lift_comp + pitch_comp))

            self.bus.publish(AttitudeControlEvent(
                roll_deg=roll_cmd,
                pitch_deg=pitch_cmd,
                yaw_rate_deg_s=0.0,
                throttle=final_throttle
            ))

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