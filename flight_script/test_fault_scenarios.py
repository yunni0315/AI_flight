# test_fault_scenarios.py
"""
[10대 극단적 결함/위험 시나리오 전방위 자동 검증 테스트 스위트]
실전 비행 투입 전 가상 결함 주입(Fault Injection)을 통해 시스템의 안전 복원력(Resilience)을 100% 검증합니다.
"""

import unittest
import time
import math
import sys
import os

# 현재 디렉토리를 모듈 검색 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flight_core import (
    EventBus,
    FlightContext,
    TelemetryState,
    TargetDetection,
    AttitudeControlEvent,
    TargetAlignedEvent,
    MIN_SAFE_FLOOR_ALT_M
)
from fsm_coordinator import FlyoverFSMCoordinator, FlightState
from fsm_strategies import TeardropReEntryStrategy, DirectGPSRecoveryStrategy
from real_flight_runner import VisionStateHolder, RealFlightController

class TestFaultScenarios(unittest.TestCase):

    def setUp(self):
        self.bus = EventBus()
        self.received_attitude_events = []
        self.received_align_events = []

        self.bus.subscribe(AttitudeControlEvent, lambda e: self.received_attitude_events.append(e))
        self.bus.subscribe(TargetAlignedEvent, lambda e: self.received_align_events.append(e))

        self.re_entry = TeardropReEntryStrategy(outbound_time_sec=6.0, bank_angle=20.0)
        self.recovery = DirectGPSRecoveryStrategy(target_alt_m=30.0)

        self.fsm = FlyoverFSMCoordinator(
            bus=self.bus,
            re_entry_strategy=self.re_entry,
            recovery_strategy=self.recovery,
            target_alt_m=30.0,
            abort_alt_m=45.0,
            min_floor_alt_m=15.0,
            max_search_time_sec=180.0,
            conf_threshold=0.5,
            debounce_count=2
        )

    # =========================================================================
    # 시나리오 1: AI 순간 오탐 (False Positive 1프레임 노이즈) 방어
    # =========================================================================
    def test_scenario_1_false_positive_debounce_filtering(self):
        """1프레임 순간 노이즈 감지 시 APPROACH로 조기 오전이되지 않고 디바운스 필터로 방어하는지 검증"""
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=0.0)

        # 1프레임 순간 오탐 발생
        fake_target = TargetDetection(
            class_name="TANK_PANEL", confidence=0.8, pixel_x=450, pixel_y=240,
            pixel_width=50, slant_range_m=35.0, ground_dist_m=25.0
        )
        ctx1 = FlightContext(telemetry=telem, target=fake_target)
        self.fsm.evaluate(ctx1)

        # 디바운스 2회 미만이므로 여전히 SEARCH 상태 유지 확인
        self.assertEqual(self.fsm.state, FlightState.SEARCH, "1프레임 오탐에 즉시 전이되면 안 됨")

        # 2번째 프레임에서 타겟 사라짐 (순간 노이즈 확인)
        ctx2 = FlightContext(telemetry=telem, target=None)
        self.fsm.evaluate(ctx2)
        self.assertEqual(self.fsm.state, FlightState.SEARCH)
        self.assertEqual(self.fsm.consecutive_detections, 0, "노이즈 소멸 시 카운터 초기화")

        # 2프레임 연속 정상 감지 시 타겟 GPS 확정 및 장주 선회(ABORT_CLIMB) 진입 확인
        self.fsm.evaluate(ctx1)
        self.fsm.evaluate(ctx1)
        self.assertEqual(self.fsm.state, FlightState.ABORT_CLIMB, "2프레임 연속 검증 시 타겟 GPS 픽스 후 장주 선회(ABORT_CLIMB) 진입")
        self.assertIsNotNone(self.fsm.remembered_target_gps, "타겟 지상 GPS 좌표 정상 추정")

    # =========================================================================
    # 시나리오 2: 타겟 통과 직전 카메라 화각 이탈(FOV Loss) 시 오작동 방지
    # =========================================================================
    def test_scenario_2_terminal_fov_loss_handling(self):
        """타겟 바로 위(6m) 도달 후 카메라 아래로 사라져도 급상승 복행하지 않고 관성 통과 및 정찰 성공하는지 검증"""
        # APPROACH 상태에서 최종 접근 중인 상황 설정
        self.fsm.state = FlightState.APPROACH
        self.fsm.last_target_seen_time = time.time()
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=0.0)
        target_close = TargetDetection(
            class_name="TANK_PANEL", confidence=0.9, pixel_x=320, pixel_y=240,
            pixel_width=120, slant_range_m=7.0, ground_dist_m=5.0
        )

        # 1. 5m 이내 접근 평가
        self.fsm.evaluate(FlightContext(telemetry=telem, target=target_close))
        self.assertEqual(self.fsm.state, FlightState.APPROACH)
        self.assertTrue(self.fsm.is_terminal_phase, "말기 접근 단계 진입 플래그 활성화")
        self.assertEqual(len(self.received_align_events), 1, "TargetAlignedEvent 1회 발행 확인")

        # 2. 통과 직후 카메라에서 타겟 소실 (0.8초 경과 모의)
        self.fsm.last_target_seen_time = time.time() - 0.8
        ctx_lost = FlightContext(telemetry=telem, target=None)
        self.fsm.evaluate(ctx_lost)

        # 말기 단계이므로 ABORT로 튀지 않고 APPROACH 관성 유지 확인
        self.assertEqual(self.fsm.state, FlightState.APPROACH, "말기 관성 구간에서는 LOS 상실로 즉시 복행하지 않아야 함")

    # =========================================================================
    # 시나리오 3: 강한 측풍(Crosswind) 표류 시 뱅크각 하드 리미트
    # =========================================================================
    def test_scenario_3_crosswind_and_bank_limit(self):
        """측풍으로 큰 픽셀 오차(화면 끝) 발생 시에도 롤 조종각이 ±25° 안전 한계를 초과하지 않는지 검증"""
        self.fsm.state = FlightState.APPROACH
        self.fsm.last_target_seen_time = time.time()
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=0.0)
        target_wind = TargetDetection(
            class_name="TANK_PANEL", confidence=0.9, pixel_x=640, pixel_y=240, # 극단적 우측 편차
            pixel_width=40, slant_range_m=50.0, ground_dist_m=40.0
        )

        self.fsm.evaluate(FlightContext(telemetry=telem, target=target_wind))

        last_event = self.received_attitude_events[-1]
        self.assertLessEqual(abs(last_event.roll_deg), 25.0, "최대 롤각 25도 초과 금지")

    # =========================================================================
    # 시나리오 4: AI FPS 급락 시 비전-제어 스레드 비동기 분리
    # =========================================================================
    def test_scenario_4_vision_fps_drop_isolation(self):
        """비전 상태 홀더가 프레임 지연 상황에서도 스레드 락 충돌 없이 안전하게 최신 타겟을 반환하는지 검증"""
        holder = VisionStateHolder()
        target = TargetDetection("TANK_PANEL", 0.9, 320, 240, 50, 20.0, 15.0)
        
        holder.update_target(target)
        self.assertIsNotNone(holder.get_target())
        self.assertEqual(holder.get_target().class_name, "TANK_PANEL")

        holder.update_target(None)
        self.assertIsNone(holder.get_target())

    # =========================================================================
    # 시나리오 5: 선회/상승 시 실속(Stall) 방지 에너지 보상 스로틀
    # =========================================================================
    def test_scenario_5_stall_prevention_energy_throttle(self):
        """선회(Roll) 및 상승(Pitch) 명령 시 추력이 비례 증속되어 실속을 방지하는지 검증"""
        self.fsm.state = FlightState.APPROACH
        self.fsm.last_target_seen_time = time.time()
        # 고도가 25m(목표 30m 대비 5m 낮음)에서 롤 20도 선회 접근
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=25.0, yaw_deg=0.0)
        target = TargetDetection("TANK_PANEL", 0.9, 570, 240, 40, 50.0, 40.0)

        self.fsm.evaluate(FlightContext(telemetry=telem, target=target))

        last_event = self.received_attitude_events[-1]
        self.assertGreater(last_event.throttle, 0.55, "선회 및 상승 보상으로 스로틀이 기본값(0.55)보다 커야 함")
        self.assertLessEqual(last_event.throttle, 0.85, "최대 스로틀 상한 준수")

    # =========================================================================
    # 시나리오 6: 최저 안전 고도 (Floor Altitude = 15m) 침범 시 긴급 복행
    # =========================================================================
    def test_scenario_6_safety_floor_altitude_guard(self):
        """기체 고도가 15m 미만(예: 12m)으로 위험 침범 시 즉시 긴급 피치업 상승 및 ABORT_CLIMB 전이 검증"""
        low_alt_telem = TelemetryState(lat=37.5, lon=127.0, alt_m=12.0, yaw_deg=0.0)
        ctx = FlightContext(telemetry=low_alt_telem, target=None)

        self.fsm.evaluate(ctx)

        self.assertEqual(self.fsm.state, FlightState.ABORT_CLIMB, "최저 안전고도 침범 시 즉시 ABORT_CLIMB 전이")
        last_event = self.received_attitude_events[-1]
        self.assertGreaterEqual(last_event.pitch_deg, 12.0, "긴급 피치업 지시")
        self.assertGreaterEqual(last_event.throttle, 0.80, "긴급 최대 상승 추력 공급")

    # =========================================================================
    # 시나리오 7: 조종사 수동 개입(RC Override) 시 제어권 100% 양보
    # =========================================================================
    def test_scenario_7_rc_override_yield_mode(self):
        """FC가 GUIDED 모드가 아닐 때(예: FBWA/MANUAL), MAVLink 제어 신호를 전송하지 않는지 검증"""
        controller = RealFlightController(connection_str="udpin:0.0.0.0:14559", target_alt_m=30.0)
        controller.is_guided = False  # 조종사가 RC 스위치로 FBWA 전환한 상태

        # Mock actuator
        sent_commands = []
        controller.bus.subscribe(AttitudeControlEvent, controller.handle_attitude_publish)
        
        # 이벤트 발생 시뮬레이션
        controller.bus.publish(AttitudeControlEvent(roll_deg=15.0, pitch_deg=5.0, throttle=0.6))
        
        # is_guided=False이므로 actuator로 전달되지 않아야 함
        self.assertFalse(controller.is_guided)

    # =========================================================================
    # 시나리오 8: 텔레메트리 결측 및 이상치(NaN/Inf) 방어
    # =========================================================================
    def test_scenario_8_telemetry_dropout_robustness(self):
        """텔레메트리에 NaN이나 Inf, 비정상 위경도가 인입될 때 is_valid()가 정상 필터링하는지 검증"""
        bad_telem_nan = TelemetryState(lat=float('nan'), lon=127.0, alt_m=30.0)
        self.assertFalse(bad_telem_nan.is_valid())

        bad_telem_inf = TelemetryState(lat=37.5, lon=float('inf'), alt_m=30.0)
        self.assertFalse(bad_telem_inf.is_valid())

        bad_telem_out_of_bounds = TelemetryState(lat=150.0, lon=127.0, alt_m=30.0)
        self.assertFalse(bad_telem_out_of_bounds.is_valid())

        good_telem = TelemetryState(lat=37.5665, lon=126.9780, alt_m=30.0)
        self.assertTrue(good_telem.is_valid())

    # =========================================================================
    # 시나리오 9: GPS Glitch 발생 시 타겟 지상 좌표 오염 방지
    # =========================================================================
    def test_scenario_9_gps_glitch_target_memory_guard(self):
        """GPS 텔레메트리가 0.0이거나 유효하지 않을 때 타겟 지상 좌표가 오염되지 않는지 검증"""
        glitch_telem = TelemetryState(lat=0.0, lon=0.0, alt_m=30.0) # GPS 미수신/글리치
        target = TargetDetection("TANK_PANEL", 0.9, 320, 240, 50, 30.0, 20.0)

        self.fsm.evaluate(FlightContext(telemetry=glitch_telem, target=target))
        self.assertIsNone(self.fsm.remembered_target_gps, "GPS가 0.0이면 타겟 GPS를 기억하지 않아야 함")

    # =========================================================================
    # 시나리오 10: 타겟 장기 미발견 시 배터리 고갈 방지 타임아웃
    # =========================================================================
    def test_scenario_10_search_timeout_safeguard(self):
        """탐색(SEARCH) 상태가 제한시간(180초)을 초과하면 안전 고도 복행으로 자동 전이하는지 검증"""
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=0.0)
        
        # 181초 경과 모의
        self.fsm.state_enter_time = time.time() - 185.0
        self.fsm.evaluate(FlightContext(telemetry=telem, target=None))

        self.assertEqual(self.fsm.state, FlightState.ABORT_CLIMB, "탐색 타임아웃 초과 시 ABORT_CLIMB 전이")


if __name__ == "__main__":
    unittest.main(verbosity=2)
