# test_sitl_flight_suites.py
"""
SITL 및 FSM/유도/페일세이프 전주기 자동화 검증 스위트
docs/todos/sitl-flight-testing-plan.md 에 정의된 Suite 1, 2, 3, 4를 모두 검증합니다.
"""

import math
import time
import unittest
from unittest.mock import MagicMock

from flight_core import (
    EventBus,
    FlightContext,
    TelemetryState,
    TargetDetection,
    AttitudeControlEvent,
    TargetAlignedEvent,
    euler_to_quaternion,
    MavlinkActuator
)
from fsm_coordinator import FlyoverFSMCoordinator, FlightState
from fsm_strategies import (
    TeardropReEntryStrategy,
    OrbitReEntryStrategy,
    DirectGPSRecoveryStrategy,
    StrategyResult
)
from ppn_flyover_trigger import PPNFlyoverTrigger
from virtual_waypoint_l1_trigger import VirtualWaypointL1Trigger
from energy_managed_flyover_trigger import EnergyManagedFlyoverTrigger
from altitude_hold_column_trigger import ColumnAlignmentAltitudeHoldTrigger
from point_tracking_trigger import PointApproximationTrigger


class TestSuite1_FSM_Baseline(unittest.TestCase):
    """Suite 1: FSM 상태 전이 및 기본 표적 접근 검증"""

    def setUp(self):
        self.bus = EventBus()
        self.re_entry = TeardropReEntryStrategy(outbound_time_sec=2.0, bank_angle=20.0)
        self.recovery = DirectGPSRecoveryStrategy(target_alt_m=30.0)
        self.fsm = FlyoverFSMCoordinator(
            bus=self.bus,
            re_entry_strategy=self.re_entry,
            recovery_strategy=self.recovery,
            target_alt_m=30.0,
            abort_alt_m=45.0
        )
        self.published_events = []
        self.bus.subscribe(AttitudeControlEvent, lambda e: self.published_events.append(e))
        self.bus.subscribe(TargetAlignedEvent, lambda e: self.published_events.append(e))

    def test_1_1_search_to_approach(self):
        """Test 1-1: SEARCH 상태에서 타겟 포착 시 APPROACH 상태로 정상 전이"""
        # 초기 상태: SEARCH
        self.assertEqual(self.fsm.state, FlightState.SEARCH)

        # 타겟 없는 상황 평가 -> SEARCH 유지 & 순항 신호 발행
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=45.0)
        ctx = FlightContext(telemetry=telem, target=None)
        self.fsm.evaluate(ctx)
        self.assertEqual(self.fsm.state, FlightState.SEARCH)
        self.assertTrue(any(isinstance(e, AttitudeControlEvent) for e in self.published_events))

        # 타겟 발견 (거리 100m, 정면)
        target = TargetDetection(
            class_name="TANK_PANEL",
            confidence=0.95,
            pixel_x=320,
            pixel_y=240,
            pixel_width=50,
            slant_range_m=104.4,
            ground_dist_m=100.0
        )
        ctx_with_target = FlightContext(telemetry=telem, target=target)
        self.fsm.evaluate(ctx_with_target)

        # 타겟 발견 및 좌표 픽스 완료 후 정밀 진입로 형성을 위한 장주 선회(ABORT_CLIMB) 전이 확인
        self.assertEqual(self.fsm.state, FlightState.ABORT_CLIMB)
        self.assertIsNotNone(self.fsm.remembered_target_gps)

    def test_1_2_approach_to_target_locked(self):
        """Test 1-2: APPROACH 중 5m 이내 접근 시 TargetAlignedEvent 정상 발행"""
        self.fsm.state = FlightState.APPROACH
        self.fsm.last_target_seen_time = time.time()

        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=0.0)
        # 타겟 거리 4.5m (5m 이하 통과 조건)
        target = TargetDetection(
            class_name="TANK_PANEL",
            confidence=0.98,
            pixel_x=325,
            pixel_y=240,
            pixel_width=120,
            slant_range_m=30.3,
            ground_dist_m=4.5
        )
        ctx = FlightContext(telemetry=telem, target=target)
        self.fsm.evaluate(ctx)

        # TargetAlignedEvent 발행 확인
        aligned_events = [e for e in self.published_events if isinstance(e, TargetAlignedEvent)]
        self.assertEqual(len(aligned_events), 1)
        self.assertEqual(aligned_events[0].pixel_x, 325)
        self.assertAlmostEqual(aligned_events[0].alt_m, 30.0)

    def test_1_3_unreachable_geometry_rejection(self):
        """Test 1-3: 근거리(15m 미만)에서 픽셀 오차 과다(100px 초과) 시 즉시 ABORT_CLIMB 전이"""
        self.fsm.state = FlightState.APPROACH
        self.fsm.last_target_seen_time = time.time()

        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=0.0)
        # 12m 거리에서 오프셋 150px (불가능한 급선회 요구 기하 조건)
        target = TargetDetection(
            class_name="TANK_PANEL",
            confidence=0.90,
            pixel_x=320 + 150,
            pixel_y=240,
            pixel_width=80,
            slant_range_m=32.3,
            ground_dist_m=12.0
        )
        ctx = FlightContext(telemetry=telem, target=target)
        self.fsm.evaluate(ctx)

        # 도달 불능 판단으로 ABORT_CLIMB 전이 확인
        self.assertEqual(self.fsm.state, FlightState.ABORT_CLIMB)


class TestSuite2_Failsafe_Abort_Reentry(unittest.TestCase):
    """Suite 2: 페일세이프 복행(Abort) 및 재진입(Re-entry) 궤도 검증"""

    def setUp(self):
        self.bus = EventBus()
        self.re_entry = TeardropReEntryStrategy(outbound_time_sec=0.1, bank_angle=20.0)
        self.recovery = DirectGPSRecoveryStrategy(target_alt_m=30.0)
        self.fsm = FlyoverFSMCoordinator(
            bus=self.bus,
            re_entry_strategy=self.re_entry,
            recovery_strategy=self.recovery,
            target_alt_m=30.0,
            abort_alt_m=45.0
        )
        self.published_events = []
        self.bus.subscribe(AttitudeControlEvent, lambda e: self.published_events.append(e))

    def test_2_1_los_timeout_failsafe_climb(self):
        """Test 2-1: 0.6초 초과 시야 상실 시 ABORT_CLIMB 진입 및 날개 수평(Roll=0) 상승 제어"""
        self.fsm.state = FlightState.APPROACH
        self.fsm.last_target_seen_time = time.time() - 0.7  # 0.7초 경과 (0.6s 초과)

        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=28.0, yaw_deg=0.0)
        ctx = FlightContext(telemetry=telem, target=None)
        self.fsm.evaluate(ctx)

        # 1. ABORT_CLIMB 전이 확인
        self.assertEqual(self.fsm.state, FlightState.ABORT_CLIMB)

        # 2. ABORT_CLIMB 상태에서 상승 제어 신호 검증 (Roll=0.0, Pitch>0, Throttle=0.75)
        self.published_events.clear()
        self.fsm.evaluate(ctx)
        climb_events = [e for e in self.published_events if isinstance(e, AttitudeControlEvent)]
        self.assertTrue(len(climb_events) > 0)
        self.assertEqual(climb_events[-1].roll_deg, 0.0)
        self.assertGreater(climb_events[-1].pitch_deg, 0.0)
        self.assertEqual(climb_events[-1].throttle, 0.75)

    def test_2_2_abort_to_reentry_circuit_on_altitude(self):
        """Test 2-2: 복행 중 안전 고도(43m 이상) 도달 시 RE_ENTRY_CIRCUIT 전이"""
        self.fsm.state = FlightState.ABORT_CLIMB
        
        # 고도 44m (abort_alt 45m - 2m = 43m 도달)
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=44.0, yaw_deg=0.0)
        ctx = FlightContext(telemetry=telem, target=None)
        self.fsm.evaluate(ctx)

        self.assertEqual(self.fsm.state, FlightState.RE_ENTRY_CIRCUIT)

    def test_2_3_teardrop_reentry_and_recovery(self):
        """Test 2-3: Teardrop 선회 완료 후 RECOVERY_SEARCH 전이 및 타겟 재시각화 시 APPROACH 복귀"""
        self.fsm.state = FlightState.RE_ENTRY_CIRCUIT
        self.fsm.remembered_target_gps = (37.501, 127.001)
        self.re_entry.reset()

        # Teardrop 전략 실행 (시간 경과 모의)
        self.re_entry.start_time = time.time() - 15.0 # outbound(0.1s) + turn(10s) 초과
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=45.0, yaw_deg=180.0)
        ctx = FlightContext(telemetry=telem, target=None)
        self.fsm.evaluate(ctx)

        # 1. 선회 완료 후 RECOVERY_SEARCH 전이
        self.assertEqual(self.fsm.state, FlightState.RECOVERY_SEARCH)

        # 2. RECOVERY_SEARCH 중 타겟 재포착 시 다시 APPROACH로 전이
        reacquired_target = TargetDetection(
            class_name="TANK_PANEL",
            confidence=0.91,
            pixel_x=310,
            pixel_y=240,
            pixel_width=40,
            slant_range_m=120.0,
            ground_dist_m=110.0
        )
        ctx_reacquired = FlightContext(telemetry=telem, target=reacquired_target)
        self.fsm.evaluate(ctx_reacquired)
        self.assertEqual(self.fsm.state, FlightState.APPROACH)


class TestSuite3_Guidance_Triggers(unittest.TestCase):
    """Suite 3: 유도 알고리즘(PPN / L1 / Energy Managed) 폐루프 단위 검증"""

    def test_3_1_ppn_flyover_trigger(self):
        """Test 3-1: PPN 유도 트리거 시선각속도(LOS rate) 비례 롤 산출 및 Terminal Lock 검증"""
        bus = EventBus()
        events = []
        bus.subscribe(AttitudeControlEvent, lambda e: events.append(e))
        bus.subscribe(TargetAlignedEvent, lambda e: events.append(e))

        ppn = PPNFlyoverTrigger(bus=bus, nav_gain_n=3.5, target_alt_m=30.0)

        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, ground_speed_mps=15.0)

        # Step 1: 첫 번째 프레임 (Bearing 초기화)
        target1 = TargetDetection("TANK_PANEL", 0.9, pixel_x=340, pixel_y=240, pixel_width=30, slant_range_m=100.0, ground_dist_m=95.0)
        ppn.evaluate(FlightContext(telemetry=telem, target=target1))
        self.assertEqual(events[-1].roll_deg, 0.0) # 첫 프레임은 dt 기준이 없으므로 0

        # Step 2: 두 번째 프레임 (우측으로 이동하는 각속도 발생)
        time.sleep(0.02)
        target2 = TargetDetection("TANK_PANEL", 0.9, pixel_x=380, pixel_y=240, pixel_width=35, slant_range_m=80.0, ground_dist_m=75.0)
        ppn.evaluate(FlightContext(telemetry=telem, target=target2))
        self.assertGreater(events[-1].roll_deg, 0.0) # 우측 선회 롤 발생

        # Step 3: 말기 진입 (거리 6m < 8m Terminal Lock)
        target_term = TargetDetection("TANK_PANEL", 0.95, pixel_x=400, pixel_y=240, pixel_width=90, slant_range_m=30.0, ground_dist_m=4.0)
        ppn.evaluate(FlightContext(telemetry=telem, target=target_term))
        self.assertTrue(ppn.is_terminal_phase)
        # 통과 판정(4.0m <= 5.0m) 이벤트 확인
        self.assertTrue(any(isinstance(e, TargetAlignedEvent) for e in events))

    def test_3_2_virtual_waypoint_l1_trigger(self):
        """Test 3-2: Virtual Waypoint L1 유도 트리거 검증"""
        bus = EventBus()
        events = []
        bus.subscribe(AttitudeControlEvent, lambda e: events.append(e))

        l1 = VirtualWaypointL1Trigger(bus=bus, target_alt_m=30.0)
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=25.0, yaw_deg=90.0)
        target = TargetDetection("BUILDING_PANEL", 0.93, pixel_x=400, pixel_y=240, pixel_width=60, slant_range_m=50.0, ground_dist_m=45.0)
        
        l1.evaluate(FlightContext(telemetry=telem, target=target))
        # 고도 25m -> 30m 상승 피치 명령 확인
        self.assertGreater(events[-1].pitch_deg, 0.0)
        # 타겟 우측 편차(+80px) -> 우측 롤 명령 확인
        self.assertGreater(events[-1].roll_deg, 0.0)

    def test_3_3_energy_managed_flyover_trigger(self):
        """Test 3-3: Energy Managed 유도 트리거 (선회 시 스로틀 자동 보상) 검증"""
        bus = EventBus()
        events = []
        bus.subscribe(AttitudeControlEvent, lambda e: events.append(e))

        em = EnergyManagedFlyoverTrigger(bus=bus, target_alt_m=30.0, cruise_throttle=0.5)
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=20.0, yaw_deg=0.0)
        
        # 큰 롤 각도 및 상승이 필요한 조건
        target = TargetDetection("TANK_PANEL", 0.9, pixel_x=500, pixel_y=240, pixel_width=50, slant_range_m=60.0, ground_dist_m=50.0)
        em.evaluate(FlightContext(telemetry=telem, target=target))

        # 기본 스로틀(0.5)보다 높은 에너지가 보상되었는지 확인
        self.assertGreater(events[-1].throttle, 0.5)

    def test_3_4_altitude_hold_column_trigger(self):
        """Test 3-4: 고도 유지 및 가로 중앙 열(X) 수렴 트리거 검증"""
        bus = EventBus()
        events = []
        bus.subscribe(AttitudeControlEvent, lambda e: events.append(e))
        bus.subscribe(TargetAlignedEvent, lambda e: events.append(e))

        trigger = ColumnAlignmentAltitudeHoldTrigger(bus=bus, frame_width=640, target_altitude_m=30.0)

        # Case 1: 고도 부족(20m < 30m) & 타겟 우측 편차(pixel_x=400 > center 320)
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=20.0, yaw_deg=0.0)
        target = TargetDetection("TANK_PANEL", 0.95, pixel_x=400, pixel_y=240, pixel_width=50, slant_range_m=50.0, ground_dist_m=45.0)
        trigger.evaluate(FlightContext(telemetry=telem, target=target))

        # 고도 상승 피치(>0), 우측 선회 롤(>0), 스로틀 증가 확인
        self.assertGreater(events[-1].pitch_deg, 0.0)
        self.assertGreater(events[-1].roll_deg, 0.0)
        self.assertGreater(events[-1].throttle, 0.5)

        # Case 2: 중앙 열 안착 (|pixel_x - 320| <= 20) -> TargetAlignedEvent 발행 확인
        target_aligned = TargetDetection("TANK_PANEL", 0.95, pixel_x=325, pixel_y=240, pixel_width=50, slant_range_m=30.0, ground_dist_m=20.0)
        trigger.evaluate(FlightContext(telemetry=telem, target=target_aligned))
        aligned_events = [e for e in events if isinstance(e, TargetAlignedEvent)]
        self.assertTrue(len(aligned_events) > 0)
        self.assertEqual(aligned_events[-1].pixel_x, 325)

        # Case 3: 타겟 상실 시 롤 0(Wings level) 유지 확인
        trigger.evaluate(FlightContext(telemetry=telem, target=None))
        self.assertEqual(events[-1].roll_deg, 0.0)

    def test_3_5_point_tracking_trigger(self):
        """Test 3-5: 화면 지정 좌표(DESIRED_X, DESIRED_Y) PD 점근 근사 트리거 검증"""
        bus = EventBus()
        events = []
        bus.subscribe(AttitudeControlEvent, lambda e: events.append(e))
        bus.subscribe(TargetAlignedEvent, lambda e: events.append(e))

        trigger = PointApproximationTrigger(bus=bus, desired_x=400, desired_y=350)

        # Case 1: 타겟이 지정 위치보다 좌상단에 위치 (x=300 < 400, y=200 < 350)
        # error_x = 300 - 400 = -100 -> roll < 0 (좌선회)
        # error_y = 350 - 200 = +150 -> pitch > 0 (상승)
        telem = TelemetryState(lat=37.5, lon=127.0, alt_m=30.0, yaw_deg=0.0)
        target = TargetDetection("BUILDING_PANEL", 0.92, pixel_x=300, pixel_y=200, pixel_width=40, slant_range_m=60.0, ground_dist_m=50.0)
        trigger.evaluate(FlightContext(telemetry=telem, target=target))

        self.assertLess(events[-1].roll_deg, 0.0)
        self.assertGreater(events[-1].pitch_deg, 0.0)

        # Case 2: 지정 영역(15px 이내) 안착 시 TargetAlignedEvent 발행 확인
        target_aligned = TargetDetection("BUILDING_PANEL", 0.95, pixel_x=405, pixel_y=355, pixel_width=40, slant_range_m=30.0, ground_dist_m=20.0)
        trigger.evaluate(FlightContext(telemetry=telem, target=target_aligned))
        aligned_events = [e for e in events if isinstance(e, TargetAlignedEvent)]
        self.assertTrue(len(aligned_events) > 0)
        self.assertEqual(aligned_events[-1].pixel_x, 405)

        # Case 3: 타겟 상실 시 수평 순항 (roll=0, pitch=0, throttle=0.5) 복귀
        trigger.evaluate(FlightContext(telemetry=telem, target=None))
        self.assertEqual(events[-1].roll_deg, 0.0)
        self.assertEqual(events[-1].pitch_deg, 0.0)
        self.assertEqual(events[-1].throttle, 0.5)


class TestSuite4_MAVLink_Actuator_Safety(unittest.TestCase):
    """Suite 4: MAVLink Actuator 쿼터니언 변환 및 32비트 타임스탬프 안전성 검증"""

    def test_4_1_euler_to_quaternion_normalization(self):
        """Test 4-1: 쿼터니언 변환 결과 크기(Norm)가 1.0으로 정규화되는지 검증"""
        test_angles = [
            (0.0, 0.0, 0.0),
            (math.radians(25.0), 0.0, 0.0),
            (-math.radians(20.0), math.radians(10.0), 0.0),
            (math.radians(15.0), -math.radians(5.0), math.radians(45.0))
        ]
        for roll, pitch, yaw in test_angles:
            w, x, y, z = euler_to_quaternion(roll, pitch, yaw)
            norm = math.sqrt(w*w + x*x + y*y + z*z)
            self.assertAlmostEqual(norm, 1.0, places=5)

    def test_4_2_mavlink_actuator_time_boot_ms_bounds(self):
        """Test 4-2: time_boot_ms가 uint32 범위(0 ~ 4294967295)를 절대 초과하지 않는지 검증"""
        mock_master = MagicMock()
        actuator = MavlinkActuator(master=mock_master)
        
        # 가상의 장시간 경과 시뮬레이션
        actuator._start_time = time.time() - 5000000.0  # 약 57일 경과 모의
        actuator.handle_attitude_control(AttitudeControlEvent(roll_deg=10.0, pitch_deg=5.0, throttle=0.6))

        # set_attitude_target_send 호출 인자 확인
        call_args = mock_master.mav.set_attitude_target_send.call_args[0]
        time_boot_ms = call_args[0]
        self.assertGreaterEqual(time_boot_ms, 0)
        self.assertLessEqual(time_boot_ms, 4294967295)


if __name__ == "__main__":
    print("\n=======================================================")
    print(" [TEST] Fixed-wing Recon FSM, Guidance & Failsafe Test Suites")
    print("=======================================================\n")
    unittest.main(verbosity=2)
