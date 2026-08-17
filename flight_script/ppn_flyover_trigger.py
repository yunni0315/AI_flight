# ppn_flyover_trigger.py
"""
[알고리즘 설명: Pure Proportional Navigation (PPN) Fly-over]
- 항공기와 타겟 사이의 시선 각도(LOS Angle)의 회전율(시선 각속도, LOS Rate)을 계산하고, 
  이에 비례하는 횡가속도(a_n = N * V * los_rate)를 산출하여 기체의 뱅크각(Roll)으로 변환하는 유도 기법.
- 시선 각속도를 0으로 수렴시켜 최소 비행거리로 타겟 상공을 관통하도록 유도함.

[장점]
1. 최단 비행 경로: 타겟을 향해 불필요한 선회 없이 가장 빠른 직선 궤적으로 접근함.
2. 높은 횡방향 정밀도: 접근 속도가 빠를수록 시선 오차에 민감하게 반응하여 통과 시점의 크로스트랙 오차를 최소화함.
3. 계산량 저열: 픽셀 각도 미분과 기초 삼각함수 연산만 사용하므로 라즈베리 파이에서 매우 가볍게 동작함.

[한계점]
1. 통과 직전 특이점(Singularity): 타겟에 매우 근접하면 시선 각도가 급격히 변해 지시 롤 각도가 과도하게 튈 수 있어 접근 말기 락(Freeze) 처리가 필수적임.
2. 측풍 취약성: 정상상태 바람(Crosswind)이 강할 경우 바람 수정을 위한 적분 요소가 없어 기체가 게걸음(Crabbing)하며 미세한 횡방향 편차가 발생할 수 있음.
"""

import time
import math
from flight_core import EventBus, FlightContext, AttitudeControlEvent, TargetAlignedEvent

class PPNFlyoverTrigger:
    def __init__(self, bus: EventBus, nav_gain_n: float = 3.5, target_alt_m: float = 30.0, focal_length: float = 600.0, frame_width: int = 640):
        self.bus = bus
        self.n_gain = nav_gain_n
        self.target_alt_m = target_alt_m
        self.focal_length = focal_length
        self.center_x = frame_width // 2
        self.g = 9.81

        self.MAX_ROLL_DEG = 25.0
        self.FLYOVER_DIST_THRESHOLD_M = 5.0
        self.TERMINAL_LOCK_DIST_M = 8.0  # 특이점 방지를 위한 제어 고정 거리

        self.prev_bearing_rad = None
        self.last_time = time.time()
        self.is_terminal_phase = False
        self.locked_roll_cmd = 0.0

    def evaluate(self, context: FlightContext):
        now = time.time()
        dt = max(now - self.last_time, 0.001)

        # 1. 고도 유지 (피치 제어)
        alt_error = self.target_alt_m - context.telemetry.alt_m
        pitch_cmd = max(-10.0, min(10.0, alt_error * 1.2))

        roll_cmd = 0.0

        if context.target is not None:
            offset_x = context.target.pixel_x - self.center_x
            current_bearing_rad = math.atan2(offset_x, self.focal_length)

            # 말기 접근 단계(Terminal Phase) 진입 여부 확인
            if context.target.ground_dist_m <= self.TERMINAL_LOCK_DIST_M and not self.is_terminal_phase:
                self.is_terminal_phase = True
                # 특이점 진입 직전 계산된 롤 각도를 완화하여 고정 (직진 관성 비행)
                self.locked_roll_cmd = self.locked_roll_cmd * 0.5

            if self.is_terminal_phase:
                roll_cmd = self.locked_roll_cmd
            else:
                if self.prev_bearing_rad is not None:
                    los_rate = (current_bearing_rad - self.prev_bearing_rad) / dt
                    ground_speed = max(context.telemetry.ground_speed_mps, 10.0)

                    # 횡가속도 a_n = N * V * los_rate
                    lateral_accel = self.n_gain * ground_speed * los_rate
                    roll_rad = math.atan(lateral_accel / self.g)
                    roll_cmd = math.degrees(roll_rad)
                    roll_cmd = max(-self.MAX_ROLL_DEG, min(self.MAX_ROLL_DEG, roll_cmd))
                    self.locked_roll_cmd = roll_cmd

                self.prev_bearing_rad = current_bearing_rad

            # 타겟 상공 통과 판정
            if context.target.ground_dist_m <= self.FLYOVER_DIST_THRESHOLD_M:
                self.bus.publish(TargetAlignedEvent(
                    pixel_x=context.target.pixel_x,
                    pixel_y=context.target.pixel_y,
                    alt_m=context.telemetry.alt_m,
                    timestamp=now
                ))
        else:
            self.prev_bearing_rad = None
            self.is_terminal_phase = False

        # 2. 이벤트 발행
        self.bus.publish(AttitudeControlEvent(
            roll_deg=roll_cmd,
            pitch_deg=pitch_cmd,
            yaw_rate_deg_s=0.0,
            throttle=0.55
        ))

        self.last_time = now