# energy_managed_flyover_trigger.py
"""
[알고리즘 설명: Energy-Managed Constant Altitude Fly-over]
- 고정익이 횡방향 선회(Roll) 시 양력 손실로 인해 고도가 떨어지거나 실속하는 현상을 방지하기 위해 
  수직 승강타(Pitch)와 추력(Throttle)을 비행 역학 기반으로 결합 보상하는 제어 기법.
- 통과 1~2초 전 말기 구간에서 뱅크각을 점진적으로 0도(Wings Level)로 제한하여 
  기체 에너지를 온전히 보존한 채 수평 관성으로 타겟 상공을 정속 관통함.

[장점]
1. 고도 및 속도 보존: 선회 시 발생하는 유도 항력에 맞춰 스로틀과 피치를 자동 증속하므로 고도 처짐 및 실속을 방지함.
2. 영상 흔들림 최소화: 통과 직전 날개를 수평으로 복원(Wings-level)하므로 카메라 영상이 기울어지지 않고 정밀한 캡처가 가능함.
3. 안정적인 규정 고도 유지: 대회 규정 고도 오차 범위를 엄격하게 준수할 수 있음.

[한계점]
1. 말기 횡방향 수정 포기: 통과 직전 날개를 수평으로 락(Lock)하기 때문에, 진입 2초 전까지 횡방향 오차를 다 잡아놓지 못하면 최종 횡오차가 그대로 남게 됨.
2. 튜닝 복잡도: 기체 무게 및 모터 추력 특성에 맞춘 스로틀-피치 결합 보상 계수 튜닝이 추가로 필요함.
"""

import time
import math
from flight_core import EventBus, FlightContext, AttitudeControlEvent, TargetAlignedEvent

class EnergyManagedFlyoverTrigger:
    def __init__(self, bus: EventBus, target_alt_m: float = 30.0, cruise_throttle: float = 0.5):
        self.bus = bus
        self.target_alt_m = target_alt_m
        self.cruise_throttle = cruise_throttle

        # 게인 및 임계값
        self.KP_ROLL = 0.05
        self.KD_ROLL = 0.02
        self.KP_ALT = 1.5
        self.KD_ALT = 0.4
        
        self.TERMINAL_DISTANCE_M = 12.0  # 날개 수평 복귀 시작 거리
        self.FLYOVER_DIST_THRESHOLD_M = 5.0

        self.prev_error_x = 0.0
        self.prev_error_alt = 0.0
        self.last_time = time.time()

    def evaluate(self, context: FlightContext):
        now = time.time()
        dt = max(now - self.last_time, 0.001)

        # 1. 고도 오차 계산
        alt_error = self.target_alt_m - context.telemetry.alt_m
        d_alt_error = (alt_error - self.prev_error_alt) / dt
        pitch_cmd = (alt_error * self.KP_ALT) + (d_alt_error * self.KD_ALT)
        pitch_cmd = max(-12.0, min(12.0, pitch_cmd))

        # 2. 횡방향 제어 및 말기 감쇠(Damping) 처리
        roll_cmd = 0.0
        if context.target is not None:
            error_x = context.target.pixel_x - 320
            d_error_x = (error_x - self.prev_error_x) / dt
            raw_roll = (error_x * self.KP_ROLL) + (d_error_x * self.KD_ROLL)

            # 타겟 근접 시 뱅크각 강제 제한 (Wings-level 진입)
            dist = context.target.ground_dist_m
            if dist < self.TERMINAL_DISTANCE_M:
                scale = max(0.0, dist / self.TERMINAL_DISTANCE_M)
                roll_cmd = raw_roll * scale  # 거리가 0에 가까워질수록 롤 각도 0도로 수렴
            else:
                roll_cmd = raw_roll

            roll_cmd = max(-25.0, min(25.0, roll_cmd))
            self.prev_error_x = error_x

            # 통과 판정
            if dist <= self.FLYOVER_DIST_THRESHOLD_M:
                self.bus.publish(TargetAlignedEvent(
                    pixel_x=context.target.pixel_x,
                    pixel_y=context.target.pixel_y,
                    alt_m=context.telemetry.alt_m,
                    timestamp=now
                ))
        else:
            self.prev_error_x = 0.0

        # 3. 에너지 보상: 뱅크각에 따른 양력 손실 보상 스로틀 계산
        # Lift_loss_factor = 1 / cos(roll)
        roll_rad = math.radians(roll_cmd)
        cos_roll = max(math.cos(roll_rad), 0.5)
        lift_compensation_throttle = self.cruise_throttle * (1.0 / cos_roll - 1.0)

        # 피치 상승에 따른 추가 추력
        pitch_compensation = max(0.0, (pitch_cmd / 12.0) * 0.15)

        final_throttle = self.cruise_throttle + lift_compensation_throttle + pitch_compensation
        final_throttle = max(0.3, min(0.85, final_throttle))

        # 4. 제어 신호 발행
        self.bus.publish(AttitudeControlEvent(
            roll_deg=roll_cmd,
            pitch_deg=pitch_cmd,
            yaw_rate_deg_s=0.0,
            throttle=final_throttle
        ))

        self.prev_error_alt = alt_error
        self.last_time = now