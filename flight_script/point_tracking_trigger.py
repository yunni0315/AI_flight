# point_tracking_trigger.py
import time
from flight_core import EventBus, FlightContext, AttitudeControlEvent, TargetAlignedEvent

class PointApproximationTrigger:
    """
    화면 내 하드코딩된 특정 좌표(DESIRED_X, DESIRED_Y)로 타겟을 점근 근사시키는 트리거
    """
    def __init__(self, bus: EventBus, desired_x: int = 400, desired_y: int = 350):
        self.bus = bus
        self.desired_x = desired_x
        self.desired_y = desired_y

        # PD 제어 게인 및 포화 제한값
        self.KP_ROLL = 0.06
        self.KD_ROLL = 0.02
        self.MAX_ROLL_DEG = 25.0

        self.KP_PITCH = 0.04
        self.KD_PITCH = 0.01
        self.MAX_PITCH_DEG = 15.0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.last_time = time.time()
        self.threshold_px = 15

    def evaluate(self, context: FlightContext):
        if context.target is None:
            # 타겟 소실 시 수평 비행 유지
            self.bus.publish(AttitudeControlEvent(roll_deg=0.0, pitch_deg=0.0, throttle=0.5))
            return

        now = time.time()
        dt = max(now - self.last_time, 0.001)

        # 1. 화면 오차 계산 (지정 좌표 기준)
        error_x = context.target.pixel_x - self.desired_x
        error_y = self.desired_y - context.target.pixel_y  # 화면 아래쪽이 양수인 좌표계 보정

        # 2. 미분 오차 계산
        d_error_x = (error_x - self.prev_error_x) / dt
        d_error_y = (error_y - self.prev_error_y) / dt

        # 3. 롤/피치 제어량 계산 (점근 수렴)
        roll_cmd = (error_x * self.KP_ROLL) + (d_error_x * self.KD_ROLL)
        pitch_cmd = (error_y * self.KP_PITCH) + (d_error_y * self.KD_PITCH)

        # 포화 한계 적용
        roll_cmd = max(-self.MAX_ROLL_DEG, min(self.MAX_ROLL_DEG, roll_cmd))
        pitch_cmd = max(-self.MAX_PITCH_DEG, min(self.MAX_PITCH_DEG, pitch_cmd))

        # 피치 변화에 따른 보조 스로틀 조정 (상승 시 스로틀 추가)
        base_throttle = 0.5
        throttle_cmd = base_throttle + (pitch_cmd / self.MAX_PITCH_DEG) * 0.15
        throttle_cmd = max(0.3, min(0.8, throttle_cmd))

        # 4. 제어 신호 발행
        self.bus.publish(AttitudeControlEvent(
            roll_deg=roll_cmd,
            pitch_deg=pitch_cmd,
            yaw_rate_deg_s=0.0,
            throttle=throttle_cmd
        ))

        # 5. 지정 영역 안착 이벤트 발행
        if abs(error_x) <= self.threshold_px and abs(error_y) <= self.threshold_px:
            self.bus.publish(TargetAlignedEvent(
                pixel_x=context.target.pixel_x,
                pixel_y=context.target.pixel_y,
                alt_m=context.telemetry.alt_m,
                timestamp=now
            ))

        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.last_time = now