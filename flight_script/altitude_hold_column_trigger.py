# altitude_hold_column_trigger.py
import time
from flight_core import EventBus, FlightContext, AttitudeControlEvent, TargetAlignedEvent

class ColumnAlignmentAltitudeHoldTrigger:
    """
    타겟을 화면 가로 중앙 열(X)로만 점근 수렴시키고, 고도는 하드코딩된 타겟 고도를 유지하는 트리거
    """
    def __init__(self, bus: EventBus, frame_width: int = 640, target_altitude_m: float = 30.0):
        self.bus = bus
        self.center_x = frame_width // 2
        self.target_altitude_m = target_altitude_m

        # 수평 롤 제어 게인
        self.KP_ROLL = 0.05
        self.KD_ROLL = 0.015
        self.MAX_ROLL_DEG = 20.0

        # 고도 유지 피치/스로틀 제어 게인
        self.KP_ALT = 1.2
        self.KD_ALT = 0.5
        self.MAX_PITCH_DEG = 10.0

        self.prev_error_x = 0.0
        self.prev_error_alt = 0.0
        self.last_time = time.time()
        self.threshold_px = 20

    def evaluate(self, context: FlightContext):
        now = time.time()
        dt = max(now - self.last_time, 0.001)

        # --- A. 수직 제어: FC 텔레메트리 기반 고도 유지 루프 ---
        alt_error = self.target_altitude_m - context.telemetry.alt_m
        d_alt_error = (alt_error - self.prev_error_alt) / dt

        pitch_cmd = (alt_error * self.KP_ALT) + (d_alt_error * self.KD_ALT)
        pitch_cmd = max(-self.MAX_PITCH_DEG, min(self.MAX_PITCH_DEG, pitch_cmd))

        # 고도 유지용 스로틀 연동
        throttle_cmd = 0.5 + (pitch_cmd / self.MAX_PITCH_DEG) * 0.1
        throttle_cmd = max(0.4, min(0.7, throttle_cmd))

        # --- B. 수평 제어: 비전 타겟의 중앙 열 점근 근사 루프 ---
        if context.target is not None:
            error_x = context.target.pixel_x - self.center_x
            d_error_x = (error_x - self.prev_error_x) / dt

            roll_cmd = (error_x * self.KP_ROLL) + (d_error_x * self.KD_ROLL)
            roll_cmd = max(-self.MAX_ROLL_DEG, min(self.MAX_ROLL_DEG, roll_cmd))

            # 중앙 열 안착 판정
            if abs(error_x) <= self.threshold_px:
                self.bus.publish(TargetAlignedEvent(
                    pixel_x=context.target.pixel_x,
                    pixel_y=context.target.pixel_y,
                    alt_m=context.telemetry.alt_m,
                    timestamp=now
                ))

            self.prev_error_x = error_x
        else:
            # 타겟 미식별 시 수평 윙 레벨 유지
            roll_cmd = 0.0

        # --- C. 통합 제어 신호 발행 ---
        self.bus.publish(AttitudeControlEvent(
            roll_deg=roll_cmd,
            pitch_deg=pitch_cmd,
            yaw_rate_deg_s=0.0,
            throttle=throttle_cmd
        ))

        self.prev_error_alt = alt_error
        self.last_time = now