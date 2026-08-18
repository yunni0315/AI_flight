# main_example.py 예시
from flight_core import EventBus, MavlinkActuator, AttitudeControlEvent, TargetAlignedEvent, FlightContext, TelemetryState, TargetDetection
from altitude_hold_column_trigger import ColumnAlignmentAltitudeHoldTrigger

bus = EventBus()
actuator = MavlinkActuator(master=None)  # 실제 mavlink connection 객체 할당

# 핸들러 등록
bus.subscribe(AttitudeControlEvent, actuator.handle_attitude_control)
bus.subscribe(TargetAlignedEvent, actuator.handle_target_aligned)

# 알고리즘 선택 (예: 2번 알고리즘 적용)
trigger = ColumnAlignmentAltitudeHoldTrigger(bus=bus, frame_width=640, target_altitude_m=25.0)

# 프레임 루프 내부:
# context = FlightContext(telemetry=current_telemetry, target=current_target)
# trigger.evaluate(context)