from flight_core import EventBus
from fsm_strategies import TeardropReEntryStrategy, DirectGPSRecoveryStrategy
from fsm_coordinator import FlyoverFSMCoordinator

bus = EventBus()

# 필요한 세부 알고리즘 객체 생성
circuit_algo = TeardropReEntryStrategy(outbound_time_sec=7.0, bank_angle=22.0)
recovery_algo = DirectGPSRecoveryStrategy(target_alt_m=30.0)

# FSM 코디네이터에 조립
fsm_trigger = FlyoverFSMCoordinator(
    bus=bus,
    re_entry_strategy=circuit_algo,
    recovery_strategy=recovery_algo,
    target_alt_m=30.0,
    abort_alt_m=45.0
)

# 메인 프레임 루프에서:
# fsm_trigger.evaluate(current_flight_context)