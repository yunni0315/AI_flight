# sitl_flight_runner.py
import sys
import time
import math
from pymavlink import mavutil

from flight_core import (
    EventBus,
    MavlinkActuator,
    AttitudeControlEvent,
    TargetAlignedEvent,
    FlightContext,
    TelemetryState,
    TargetDetection
)
from fsm_coordinator import FlyoverFSMCoordinator, FlightState
from fsm_strategies import TeardropReEntryStrategy, DirectGPSRecoveryStrategy

def calculate_bearing(lat1, lon1, lat2, lon2):
    """두 위경도 좌표 간의 방위각(Bearing in degrees) 계산"""
    d_lon = math.radians(lon2 - lon1)
    y = math.sin(d_lon) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def calculate_distance(lat1, lon1, lat2, lon2):
    """두 위경도 좌표 간의 지표면 수평 거리(m) 계산 (Haversine 간이식)"""
    R = 6378137.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2)**2
    return 2 * R * math.asin(math.sqrt(a))

def main():
    # 1. MAVLink 연결 (Mission Planner UDP 포워딩 14552 수신 대기)
    connection_string = "udpin:0.0.0.0:14552"
    print(f"[*] MAVLink 연결 대기 중... ({connection_string})")
    print("    (Mission Planner에서 Ctrl+F -> Mavlink -> UDP Client 127.0.0.1:14552 연결 확인)")

    try:
        master = mavutil.mavlink_connection(connection_string)
    except Exception as e:
        print(f"[!] MAVLink 포트 열기 실패: {e}")
        return

    # Heartbeat 수신 대기
    master.wait_heartbeat()
    print(f"[+] Heartbeat 수신 완료! (System ID: {master.target_system}, Component ID: {master.target_component})")

    # 2. EventBus 및 Actuator 세팅
    bus = EventBus()
    actuator = MavlinkActuator(master=master)

    bus.subscribe(AttitudeControlEvent, actuator.handle_attitude_control)
    bus.subscribe(TargetAlignedEvent, actuator.handle_target_aligned)

    # 3. FSM Coordinator 및 전략 모듈 인스턴스화
    re_entry_strat = TeardropReEntryStrategy(outbound_time_sec=6.0, bank_angle=20.0)
    recovery_strat = DirectGPSRecoveryStrategy(target_alt_m=30.0)

    fsm = FlyoverFSMCoordinator(
        bus=bus,
        re_entry_strategy=re_entry_strat,
        recovery_strategy=recovery_strat,
        target_alt_m=30.0,
        abort_alt_m=45.0
    )

    # 4. 텔레메트리 변수 초기화
    curr_lat = 0.0
    curr_lon = 0.0
    curr_alt = 0.0
    curr_roll = 0.0
    curr_pitch = 0.0
    curr_yaw = 0.0
    curr_speed = 0.0

    # 가상 모의 타겟 (초기에는 None, 첫 GPS 수신 후 기체 전방 120m에 자동 생성)
    virtual_target_gps = None
    target_spawned = False

    last_eval_time = time.time()
    last_print_time = time.time()

    print("\n=======================================================")
    print(" 🚀 고정익 정찰기 FSM 자율 제어 루프 가동 시작")
    print(" (QGC에서 기체를 GUIDED 모드로 변경하면 제어 명령이 반영됩니다)")
    print("=======================================================\n")

    while True:
        # MAVLink 메시지 논블로킹 수신
        msg = master.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'HEARTBEAT'], blocking=False)

        if msg:
            mtype = msg.get_type()
            if mtype == 'ATTITUDE':
                curr_roll = math.degrees(msg.roll)
                curr_pitch = math.degrees(msg.pitch)
                curr_yaw = math.degrees(msg.yaw)
            elif mtype == 'GLOBAL_POSITION_INT':
                curr_lat = msg.lat / 1e7
                curr_lon = msg.lon / 1e7
                curr_alt = msg.relative_alt / 1000.0  # 상대 고도 (m)
            elif mtype == 'VFR_HUD':
                curr_speed = msg.groundspeed

        # 첫 GPS 수신 시 가상 타겟(탱크) 자동 배치 (기체 현재 위치에서 북동쪽 120m)
        if not target_spawned and curr_lat != 0.0 and curr_lon != 0.0:
            # 기체 진행 방향 앞쪽 120m 지점에 가상 타겟 배치
            heading_rad = math.radians(curr_yaw if curr_yaw != 0.0 else 45.0)
            R_EARTH = 6378137.0
            d_north = 120.0 * math.cos(heading_rad)
            d_east = 120.0 * math.sin(heading_rad)
            t_lat = curr_lat + (d_north / R_EARTH) * (180.0 / math.pi)
            t_lon = curr_lon + (d_east / (R_EARTH * math.cos(math.radians(curr_lat)))) * (180.0 / math.pi)
            virtual_target_gps = (t_lat, t_lon)
            target_spawned = True
            print(f"[+] [가상 타겟 생성] Target GPS: ({t_lat:.6f}, {t_lon:.6f}) (전방 약 120m)")

        # 제어 루프 주기: 10Hz (0.1초마다 FSM 평가)
        now = time.time()
        if now - last_eval_time >= 0.1:
            last_eval_time = now

            # 현재 텔레메트리 상태 패키징
            telemetry = TelemetryState(
                lat=curr_lat,
                lon=curr_lon,
                alt_m=curr_alt,
                roll_deg=curr_roll,
                pitch_deg=curr_pitch,
                yaw_deg=curr_yaw,
                ground_speed_mps=curr_speed
            )

            # 가상 카메라 비전 모델 시뮬레이션:
            # 기체와 타겟 사이의 거리/각도를 계산하여 카메라 화각(FOV) 내에 들어오면 TargetDetection 생성
            sim_target = None
            if target_spawned and virtual_target_gps:
                t_lat, t_lon = virtual_target_gps
                dist_m = calculate_distance(curr_lat, curr_lon, t_lat, t_lon)
                bearing = calculate_bearing(curr_lat, curr_lon, t_lat, t_lon)
                angle_offset = (bearing - curr_yaw + 180) % 360 - 180

                # 카메라 FOV(좌우 35도 이내) 및 거리 250m 이내일 때 화면에 감지된 것으로 시뮬레이션
                if abs(angle_offset) < 35.0 and dist_m < 250.0:
                    focal_px = 600.0
                    screen_cx = 320
                    # 편각에 따른 픽셀 오프셋
                    pixel_x = int(screen_cx + math.tan(math.radians(angle_offset)) * focal_px)
                    pixel_y = 240
                    slant_m = math.sqrt(dist_m**2 + curr_alt**2)
                    sim_target = TargetDetection(
                        class_name="TANK_PANEL",
                        confidence=0.92,
                        pixel_x=pixel_x,
                        pixel_y=pixel_y,
                        pixel_width=max(20, int(300.0 * focal_px / (slant_m * 100.0))),
                        slant_range_m=slant_m,
                        ground_dist_m=dist_m
                    )

            # Context 생성 및 FSM 평가 실행
            context = FlightContext(telemetry=telemetry, target=sim_target)
            fsm.evaluate(context)

            # 1초마다 콘솔 모니터링 출력
            if now - last_print_time >= 1.0:
                last_print_time = now
                target_str = f"Dist: {sim_target.ground_dist_m:.1f}m, OffsetX: {sim_target.pixel_x - 320:+d}px" if sim_target else "No Target (LOS Lost)"
                print(f"[상태: {fsm.state.name:<16}] Alt: {curr_alt:4.1f}m | Yaw: {curr_yaw:5.1f}° | Roll: {curr_roll:5.1f}° | {target_str}")

        time.sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[*] FSM 자율 제어 루프를 종료합니다.")
