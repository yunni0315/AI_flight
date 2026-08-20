# simulate_mission_lifecycle.py
"""
가상 비행 역학(Flight Dynamics) 기반 FSM 전주기 폐루프 시뮬레이션 러너
"""

import time
import math
from flight_core import (
    EventBus,
    FlightContext,
    TelemetryState,
    TargetDetection,
    AttitudeControlEvent,
    TargetAlignedEvent
)
from fsm_coordinator import FlyoverFSMCoordinator, FlightState
from fsm_strategies import TeardropReEntryStrategy, DirectGPSRecoveryStrategy

def run_mission_simulation():
    bus = EventBus()
    re_entry = TeardropReEntryStrategy(outbound_time_sec=4.0, bank_angle=22.0)
    recovery = DirectGPSRecoveryStrategy(target_alt_m=30.0)
    fsm = FlyoverFSMCoordinator(
        bus=bus,
        re_entry_strategy=re_entry,
        recovery_strategy=recovery,
        target_alt_m=30.0,
        abort_alt_m=45.0
    )

    current_cmd = AttitudeControlEvent(roll_deg=0.0, pitch_deg=0.0, throttle=0.5)
    flyover_events = []

    bus.subscribe(AttitudeControlEvent, lambda e: globals().update(current_cmd=e))
    bus.subscribe(TargetAlignedEvent, lambda e: flyover_events.append(e))

    # 기체 초기 상태 (고도 30m, 남쪽에서 북쪽으로 15m/s 비행)
    plane_lat = 37.500000
    plane_lon = 127.000000
    plane_alt = 30.0
    plane_roll = 0.0
    plane_pitch = 0.0
    plane_yaw = 0.0 # 북쪽(0도)
    plane_speed = 15.0 # m/s

    # 타겟 위치 (기체 북쪽 150m 지점)
    R_EARTH = 6378137.0
    target_lat = plane_lat + (150.0 / R_EARTH) * (180.0 / math.pi)
    target_lon = plane_lon

    dt = 0.1 # 10Hz
    sim_time = 0.0

    print("=" * 85)
    print(f"{'Time(s)':^7} | {'State':^18} | {'Alt(m)':^6} | {'Dist(m)':^7} | {'Roll(deg)':^9} | {'Pitch(deg)':^10} | {'Throttle':^8}")
    print("=" * 85)

    logs = []
    while sim_time <= 25.0:
        # 기체와 타겟 거리 및 상대각 계산
        d_lat = math.radians(target_lat - plane_lat)
        d_lon = math.radians(target_lon - plane_lon)
        dist_m = 2 * R_EARTH * math.asin(math.sqrt(math.sin(d_lat/2)**2 + math.cos(math.radians(plane_lat))*math.cos(math.radians(target_lat))*math.sin(d_lon/2)**2))

        y = math.sin(d_lon) * math.cos(math.radians(target_lat))
        x = math.cos(math.radians(plane_lat)) * math.sin(math.radians(target_lat)) - \
            math.sin(math.radians(plane_lat)) * math.cos(math.radians(target_lat)) * math.cos(d_lon)
        bearing_deg = (math.degrees(math.atan2(y, x)) + 360) % 360
        angle_offset_deg = (bearing_deg - plane_yaw + 180) % 360 - 180

        # 비전 탐지 모델링 (FOV +-35도, 거리 200m 이내)
        target_det = None
        # 시뮬레이션 시나리오: 7.0초 ~ 8.5초 사이에 임의로 타겟 시야 유실(LOS Lost) 주입하여 복행/재진입 유도
        is_los_injected = (7.0 <= sim_time <= 9.0)

        if not is_los_injected and abs(angle_offset_deg) < 35.0 and dist_m < 200.0:
            focal_px = 600.0
            screen_cx = 320
            px_x = int(screen_cx + math.tan(math.radians(angle_offset_deg)) * focal_px)
            slant_m = math.sqrt(dist_m**2 + plane_alt**2)
            target_det = TargetDetection(
                class_name="TANK_PANEL",
                confidence=0.94,
                pixel_x=px_x,
                pixel_y=240,
                pixel_width=max(20, int(300.0 * focal_px / (slant_m * 100.0))),
                slant_range_m=slant_m,
                ground_dist_m=dist_m
            )

        telem = TelemetryState(
            lat=plane_lat,
            lon=plane_lon,
            alt_m=plane_alt,
            roll_deg=plane_roll,
            pitch_deg=plane_pitch,
            yaw_deg=plane_yaw,
            ground_speed_mps=plane_speed
        )

        ctx = FlightContext(telemetry=telem, target=target_det)
        fsm.evaluate(ctx)

        # 비행 역학 간이 전파 (Euler Forward)
        # Roll 명령에 따른 Yaw 각속도: yaw_rate = (g / V) * tan(roll)
        yaw_rate = (9.81 / plane_speed) * math.tan(math.radians(current_cmd.roll_deg))
        plane_yaw = (plane_yaw + math.degrees(yaw_rate) * dt) % 360
        plane_roll = plane_roll + (current_cmd.roll_deg - plane_roll) * 0.4 # 1차 지연 응답
        plane_pitch = plane_pitch + (current_cmd.pitch_deg - plane_pitch) * 0.4

        # 피치에 따른 고도 변화: dAlt = V * sin(pitch)
        plane_alt += plane_speed * math.sin(math.radians(plane_pitch)) * dt

        # 위치 이동 (Lat/Lon)
        heading_rad = math.radians(plane_yaw)
        d_north = plane_speed * math.cos(heading_rad) * dt
        d_east = plane_speed * math.sin(heading_rad) * dt
        plane_lat += (d_north / R_EARTH) * (180.0 / math.pi)
        plane_lon += (d_east / (R_EARTH * math.cos(math.radians(plane_lat)))) * (180.0 / math.pi)

        if int(sim_time * 10) % 10 == 0:
            log_line = f"{sim_time:7.1f} | {fsm.state.name:18} | {plane_alt:6.1f} | {dist_m:7.1f} | {current_cmd.roll_deg:9.1f} | {current_cmd.pitch_deg:10.1f} | {current_cmd.throttle:8.2f}"
            print(log_line)
            logs.append(log_line)

        sim_time += dt

    print("=" * 85)
    print(f"Total Flyover Locked Events Fired: {len(flyover_events)}")
    return logs

if __name__ == "__main__":
    run_mission_simulation()
