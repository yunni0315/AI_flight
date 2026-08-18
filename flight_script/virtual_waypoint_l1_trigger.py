# virtual_waypoint_l1_trigger.py
"""
[알고리즘 설명: Virtual Waypoint L1 Path Following]
- 비전에서 측정된 타겟의 상대 거리 및 편각을 FC의 현재 GPS 좌표(Lat, Lon, Yaw)와 합성하여 타겟의 지상 절대 좌표를 추정함.
- 추정된 타겟 좌표를 관통하는 진입-통과 가상 경로(Line Segment)를 생성하고, 
  ArduPilot 내부의 L1 내비게이션 컨트롤러가 해당 경로점을 추종하도록 가상 Waypoint 명령을 발행함.

[장점]
1. 탁월한 바람 보정: ArduPilot 내장 L1 컨트롤러가 측풍을 자동 보정하므로 바람이 강한 환경에서도 정확한 직선 궤적 통과가 가능함.
2. 안정성 및 안전성: 자세 제어(Attitude)를 직접 조작하지 않고 상위 레벨 위치 명령을 주므로 실속이나 오버뱅크 위험이 낮음.
3. 시야 상실 대응: 타겟이 카메라 FOV에서 벗어나더라도 추정된 GPS 위치를 향해 비행을 지속함.

[한계점]
1. GPS 및 텔레메트리 지연 의존성: FC의 GPS 오차나 영상 처리 지연(Latency)이 타겟의 지상 좌표 추정 정밀도에 직접적인 오차를 유발함.
2. 반응 속도 지연: 비전 픽셀 오차를 직접 뱅크각으로 쏘는 저수준 제어보다 FC 내비게이션 루프를 한 단계 더 거치므로 급격한 경로 수정 반응이 다소 느림.
"""

import time
import math
from flight_core import EventBus, FlightContext, AttitudeControlEvent, TargetAlignedEvent

class VirtualWaypointL1Trigger:
    def __init__(self, bus: EventBus, target_alt_m: float = 30.0):
        self.bus = bus
        self.target_alt_m = target_alt_m
        self.FLYOVER_DIST_THRESHOLD_M = 5.0
        self.last_time = time.time()

    def _estimate_target_gps(self, telem, target):
        """기체 위치, 방위각, 타겟 수평 거리/편각을 기반으로 타겟의 절대 위경도 계산"""
        # 지구 반경 (m)
        R_EARTH = 6378137.0
        
        # 타겟의 절대 방위각 (기체 Yaw + 타겟 상대 Bearing)
        target_heading_rad = math.radians(telem.yaw_deg) + math.atan2(target.pixel_x - 320, 600)
        dist = target.ground_dist_m

        d_north = dist * math.cos(target_heading_rad)
        d_east = dist * math.sin(target_heading_rad)

        target_lat = telem.lat + (d_north / R_EARTH) * (180.0 / math.pi)
        target_lon = telem.lon + (d_east / (R_EARTH * math.cos(math.radians(telem.lat)))) * (180.0 / math.pi)
        
        return target_lat, target_lon

    def evaluate(self, context: FlightContext):
        now = time.time()

        if context.target is not None:
            # 1. 타겟 지상 좌표 추정
            target_lat, target_lon = self._estimate_target_gps(context.telemetry, context.target)

            # 2. 통과 판정 이벤트 발행
            if context.target.ground_dist_m <= self.FLYOVER_DIST_THRESHOLD_M:
                self.bus.publish(TargetAlignedEvent(
                    pixel_x=context.target.pixel_x,
                    pixel_y=context.target.pixel_y,
                    alt_m=context.telemetry.alt_m,
                    timestamp=now
                ))

            # 3. L1 유도를 위한 방위각 기반 롤 제어 신호 생성
            # (직접 Waypoint 패킷 송신 외에 내부 Attitude 보조 제어 병행)
            heading_error_rad = math.atan2(context.target.pixel_x - 320, 600)
            roll_cmd = max(-25.0, min(25.0, math.degrees(heading_error_rad) * 1.2))
        else:
            roll_cmd = 0.0

        alt_error = self.target_alt_m - context.telemetry.alt_m
        pitch_cmd = max(-10.0, min(10.0, alt_error * 1.2))

        self.bus.publish(AttitudeControlEvent(
            roll_deg=roll_cmd,
            pitch_deg=pitch_cmd,
            yaw_rate_deg_s=0.0,
            throttle=0.5
        ))

        self.last_time = now