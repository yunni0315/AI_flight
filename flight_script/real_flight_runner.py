# real_flight_runner.py
"""
[실기체 전용 온보드 자율 비행 & AI 비전 통합 러너]
- 아키텍처: 비동기 듀얼 스레드 (Vision Thread ↔ 20Hz Flight Control Thread)
- 페일세이프: RC Override(수동 전환) 감지 시 MAVLink 제어 즉시 양보, 최저 안전 고도 리미트
- 통신: RPi5 UART 시리얼(/dev/ttyAMA0 등) 및 SITL UDP 자동 지원
"""

import sys
import os
import time
import math
import argparse
import threading
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pymavlink import mavutil

from flight_core import (
    EventBus,
    MavlinkActuator,
    AttitudeControlEvent,
    TargetAlignedEvent,
    FlightContext,
    TelemetryState,
    TargetDetection,
    MIN_SAFE_FLOOR_ALT_M,
    DEFAULT_TARGET_ALT_M,
    DEFAULT_ABORT_ALT_M
)
from fsm_coordinator import FlyoverFSMCoordinator, FlightState
from fsm_strategies import TeardropReEntryStrategy, DirectGPSRecoveryStrategy

# ==========================================
# 1. 전역 공유 상태 및 비전 스레드 (Vision Worker)
# ==========================================
class VisionStateHolder:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_target: TargetDetection | None = None
        self.fps: float = 0.0
        self.last_frame_time: float = 0.0
        self.is_running: bool = True

    def update_target(self, target: TargetDetection | None):
        with self.lock:
            self.latest_target = target
            now = time.time()
            if self.last_frame_time > 0:
                dt = now - self.last_frame_time
                self.fps = 1.0 / max(dt, 0.001)
            self.last_frame_time = now

    def get_target(self) -> TargetDetection | None:
        with self.lock:
            return self.latest_target

    def get_fps(self) -> float:
        with self.lock:
            return self.fps

    def stop(self):
        with self.lock:
            self.is_running = False

def run_vision_worker(
    state_holder: VisionStateHolder, 
    camera_id: int | str = 0, 
    model_path: str = "best.pt",
    focal_length: float = 600.0,
    known_width_cm: float = 300.0,
    conf_threshold: float = 0.5,
    mock_mode: bool = False
):
    """카메라 영상 취득 및 YOLO 추론 독립 스레드 (비행 제어 루프 블로킹 방지)"""
    if mock_mode:
        print("[Vision Thread] 모의 비전(Mock Mode)으로 실행 중 - 실제 카메라를 열지 않습니다.")
        while state_holder.is_running:
            time.sleep(0.1)
        return

    try:
        import cv2
        from ultralytics import YOLO
    except ImportError:
        print("[Vision Thread 경고] cv2 또는 ultralytics 라이브러리가 없어 모의 비전 모드로 전환합니다.")
        return

    print(f"[Vision Thread] YOLO 모델 로드 중: {model_path}")
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"[Vision Thread 오류] YOLO 모델 로드 실패 ({e}) - 모의 모드로 대기")
        return

    print(f"[Vision Thread] 카메라({camera_id}) 연결 시도 중...")
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"[Vision Thread 오류] 카메라({camera_id})를 열 수 없습니다.")
        return

    print("[Vision Thread] 카메라 스트림 및 AI 추론 가동 완료.")

    while state_holder.is_running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        img_h, img_w = frame.shape[:2]
        center_x = img_w // 2

        # Headless AI 추론 (verbose=False)
        results = model.track(frame, persist=True, conf=conf_threshold, verbose=False)
        detected_target = None

        if results and len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                score = float(box.conf[0].item())

                # 타겟 식별 (0번: TANK_PANEL 등)
                if cls_id == 0 or cls_id == 1:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    pixel_width = max(x2 - x1, 1)
                    target_cx = (x1 + x2) // 2
                    target_cy = (y1 + y2) // 2

                    # 직선거리 및 수평거리 계산
                    slant_m = (known_width_cm * focal_length) / (pixel_width * 100.0)
                    # 고도 20m 기준 간이 수평거리 추정
                    ground_m = math.sqrt(max(0.0, slant_m**2 - 20.0**2)) if slant_m > 20.0 else slant_m

                    detected_target = TargetDetection(
                        class_name="TANK_PANEL" if cls_id == 0 else "BUILDING_PANEL",
                        confidence=score,
                        pixel_x=target_cx,
                        pixel_y=target_cy,
                        pixel_width=pixel_width,
                        slant_range_m=slant_m,
                        ground_dist_m=ground_m
                    )
                    break

        state_holder.update_target(detected_target)

    cap.release()
    print("[Vision Thread] 카메라 리소스 해제 완료.")


# ==========================================
# 2. 비행 제어 마스터 클래스 (Flight Controller)
# ==========================================
class RealFlightController:
    def __init__(
        self,
        connection_str: str,
        baud: int = 57600,
        target_alt_m: float = DEFAULT_TARGET_ALT_M,
        abort_alt_m: float = DEFAULT_ABORT_ALT_M,
        min_floor_alt_m: float = MIN_SAFE_FLOOR_ALT_M
    ):
        self.connection_str = connection_str
        self.baud = baud
        self.target_alt_m = target_alt_m
        self.abort_alt_m = abort_alt_m
        self.min_floor_alt_m = min_floor_alt_m

        self.bus = EventBus()
        self.master = None
        self.actuator = None
        self.fsm = None

        # 텔레메트리 캐시
        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.curr_alt = 0.0
        self.curr_roll = 0.0
        self.curr_pitch = 0.0
        self.curr_yaw = 0.0
        self.curr_speed = 0.0
        self.current_flight_mode = "UNKNOWN"
        self.is_guided = False

    def connect(self) -> bool:
        print(f"[*] MAVLink 연결 시도: {self.connection_str} (Baud: {self.baud})")
        try:
            if "udp" in self.connection_str:
                self.master = mavutil.mavlink_connection(self.connection_str)
            else:
                self.master = mavutil.mavlink_connection(self.connection_str, baud=self.baud)
        except Exception as e:
            print(f"[!] MAVLink 연결 실패: {e}")
            return False

        print("[*] FC Heartbeat 대기 중...")
        try:
            self.master.wait_heartbeat(timeout=10)
            print(f"[+] Heartbeat 수신 성공! (System: {self.master.target_system}, Component: {self.master.target_component})")
        except Exception as e:
            print(f"[!] Heartbeat 수신 타임아웃: {e}")
            return False

        # Actuator 및 FSM 초기화
        self.actuator = MavlinkActuator(master=self.master)
        self.bus.subscribe(AttitudeControlEvent, self.handle_attitude_publish)
        self.bus.subscribe(TargetAlignedEvent, self.actuator.handle_target_aligned)

        re_entry_strat = TeardropReEntryStrategy(outbound_time_sec=6.0, bank_angle=20.0)
        recovery_strat = DirectGPSRecoveryStrategy(target_alt_m=self.target_alt_m)

        self.fsm = FlyoverFSMCoordinator(
            bus=self.bus,
            re_entry_strategy=re_entry_strat,
            recovery_strategy=recovery_strat,
            target_alt_m=self.target_alt_m,
            abort_alt_m=self.abort_alt_m,
            min_floor_alt_m=self.min_floor_alt_m
        )
        return True

    def handle_attitude_publish(self, event: AttitudeControlEvent):
        """FC가 GUIDED 모드일 때만 액추에이터로 MAVLink 제어 신호 전달"""
        if self.is_guided:
            self.actuator.handle_attitude_control(event)

    def process_mavlink_messages(self):
        """논블로킹 MAVLink 메시지 수신 및 모드/텔레메트리 갱신"""
        while True:
            msg = self.master.recv_match(
                type=['HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD'], 
                blocking=False
            )
            if msg is None:
                break

            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                # 비행 모드 파싱 (ArduPlane GUIDED 모드 확인)
                mode_mapping = self.master.mode_mapping()
                if mode_mapping:
                    # custom_mode로 역조회
                    current_mode = "UNKNOWN"
                    for k, v in mode_mapping.items():
                        if v == msg.custom_mode:
                            current_mode = k
                            break
                    self.current_flight_mode = current_mode
                    self.is_guided = (current_mode == "GUIDED")
            elif mtype == 'ATTITUDE':
                self.curr_roll = math.degrees(msg.roll)
                self.curr_pitch = math.degrees(msg.pitch)
                self.curr_yaw = math.degrees(msg.yaw)
            elif mtype == 'GLOBAL_POSITION_INT':
                self.curr_lat = msg.lat / 1e7
                self.curr_lon = msg.lon / 1e7
                self.curr_alt = msg.relative_alt / 1000.0
            elif mtype == 'VFR_HUD':
                self.curr_speed = msg.groundspeed

    def run_control_loop(self, vision_state: VisionStateHolder, test_mode: bool = False):
        """20Hz 비행 제어 및 FSM 평가 메인 루프"""
        print("\n=======================================================")
        print(" 🚀 고정익 정찰기 자율 비행 통합 제어 루프 가동 시작")
        print(" (RC 스위치로 GUIDED 모드 진입 시 자동 정찰이 활성화됩니다)")
        print("=======================================================\n")

        last_loop_time = time.time()
        last_log_time = time.time()

        try:
            while True:
                now = time.time()
                self.process_mavlink_messages()

                # 20Hz (0.05s) 고정 주기 제어
                if now - last_loop_time >= 0.05:
                    last_loop_time = now

                    telemetry = TelemetryState(
                        lat=self.curr_lat,
                        lon=self.curr_lon,
                        alt_m=self.curr_alt,
                        roll_deg=self.curr_roll,
                        pitch_deg=self.curr_pitch,
                        yaw_deg=self.curr_yaw,
                        ground_speed_mps=self.curr_speed
                    )

                    target = vision_state.get_target()
                    context = FlightContext(
                        telemetry=telemetry, 
                        target=target,
                        is_guided_mode=self.is_guided
                    )

                    # FSM 평가 (FSM 내부에서 event 발행)
                    self.fsm.evaluate(context)

                # 1초 주기 상태 모니터링 출력
                if now - last_log_time >= 1.0:
                    last_log_time = now
                    mode_str = f"MODE: [{self.current_flight_mode}]" if not self.is_guided else "MODE: [GUIDED (AUTO ACTIVE)]"
                    target_info = f"TARGET: ({target.class_name} {target.ground_dist_m:.1f}m)" if target else "TARGET: None"
                    fps_str = f"Vision FPS: {vision_state.get_fps():.1f}"
                    print(f"[{self.fsm.state.name:<16}] {mode_str} | Alt: {self.curr_alt:4.1f}m | Yaw: {self.curr_yaw:5.1f}° | {target_info} | {fps_str}")

                time.sleep(0.005)

        except KeyboardInterrupt:
            print("\n[*] 제어 루프를 정상 종료합니다.")
        finally:
            vision_state.stop()


def main():
    parser = argparse.ArgumentParser(description="실기체 온보드 비행 제어 및 AI 정찰 통합 러너")
    parser.add_argument("--connection", default="udpin:0.0.0.0:14552", help="MAVLink 연결 문자열 (예: /dev/ttyAMA0 또는 udpin:0.0.0.0:14552)")
    parser.add_argument("--baud", type=int, default=57600, help="시리얼 통신 Baudrate (기본 57600)")
    parser.add_argument("--camera-id", default=0, help="카메라 인덱스 또는 스트림 주소")
    parser.add_argument("--model", default="best_ncnn_model", help="YOLO 모델 경로")
    parser.add_argument("--target-alt", type=float, default=30.0, help="순항 정찰 고도 (m)")
    parser.add_argument("--abort-alt", type=float, default=45.0, help="복행 안전 고도 (m)")
    parser.add_argument("--mock-vision", action="store_true", help="카메라 없이 모의 비전으로 실행")
    parser.add_argument("--test-mode", action="store_true", help="벤치 테스트 모드")
    args = parser.parse_args()

    vision_state = VisionStateHolder()

    # 1. 비전 스레드 백그라운드 시작
    vision_thread = threading.Thread(
        target=run_vision_worker,
        kwargs={
            "state_holder": vision_state,
            "camera_id": args.camera_id,
            "model_path": args.model,
            "conf_threshold": 0.5,
            "mock_mode": args.mock_vision
        },
        daemon=True
    )
    vision_thread.start()

    # 2. 비행 제어기 연결 및 루프 가동
    controller = RealFlightController(
        connection_str=args.connection,
        baud=args.baud,
        target_alt_m=args.target_alt,
        abort_alt_m=args.abort_alt
    )

    if not args.test_mode:
        if not controller.connect():
            print("[!] MAVLink 연결에 실패하여 러너를 종료합니다.")
            vision_state.stop()
            sys.exit(1)
        controller.run_control_loop(vision_state=vision_state)
    else:
        print("[+] 벤치 테스트 모드 초기화 완료.")

if __name__ == "__main__":
    main()
