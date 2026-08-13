import cv2
import math
from ultralytics import YOLO
# 통신용 라이브러리 (추후 Pixhawk와 시리얼 통신 시 주석 해제)
# import serial 
# from pymavlink import mavutil

KNOWN_WIDTH = 300.0 
FOCAL_LENGTH = 600.0 
DRONE_ALTITUDE_CM = 2000.0 
TARGET_NAMES = {0: "TANK_PANEL"}

# (거리 및 각도 계산 함수들은 기존과 동일하게 유지)
def calculate_slant_range(known_width, focal_length, pixel_width):
    return (known_width * focal_length) / pixel_width

def calculate_ground_distance(slant_range_cm, altitude_cm):
    if slant_range_cm > altitude_cm:
        return math.sqrt(slant_range_cm**2 - altitude_cm**2)
    return 0.0

def calculate_angle(offset_x, focal_length):
    angle_rad = math.atan(offset_x / focal_length)
    return math.degrees(angle_rad)

def main():
    # [중요] .pt 파일이 아닌, 라즈베리 파이용으로 경량화된 ncnn 폴더를 불러옵니다.
    model = YOLO("best_ncnn_model")

    # 카메라 연결 (라즈베리 파이 전용 카메라 모듈 사용 시 설정이 다를 수 있음)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    print("=== 고정익 자율 정찰 시스템 가동 (모니터 없는 Headless 모드) ===")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        img_h, img_w = frame.shape[:2]
        center_x = img_w // 2

        # conf=0.5 유지, 화면에 박스를 그릴 필요가 없으므로 연산만 빠르게 진행
        results = model.track(frame, persist=True, conf=0.5, verbose=False)

        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                cls_id = int(box.cls[0].item())
                
                if cls_id in TARGET_NAMES:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    pixel_width = x2 - x1  
                    
                    target_cx = (x1 + x2) // 2
                    
                    # 수평 거리와 편각 계산 로직
                    slant_dist_m = calculate_slant_range(KNOWN_WIDTH, FOCAL_LENGTH, pixel_width) / 100.0
                    ground_dist_m = calculate_ground_distance(slant_dist_m * 100.0, DRONE_ALTITUDE_CM) / 100.0
                    offset_x = target_cx - center_x
                    angle = calculate_angle(offset_x, FOCAL_LENGTH)

                    # 모니터 출력 대신, 텍스트(로그)로만 데이터를 뱉어냅니다.
                    print(f"[타겟 발견] 남은 수평거리: {ground_dist_m:.1f}m | 편각: {angle:.1f}도")

                    # TODO: 이 부분에서 시리얼 통신을 통해 Pixhawk(비행제어기)로 MAVLink 제어 신호를 전송합니다.
                    
        # 모니터가 없으므로 cv2.imshow() 와 cv2.waitKey() 는 모두 삭제합니다.

    cap.release()

if __name__ == "__main__":
    main()