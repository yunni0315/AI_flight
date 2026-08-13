import cv2
import math
from ultralytics import YOLO

# 사진 판넬의 실제 가로 폭 (3m -> 300cm 고정)
KNOWN_WIDTH = 300.0 
FOCAL_LENGTH = 600.0 

# 고정익 드론의 현재 비행 고도 기본값 (단위: cm, 예: 20m -> 2000cm)
DRONE_ALTITUDE_CM = 2000.0 

# [수정] 공유해주신 data.yaml 기준: 0번 클래스가 무조건 탱크(TANK)입니다.
TARGET_NAMES = {
    0: "TANK_PANEL"
}

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
    model = YOLO(r"C:\Users\asus\OneDrive\Desktop\AI\runs\detect\train\weights\best.pt")

    cap = cv2.VideoCapture(0) # OpenCV 웹캠 연결 완료

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    print("=== 고정익 탱크 정찰 시스템 가동 ===")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        img_h, img_w = frame.shape[:2]
        center_x, center_y = img_w // 2, img_h // 2

        # 화면 중앙 조준선 (녹색)
        cv2.line(frame, (center_x, 0), (center_x, img_h), (0, 255, 0), 1)
        cv2.line(frame, (0, center_y), (img_w, center_y), (0, 255, 0), 1)

        # 전체 영역 자동 탐지 
        results = model.track(frame, persist=True, conf=0.6, verbose=False)

        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                cls_id = int(box.cls[0].item())
                score = float(box.conf[0].item())  # AI의 현재 확신 점수(0.0 ~ 1.0) 추출

                
                # 학습된 탱크(0번)를 발견한 경우에만 진입
                if cls_id in TARGET_NAMES:
                    target_name = TARGET_NAMES[cls_id]
                    obj_id = int(box.id[0].item()) if box.id is not None else 0

                    # 바운딩 박스 가로 폭 계산
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    pixel_width = x2 - x1  
                    
                    target_cx = (x1 + x2) // 2
                    target_cy = (y1 + y2) // 2

                    # 고고도 직선거리 및 지면 수평거리 계산
                    slant_dist_m = calculate_slant_range(KNOWN_WIDTH, FOCAL_LENGTH, pixel_width) / 100.0
                    ground_dist_m = calculate_ground_distance(slant_dist_m * 100.0, DRONE_ALTITUDE_CM) / 100.0
                    
                    offset_x = target_cx - center_x
                    angle = calculate_angle(offset_x, FOCAL_LENGTH)

                    # 화면 시각화 (탱크 표적: 빨간색 표시)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.circle(frame, (target_cx, target_cy), 5, (0, 0, 255), -1)
                    cv2.line(frame, (center_x, center_y), (target_cx, target_cy), (0, 255, 255), 1)

                    # 텔레메트리 데이터 모니터 오버레이
                    # 텍스트 출력 부분에 점수(%) 추가
                    info_text = f"{target_name} ({score*100:.1f}%)"
                    telemetry_text = f"SLNT:{slant_dist_m:.1f}m | GRND:{ground_dist_m:.1f}m | ANG:{angle:.1f}deg"
                    
                    cv2.putText(frame, info_text, (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.putText(frame, telemetry_text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.imshow("Fixed-Wing Drone Recon System", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()