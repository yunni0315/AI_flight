# 🛰️ Flight Script 배포 및 운용 가이드 (Raspberry Pi 5)

본 문서는 **라즈베리파이 5(Companion Computer)**에 탑재해야 하는 필수 파일 목록, 실전 비행 시 실행 명령어, 그리고 출발 전 자가진단(Test) 방법을 안내합니다.

---

## 📦 1. 파일 분류 및 배포 매트릭스

### 🟢 A. 라즈베리파이에 반드시 올려야 할 파일 (Onboard Deployment Files)
라즈베리파이 온보드 환경에서 실제로 실행되는 핵심 비행 제어 및 AI 추론 패키지입니다.

| 파일명 | 역할 | 비고 |
| :--- | :--- | :--- |
| **`real_flight_runner.py`** | **[실행 엔트리]** AI 비전(스레드 1) ↔ 20Hz MAVLink(스레드 2) 비동기 통합 러너 | **현장에서 실행할 유일한 메인 파일** |
| **`fsm_coordinator.py`** | **[FSM 코어]** 2단계 GPS 확정 후 장주 선회 진입, 15m 최저고도 리미트, 페일세이프 | 필수 모듈 |
| **`flight_core.py`** | **[Core 통신]** EventBus, 쿼터니언 변환, 32bit 타임스탬프, MavlinkActuator | 필수 공통 라이브러리 |
| **`fsm_strategies.py`** | **[비행 전략]** 180도 티어드롭 반전 선회(`Teardrop`), 타겟 GPS 직선 복귀(`DirectGPS`) | 필수 모듈 |
| **`ppn_flyover_trigger.py`** | **[유도 알고리즘]** 순수 비례항법(PPN) 및 말기 8m 잠금 직진 유도 | 필수 유도 모듈 |
| **`energy_managed_flyover_trigger.py`** | **[유도 알고리즘]** 롤 선회 시 양력 손실 보상 및 실속 방지 스로틀 유도 | 보조 유도 모듈 |
| **`virtual_waypoint_l1_trigger.py`** | **[유도 알고리즘]** 가상 웨이포인트 L1 비선형 유도 | 보조 유도 모듈 |
| **`altitude_hold_column_trigger.py`** | **[유도 알고리즘]** 고도 유지 및 가로 중앙 열(X) 수렴 유도 | 보조 유도 모듈 |
| **`point_tracking_trigger.py`** | **[유도 알고리즘]** 지정 좌표(X,Y) PD 추적 유도 | 보조 유도 모듈 |
| **`best_ncnn_model/`** 또는 **`best.pt`** | **[AI 가중치]** 탱크/건물 타겟 탐지용 학습 모델 | 라즈베리파이에 배치 필수 |

---

### 🧪 B. 온보드/현장 자가진단 및 테스트 파일 (Self-Test Suites)
라즈베리파이에서 코드 이상 유무와 결함 복원력을 현장에서 즉시 검증할 수 있는 테스트 스크립트입니다.

| 파일명 | 검증 내용 | 실행 목적 |
| :--- | :--- | :--- |
| **`test_fault_scenarios.py`** | 10대 극단적 결함 시나리오 자동 검증 (10/10 PASS) | AI 오탐, 화각이탈, 실속, 15m 최저고도, RC Override 등 방어 확인 |
| **`test_sitl_flight_suites.py`** | FSM 전주기 및 5대 유도 알고리즘 단위/통합 검증 (13/13 PASS) | 상태 전이 및 수학적 유도 계산 검증 |
| **`simulate_mission_lifecycle.py`** | 25초 전주기 비행 동역학 폐루프 시뮬레이션 | 이륙부터 타겟 관통까지의 궤적 검증 |

---

### 💻 C. 데스크톱 개발 / PC 시뮬레이션 전용 파일 (PC Only)
PC 환경에서 ArduPilot SITL 시뮬레이터와 연동하거나 개발용으로 사용하는 파일입니다 (라즈베리파이에서는 실행하지 않음).

| 파일명 | 설명 |
| :--- | :--- |
| **`sitl_flight_runner.py`** | PC에서 Mission Planner SITL(UDP 14552)과 연동하여 가상 비행할 때 사용하는 러너 |
| **`main_example.py`** | 기본 FSM 조립 템플릿 예제 |
| **`fsm_trigger.py`** | 레거시 어댑터 스크립트 |

---

## 🚀 2. 실전 비행 실행 명령어 (Execution Guide)

### 1단계: 온보드 자가진단 (비행장 도착 직후 라즈베리파이에서 실행)
```bash
# 1. 10대 결함 대응 자가진단 (0.01초 소요, OK 확인)
python test_fault_scenarios.py

# 2. 카메라 및 AI 비전 단독 벤치 테스트 (서보/모터 제어 없이 영상 인식 FPS 확인)
python real_flight_runner.py --test-mode
```

---

### 2단계: 실전 비행 제어 루프 가동
FC(Matek)와 UART 시리얼 케이블을 연결한 뒤 아래 명령어를 실행합니다:

```bash
# 기본 실행 (RPi5 UART /dev/ttyAMA0, 57600 baud)
python real_flight_runner.py --connection /dev/ttyAMA0 --baud 57600

# 옵션 지정 실행 (고도 30m, 복행 45m, NCNN 모델 지정)
python real_flight_runner.py --connection /dev/ttyAMA0 --baud 57600 --target-alt 30.0 --abort-alt 45.0 --model best_ncnn_model
```

---

## 🎮 3. 조종기(RC 송신기) 모드 전환 운용 수칙

```txt
┌───────────────────┬────────────────────────────────────────────────────────┐
│   조종기 비행 모드  │                     시스템 동작 상태                    │
├───────────────────┼────────────────────────────────────────────────────────┤
│  FBWA 또는 MANUAL │ • 조종사 100% 수동 조종 (이륙 및 비상 복귀 시 사용)      │
│                   │ • 온보드 러너 콘솔: "MODE: [FBWA]"                      │
│                   │ • MAVLink 제어 신호 자동 차단 (조종사와 힘겨루기 없음)    │
├───────────────────┼────────────────────────────────────────────────────────┤
│      GUIDED       │ • 자율 정찰 및 FSM 비행 제어 활성화                      │
│                   │ • 온보드 러너 콘솔: "MODE: [GUIDED (AUTO ACTIVE)]"       │
│                   │ • AI 탐지 ➔ 2단계 정밀 장주 진입 ➔ 게이트 완벽 관통       │
└───────────────────┴────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> 비상 상황 발생 시 언제든 조종기 스위치를 **`FBWA`** 또는 **`RTL`**로 내리면, 라즈베리파이가 즉시 제어권을 사람에게 넘깁니다.
