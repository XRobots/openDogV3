# OpenDog V3 Quadruped Robot - 프로그램 매뉴얼

## 목차
1. [개요](#개요)
2. [시스템 요구사항](#시스템-요구사항)
3. [하드웨어 구성](#하드웨어-구성)
4. [소프트웨어 설치](#소프트웨어-설치)
5. [설정 및 칼리브레이션](#설정-및-칼리브레이션)
6. [로봇 운영](#로봇-운영)
7. [원격 제어기](#원격-제어기)
8. [테스트 및 진단](#테스트-및-진단)
9. [문제 해결](#문제-해결)
10. [API 참조](#api-참조)

---

## 개요

OpenDog V3는 4족 보행 로봇으로, Arduino C++ 코드를 Python으로 완전 변환하여 더 나은 확장성과 유지보수성을 제공합니다. 이 시스템은 다음과 같은 주요 기능을 포함합니다:

- **실시간 역기구학 제어** - 100Hz 제어 루프
- **IMU 기반 안정성 제어** - 자이로스코프 및 가속도계 피드백
- **nRF24L01 무선 통신** - 원격 제어기와의 통신
- **모듈식 아키텍처** - 독립적인 구성 요소들
- **포괄적인 테스트 스위트** - 자동화된 테스트 및 검증

### 주요 구성 요소

- **메인 컨트롤러** (`opendog_v3_controller.py`) - 로봇의 핵심 제어 시스템
- **역기구학 모듈** (`kinematics.py`) - 다리 위치 계산
- **모터 제어** (`odrive_interface.py`) - ODrive 모터 컨트롤러 인터페이스
- **IMU 인터페이스** (`imu_interface.py`) - 관성 측정 장치 제어
- **입력 처리** (`input_processing.py`) - 사용자 입력 및 보행 패턴
- **원격 제어기** (`remote_controller.py`) - 무선 원격 제어
- **설정 관리** (`config.py`) - 시스템 매개변수 관리

---

## 시스템 요구사항

### 하드웨어 요구사항

#### 주요 컴퓨터 (로봇 온보드)
- **Raspberry Pi 4 Model B** (4GB 이상 권장)
- **microSD 카드** 64GB 이상 (Class 10)
- **전원 공급장치** 5V 3A 이상

#### 원격 제어기
- **Raspberry Pi Zero W** 또는 Pi 4
- **nRF24L01 무선 모듈**
- **20x4 LCD 디스플레이** (I2C)
- **조이스틱 및 버튼** (아날로그/디지털 입력)

#### 로봇 하드웨어
- **ODrive 모터 컨트롤러** v3.6 (2개)
- **브러시리스 DC 모터** 8개 (각 다리당 2개)
- **IMU 센서** (MPU6050 또는 호환)
- **nRF24L01 무선 모듈**
- **전원 시스템** (리튬 폴리머 배터리 권장)

### 소프트웨어 요구사항

#### 운영체제
- **Raspberry Pi OS** (Bullseye 이상)
- **Python 3.8+**

#### 필수 Python 패키지
```
numpy>=1.21.0
odrive>=0.6.0
pyserial>=3.5
RPi.GPIO>=0.7.1
smbus2>=0.4.1
spidev>=3.5
nrf24>=0.1.0
```

---

## 하드웨어 구성

### 핀 배치도

#### 메인 컨트롤러 (Raspberry Pi 4)

##### GPIO 핀 할당
```
Pin  | Function        | Description
-----|-----------------|----------------------------------
2    | SDA (I2C)      | IMU 통신
3    | SCL (I2C)      | IMU 통신
8    | CE0 (SPI)      | nRF24L01 CSN
10   | MOSI (SPI)     | nRF24L01 MOSI
9    | MISO (SPI)     | nRF24L01 MISO
11   | SCLK (SPI)     | nRF24L01 SCK
18   | nRF24 CE       | nRF24L01 CE
24   | nRF24 IRQ      | nRF24L01 IRQ (선택사항)
```

##### UART 연결
```
UART 0: ODrive 1 (GPIO 14, 15)
UART 1: ODrive 2 (GPIO 0, 1)
```

#### 원격 제어기 핀 배치

##### 디지털 입력
```
Pin  | Function       | Description
-----|----------------|----------------------------------
2    | Menu Down      | 메뉴 하향 버튼
3    | Menu Up        | 메뉴 상향 버튼
0    | Toggle 2       | 토글 버튼 2
1    | Select         | 선택 버튼
4    | Motor Enable   | 모터 활성화 스위치
5    | IMU Enable     | IMU 활성화 스위치
6    | Toggle 1       | 토글 버튼 1
7    | Walk Reverse   | 역방향 보행 스위치
```

##### 아날로그 입력 (ADC 필요)
```
Channel | Function          | Description
--------|-------------------|----------------------------------
A1      | Left Trigger      | 왼쪽 트리거
A2      | Left L/R Stick    | 왼쪽 스틱 좌우
A3      | Left F/B Stick    | 왼쪽 스틱 전후
A4      | Right L/R Stick   | 오른쪽 스틱 좌우
A5      | Right F/B Stick   | 오른쪽 스틱 전후
A6      | Right Trigger     | 오른쪽 트리거
```

### 배선도

#### 메인 시스템 배선
```
[Raspberry Pi 4] ---- UART ---- [ODrive 1]
                |                    |
                ---- UART ---- [ODrive 2]
                |
                ---- I2C ----- [IMU (MPU6050)]
                |
                ---- SPI ----- [nRF24L01]
```

#### 전원 분배
```
[Battery Pack] ---- [Power Distribution Board]
                              |
                              |-- 24V --> ODrive Controllers
                              |-- 12V --> Motor Power
                              |-- 5V  --> Raspberry Pi
                              |-- 3.3V -> Sensors
```

---

## 소프트웨어 설치

### 1. 기본 시스템 설정

#### Raspberry Pi OS 설치
```bash
# SD 카드에 Raspberry Pi OS 이미지 기록
# SSH 및 I2C, SPI 활성화
sudo raspi-config
```

#### 시스템 업데이트
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-pip git
```

### 2. Python 환경 설정

#### 가상환경 생성
```bash
cd /home/pi
python3 -m venv opendog_env
source opendog_env/bin/activate
```

#### 필수 패키지 설치
```bash
pip install numpy
pip install odrive
pip install pyserial
pip install RPi.GPIO
pip install smbus2
pip install spidev
```

#### nRF24L01 라이브러리 설치
```bash
# CircuitPython nRF24L01 라이브러리
pip install circuitpython-nrf24l01
```

### 3. 프로젝트 다운로드

```bash
git clone <repository-url> opendog_v3
cd opendog_v3
```

### 4. 권한 설정

```bash
# GPIO 접근 권한
sudo usermod -a -G gpio pi
sudo usermod -a -G spi pi
sudo usermod -a -G i2c pi

# 시리얼 포트 권한
sudo usermod -a -G dialout pi
```

### 5. 시스템 서비스 설정 (선택사항)

#### systemd 서비스 생성
```bash
sudo nano /etc/systemd/system/opendog.service
```

```ini
[Unit]
Description=OpenDog V3 Controller
After=multi-user.target

[Service]
Type=idle
User=pi
ExecStart=/home/pi/opendog_env/bin/python /home/pi/opendog_v3/Code/openDogV3_experimental_stability/opendog_v3_controller.py
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable opendog.service
```

---

## 설정 및 칼리브레이션

### 1. 하드웨어 검증

#### 시스템 진단 실행
```bash
cd Code/openDogV3_experimental_stability
python3 hardware_setup.py
```

이 스크립트는 다음을 확인합니다:
- ODrive 컨트롤러 연결
- IMU 센서 통신
- nRF24L01 무선 모듈
- GPIO 핀 상태
- 시스템 리소스

#### 예상 출력
```
=== OpenDog V3 Hardware Setup ===
✓ ODrive 1 detected on /dev/ttyACM0
✓ ODrive 2 detected on /dev/ttyACM1
✓ IMU detected on I2C address 0x68
✓ nRF24L01 radio initialized
✓ GPIO pins configured
✓ System resources OK

Hardware setup complete!
```

### 2. ODrive 모터 컨트롤러 설정

#### 자동 설정
```bash
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.configure_odrives()
"
```

#### 수동 설정 (필요시)
```bash
# ODrive 구성 도구 실행
odrivetool
```

ODrivetool에서 실행할 명령:
```python
# 기본 설정
odrv0.axis0.motor.config.pole_pairs = 14
odrv0.axis0.motor.config.resistance_calib_max_voltage = 2
odrv0.axis0.motor.config.requested_current_range = 25
odrv0.axis0.motor.config.current_control_bandwidth = 100

# 엔코더 설정
odrv0.axis0.encoder.config.cpr = 8192

# 컨트롤러 설정
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.1

# 설정 저장
odrv0.save_configuration()
odrv0.reboot()
```

### 3. IMU 칼리브레이션

#### 자동 칼리브레이션
```bash
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.calibrate_imu()
"
```

#### 칼리브레이션 과정
1. **로봇을 평평한 표면에 배치**
2. **정적 칼리브레이션** (5초간 정지)
3. **오프셋 계산 및 저장**
4. **검증 테스트**

### 4. 무선 통신 설정

#### nRF24L01 설정 확인
```bash
python3 -c "
from remote_controller import RadioInterface
radio = RadioInterface(simulation_mode=False)
print('Radio initialized successfully')
"
```

### 5. 시스템 구성 파일 편집

#### config.py 파일 확인 및 수정
```bash
nano config.py
```

주요 설정 항목:
```python
# 하드웨어 구성
ODRIVE_SERIAL_NUMBERS = ['serial1', 'serial2']
IMU_I2C_ADDRESS = 0x68
RADIO_CHANNEL = 96

# 제어 매개변수
CONTROL_FREQUENCY = 100  # Hz
SAFETY_TIMEOUT = 1.0     # seconds

# 기구학 매개변수
LEG_LENGTH_1 = 110  # mm
LEG_LENGTH_2 = 120  # mm
BODY_WIDTH = 160    # mm
BODY_LENGTH = 280   # mm

# 모터 제한
MAX_MOTOR_CURRENT = 20  # A
MAX_MOTOR_VELOCITY = 50 # rad/s
```

---

## 로봇 운영

### 1. 기본 시작 절차

#### 시스템 시작
```bash
# 가상환경 활성화
source /home/pi/opendog_env/bin/activate

# 메인 디렉토리로 이동
cd /home/pi/opendog_v3/Code/openDogV3_experimental_stability

# 로봇 컨트롤러 시작
python3 opendog_v3_controller.py
```

#### 예상 시작 로그
```
2025-05-30 10:00:00 - OpenDogController - INFO - Initializing OpenDog V3 Controller
2025-05-30 10:00:01 - ODriveInterface - INFO - ODrive 1 connected on /dev/ttyACM0
2025-05-30 10:00:01 - ODriveInterface - INFO - ODrive 2 connected on /dev/ttyACM1
2025-05-30 10:00:02 - IMUInterface - INFO - IMU initialized successfully
2025-05-30 10:00:02 - RadioInterface - INFO - nRF24L01 radio initialized
2025-05-30 10:00:03 - OpenDogController - INFO - Control loop started at 100Hz
2025-05-30 10:00:03 - OpenDogController - INFO - Robot ready for operation
```

### 2. 운영 모드

#### 안전 체크리스트
- [ ] 모든 하드웨어 연결 확인
- [ ] 배터리 전압 확인 (>22V)
- [ ] 작업 공간 확보 (최소 2m x 2m)
- [ ] 원격 제어기 연결 확인
- [ ] 비상 정지 버튼 준비

#### 시작 시퀀스
1. **시스템 부팅** - 모든 구성 요소 초기화
2. **하드웨어 검증** - 센서 및 액추에이터 체크
3. **홈 포지션** - 초기 자세로 이동
4. **대기 모드** - 원격 제어 입력 대기

### 3. 제어 모드

#### 모드 1: 수동 제어
- **목적**: 직접적인 다리 위치 제어
- **입력**: 조이스틱을 통한 개별 다리 제어
- **용도**: 테스트, 칼리브레이션, 정밀 제어

#### 모드 2: 보행 제어
- **목적**: 자동 보행 패턴
- **입력**: 방향 및 속도 명령
- **패턴**: 트롯, 워크, 바운드

#### 모드 3: 자세 제어
- **목적**: 정적 자세 유지
- **입력**: 몸체 위치 및 자세 명령
- **기능**: 롤, 피치, 요 제어

#### 모드 4: 균형 제어
- **목적**: 동적 균형 유지
- **센서**: IMU 피드백
- **알고리즘**: PID 제어

### 4. 보행 패턴

#### 트롯 보행 (Trot)
```
다리 순서: FL+RR, FR+RL
주기: 0.8초
최대 속도: 0.5 m/s
안정성: 높음
```

#### 워크 보행 (Walk)
```
다리 순서: FL, RR, FR, RL
주기: 1.2초
최대 속도: 0.3 m/s
안정성: 최고
```

#### 바운드 보행 (Bound)
```
다리 순서: FL+FR, RL+RR
주기: 0.6초
최대 속도: 1.0 m/s
안정성: 낮음
```

---

## 원격 제어기

### 1. 원격 제어기 시작

#### 하드웨어 연결 확인
```bash
# I2C 장치 스캔 (LCD)
i2cdetect -y 1

# GPIO 핀 상태 확인
gpio readall
```

#### 제어기 실행
```bash
cd /home/pi/opendog_v3/Code/openDogV3_experimental_stability
python3 remote_controller.py
```

### 2. 제어기 인터페이스

#### LCD 디스플레이 (20x4)
```
Line 0: "Remote Control v3.0 "
Line 1: "Mode: XX  Count: XXXX"
Line 2: "RFB:XXXX RLR:XXXX"
Line 3: "LFB:XXXX LLR:XXXX"
```

#### 버튼 레이아웃
```
[Menu Up]    [Select]     [Menu Down]
   (3)         (1)           (2)

[Toggle 1]   [Toggle 2]
    (6)         (0)

스위치:
[Motor En]  [IMU En]  [Toggle1]  [Reverse]
   (4)        (5)       (6)        (7)
```

#### 조이스틱 매핑
```
왼쪽 스틱:
- X축 (A2): 몸체 회전 (요)
- Y축 (A3): 전진/후진

오른쪽 스틱:
- X축 (A4): 좌우 이동
- Y축 (A5): 몸체 높이

트리거:
- 왼쪽 (A1): 몸체 롤
- 오른쪽 (A6): 몸체 피치
```

### 3. 통신 프로토콜

#### 송신 데이터 구조 (28 바이트)
```c
struct SendData {
    int16_t menu_down;     // 메뉴 하향 버튼
    int16_t select;        // 선택 버튼
    int16_t menu_up;       // 메뉴 상향 버튼
    int16_t toggle_bottom; // IMU 활성화
    int16_t toggle_top;    // 모터 활성화
    int16_t toggle1;       // 토글 1
    int16_t toggle2;       // 토글 2
    int16_t mode;          // 제어 모드
    int16_t rlr;           // 오른쪽 스틱 L/R
    int16_t rfb;           // 오른쪽 스틱 F/B
    int16_t rt;            // 오른쪽 트리거
    int16_t llr;           // 왼쪽 스틱 L/R
    int16_t lfb;           // 왼쪽 스틱 F/B
    int16_t lt;            // 왼쪽 트리거
};
```

#### 수신 데이터 구조 (4 바이트)
```c
struct ReceiveData {
    int16_t mode;          // 현재 모드
    int16_t count;         // 패킷 카운터
};
```

### 4. 제어 명령

#### 기본 제어
- **전진/후진**: 왼쪽 스틱 Y축
- **좌우 회전**: 왼쪽 스틱 X축
- **측면 이동**: 오른쪽 스틱 X축
- **높이 제어**: 오른쪽 스틱 Y축

#### 자세 제어
- **몸체 롤**: 왼쪽 트리거
- **몸체 피치**: 오른쪽 트리거
- **몸체 요**: 좌우 이동과 결합

#### 모드 전환
- **Menu Up/Down**: 모드 선택
- **Select**: 모드 확인
- **Motor Enable**: 모터 활성화/비활성화
- **IMU Enable**: 균형 제어 활성화/비활성화

---

## 테스트 및 진단

### 1. 통합 테스트 실행

#### 전체 테스트 스위트
```bash
cd Code/openDogV3_experimental_stability
python3 test_suite.py
```

#### 테스트 카테고리
1. **단위 테스트** - 개별 모듈 기능
2. **통합 테스트** - 모듈 간 상호작용
3. **성능 테스트** - 실시간 요구사항
4. **통신 테스트** - 무선 통신 안정성
5. **안전 테스트** - 비상 정지 및 한계값

### 2. 개별 구성 요소 테스트

#### 역기구학 테스트
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_kinematics()
"
```

#### 모터 제어 테스트
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_odrive_interface()
"
```

#### IMU 테스트
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_imu_interface()
"
```

### 3. 성능 벤치마크

#### 제어 루프 타이밍
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_control_loop_timing()
"
```

예상 결과:
```
Control Loop Performance:
- Average cycle time: 9.8ms
- Maximum cycle time: 12.1ms
- Minimum cycle time: 9.2ms
- Jitter: 0.9ms
- Target: 10ms (100Hz)
Status: PASS
```

#### 역기구학 성능
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_kinematics_performance()
"
```

### 4. 진단 도구

#### 시스템 상태 모니터링
```bash
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.system_diagnostics()
"
```

출력 예시:
```
=== System Diagnostics ===
CPU Usage: 45.2%
Memory Usage: 67.8%
Temperature: 52.3°C
Disk Usage: 34.1%

ODrive Status:
- ODrive 1: Connected, Voltage: 24.1V
- ODrive 2: Connected, Voltage: 24.2V

IMU Status:
- Temperature: 28.5°C
- Gyro Bias: X:0.02, Y:-0.01, Z:0.03
- Accel Bias: X:0.15, Y:-0.08, Z:9.78

Radio Status:
- Signal Strength: -45dBm
- Packet Loss: 0.1%
- Latency: 2.3ms
```

### 5. 로그 분석

#### 로그 파일 위치
```
/var/log/opendog/
├── system.log          # 시스템 전체 로그
├── control.log         # 제어 루프 로그
├── odrive.log          # 모터 제어 로그
├── imu.log             # IMU 데이터 로그
└── communication.log   # 통신 로그
```

#### 로그 모니터링
```bash
# 실시간 로그 확인
tail -f /var/log/opendog/system.log

# 오류 로그 검색
grep "ERROR" /var/log/opendog/*.log

# 성능 분석
grep "Performance" /var/log/opendog/control.log
```

---

## 문제 해결

### 1. 일반적인 문제

#### 시스템이 시작되지 않음
**증상**: 로봇이 초기화에 실패
**원인**: 
- 하드웨어 연결 문제
- 권한 부족
- 의존성 패키지 누락

**해결방법**:
```bash
# 하드웨어 연결 확인
python3 hardware_setup.py

# 권한 확인
groups $USER

# 패키지 재설치
pip install --force-reinstall -r requirements.txt
```

#### ODrive 연결 실패
**증상**: "ODrive not found" 오류
**원인**:
- USB 연결 불량
- 시리얼 포트 권한
- ODrive 펌웨어 문제

**해결방법**:
```bash
# 시리얼 포트 확인
ls -la /dev/ttyACM*

# 권한 추가
sudo usermod -a -G dialout $USER

# ODrive 재부팅
python3 -c "
import odrive
odrv = odrive.find_any()
odrv.reboot()
"
```

#### IMU 읽기 실패
**증상**: IMU 데이터가 0 또는 NaN
**원인**:
- I2C 연결 문제
- 잘못된 주소
- 센서 초기화 실패

**해결방법**:
```bash
# I2C 장치 스캔
i2cdetect -y 1

# IMU 재초기화
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.calibrate_imu()
"
```

#### 무선 통신 끊김
**증상**: 원격 제어 응답 없음
**원인**:
- 안테나 문제
- 간섭
- 전원 부족

**해결방법**:
```bash
# 신호 강도 확인
python3 -c "
from remote_controller import RadioInterface
radio = RadioInterface()
# 신호 테스트 코드
"

# 채널 변경
# config.py에서 RADIO_CHANNEL 수정
```

### 2. 성능 문제

#### 제어 루프 지연
**증상**: 제어 주기가 10ms를 초과
**원인**:
- CPU 부하 과다
- 메모리 부족
- 비효율적인 코드

**해결방법**:
```bash
# CPU 사용률 확인
htop

# 메모리 사용률 확인
free -h

# 프로세스 우선순위 조정
sudo renice -10 $(pgrep python3)
```

#### 불안정한 보행
**증상**: 로봇이 넘어지거나 떨림
**원인**:
- 칼리브레이션 부정확
- PID 게인 설정
- 기계적 문제

**해결방법**:
```bash
# IMU 재칼리브레이션
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.calibrate_imu()
"

# PID 게인 조정 (config.py)
BALANCE_PID_GAINS = {
    'kp': 1.0,  # 비례 게인 감소
    'ki': 0.1,  # 적분 게인 감소
    'kd': 0.05  # 미분 게인 조정
}
```

### 3. 하드웨어 문제

#### 모터 과열
**증상**: 모터가 뜨거워지고 성능 저하
**원인**:
- 과전류
- 부적절한 냉각
- 기계적 부하

**해결방법**:
```bash
# 전류 제한 확인
python3 -c "
import odrive
odrv = odrive.find_any()
print(f'Current limit: {odrv.axis0.motor.config.current_lim}')
"

# 전류 제한 감소
odrv.axis0.motor.config.current_lim = 15  # 20에서 15로 감소
```

#### 배터리 수명 단축
**증상**: 작동 시간이 예상보다 짧음
**원인**:
- 과소비
- 배터리 노화
- 비효율적인 제어

**해결방법**:
- 절전 모드 구현
- 모터 효율성 최적화
- 배터리 상태 모니터링

### 4. 소프트웨어 문제

#### 메모리 누수
**증상**: 시간이 지나면서 메모리 사용량 증가
**원인**:
- 객체 정리 부족
- 순환 참조
- 대용량 데이터 누적

**해결방법**:
```bash
# 메모리 프로파일링
python3 -m memory_profiler opendog_v3_controller.py

# 가비지 컬렉션 강제 실행
python3 -c "
import gc
gc.collect()
print(f'Objects: {len(gc.get_objects())}')
"
```

#### 예외 처리
**증상**: 예상치 못한 종료
**원인**:
- 처리되지 않은 예외
- 리소스 정리 실패

**해결방법**:
```python
# 로깅 레벨 증가
logging.basicConfig(level=logging.DEBUG)

# 예외 추적 활성화
import traceback
try:
    # 코드 실행
    pass
except Exception as e:
    logging.error(f"Error: {e}")
    traceback.print_exc()
```

---

## API 참조

### 1. 메인 컨트롤러 API

#### OpenDogController 클래스

```python
class OpenDogController:
    def __init__(self, config_file: str = "config.py"):
        """
        OpenDog V3 메인 컨트롤러 초기화
        
        Args:
            config_file: 설정 파일 경로
        """
    
    def start(self) -> None:
        """컨트롤러 시작"""
    
    def stop(self) -> None:
        """컨트롤러 정지"""
    
    def set_mode(self, mode: int) -> None:
        """
        제어 모드 설정
        
        Args:
            mode: 제어 모드 (0: 수동, 1: 보행, 2: 자세, 3: 균형)
        """
    
    def get_status(self) -> Dict[str, Any]:
        """현재 상태 반환"""
    
    def emergency_stop(self) -> None:
        """비상 정지"""
```

### 2. 역기구학 API

#### Kinematics 클래스

```python
class Kinematics:
    def inverse_kinematics(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """
        역기구학 계산
        
        Args:
            x, y, z: 목표 위치 (mm)
            
        Returns:
            (theta1, theta2): 관절 각도 (라디안)
        """
    
    def forward_kinematics(self, theta1: float, theta2: float) -> Tuple[float, float, float]:
        """
        순기구학 계산
        
        Args:
            theta1, theta2: 관절 각도 (라디안)
            
        Returns:
            (x, y, z): 발끝 위치 (mm)
        """
    
    def calculate_body_ik(self, body_pos: List[float], body_rot: List[float]) -> List[List[float]]:
        """
        몸체 역기구학 계산
        
        Args:
            body_pos: [x, y, z] 몸체 위치
            body_rot: [roll, pitch, yaw] 몸체 회전
            
        Returns:
            4개 다리의 발끝 위치 리스트
        """
```

### 3. 모터 제어 API

#### ODriveInterface 클래스

```python
class ODriveInterface:
    def __init__(self):
        """ODrive 인터페이스 초기화"""
    
    def connect(self) -> bool:
        """ODrive 연결"""
    
    def set_position(self, axis: int, position: float) -> None:
        """
        위치 제어
        
        Args:
            axis: 축 번호 (0-7)
            position: 목표 위치 (라디안)
        """
    
    def set_velocity(self, axis: int, velocity: float) -> None:
        """
        속도 제어
        
        Args:
            axis: 축 번호 (0-7)
            velocity: 목표 속도 (rad/s)
        """
    
    def get_position(self, axis: int) -> float:
        """현재 위치 반환 (라디안)"""
    
    def get_velocity(self, axis: int) -> float:
        """현재 속도 반환 (rad/s)"""
    
    def enable_motor(self, axis: int) -> None:
        """모터 활성화"""
    
    def disable_motor(self, axis: int) -> None:
        """모터 비활성화"""
```

### 4. IMU API

#### IMUInterface 클래스

```python
class IMUInterface:
    def __init__(self):
        """IMU 인터페이스 초기화"""
    
    def read_gyro(self) -> Tuple[float, float, float]:
        """자이로스코프 읽기 (rad/s)"""
    
    def read_accel(self) -> Tuple[float, float, float]:
        """가속도계 읽기 (m/s²)"""
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """현재 자세 반환 (roll, pitch, yaw)"""
    
    def calibrate(self) -> bool:
        """IMU 칼리브레이션"""
    
    def set_offsets(self, gyro_offsets: List[float], accel_offsets: List[float]) -> None:
        """오프셋 설정"""
```

### 5. 원격 제어 API

#### RemoteController 클래스

```python
class RemoteController:
    def __init__(self, simulation_mode: bool = False):
        """원격 제어기 초기화"""
    
    def start(self) -> None:
        """제어기 시작"""
    
    def stop(self) -> None:
        """제어기 정지"""
    
    def read_inputs(self) -> bool:
        """입력 읽기"""
    
    def transmit_data(self) -> bool:
        """데이터 전송"""
    
    def receive_data(self) -> bool:
        """데이터 수신"""
    
    def get_status(self) -> Dict[str, Any]:
        """상태 반환"""
```

### 6. 설정 관리 API

#### ConfigManager 클래스

```python
class ConfigManager:
    def load_config(self, file_path: str) -> Dict[str, Any]:
        """설정 파일 로드"""
    
    def save_config(self, config: Dict[str, Any], file_path: str) -> None:
        """설정 파일 저장"""
    
    def get_parameter(self, key: str) -> Any:
        """매개변수 반환"""
    
    def set_parameter(self, key: str, value: Any) -> None:
        """매개변수 설정"""
    
    def validate_config(self) -> bool:
        """설정 유효성 검사"""
```

### 7. 테스트 API

#### TestSuite 클래스

```python
class TestSuite:
    def run_all_tests(self) -> Dict[str, bool]:
        """모든 테스트 실행"""
    
    def test_kinematics(self) -> bool:
        """역기구학 테스트"""
    
    def test_odrive_interface(self) -> bool:
        """ODrive 인터페이스 테스트"""
    
    def test_imu_interface(self) -> bool:
        """IMU 인터페이스 테스트"""
    
    def test_communication(self) -> bool:
        """통신 테스트"""
    
    def test_integration(self) -> bool:
        """통합 테스트"""
    
    def performance_benchmark(self) -> Dict[str, float]:
        """성능 벤치마크"""
```

---

## 부록

### A. 하드웨어 사양

#### ODrive v3.6 사양
- **전압**: 12-56V
- **전류**: 최대 60A 연속
- **제어 방식**: FOC (Field Oriented Control)
- **인터페이스**: USB, UART, CAN
- **엔코더**: 증분형 엔코더 지원

#### MPU6050 IMU 사양
- **자이로스코프**: ±250, ±500, ±1000, ±2000 °/s
- **가속도계**: ±2g, ±4g, ±8g, ±16g
- **인터페이스**: I2C
- **업데이트 주파수**: 최대 1kHz

#### nRF24L01 무선 모듈 사양
- **주파수**: 2.4GHz ISM 대역
- **데이터 전송률**: 250kbps, 1Mbps, 2Mbps
- **전송 거리**: 최대 100m (개방 공간)
- **인터페이스**: SPI

### B. 전기적 특성

#### 전원 요구사항
- **메인 배터리**: 24V 리튬 폴리머 (6S)
- **용량**: 최소 5000mAh 권장
- **방전률**: 20C 이상
- **보호 회로**: BMS 필수

#### 전류 소비
- **Raspberry Pi 4**: 0.6-1.2A @ 5V
- **ODrive (대기)**: 0.1A @ 24V
- **ODrive (동작)**: 1-5A @ 24V (부하에 따라)
- **모터 (각각)**: 1-20A @ 24V (부하에 따라)

### C. 기계적 사양

#### 로봇 치수
- **길이**: 400mm
- **폭**: 200mm  
- **높이**: 150-300mm (가변)
- **무게**: 3-5kg (배터리 포함)

#### 다리 사양
- **상완 길이**: 110mm
- **하완 길이**: 120mm
- **작업 반경**: 80-180mm
- **최대 하중**: 각 다리당 15kg

### D. 소프트웨어 의존성

#### Python 패키지 버전
```
numpy==1.21.0
odrive==0.6.7
pyserial==3.5
RPi.GPIO==0.7.1
smbus2==0.4.1
spidev==3.5
circuitpython-nrf24l01==1.2.3
```

#### 시스템 라이브러리
```
libatlas-base-dev
libopenblas-dev
python3-dev
python3-pip
i2c-tools
```

### E. 안전 지침

#### 일반 안전
- 작업 전 전원 차단
- 적절한 보호 장비 착용
- 충분한 작업 공간 확보
- 비상 정지 버튼 준비

#### 전기 안전
- 배터리 취급 주의
- 단락 방지
- 적절한 퓨즈 사용
- 절연 확인

#### 기계 안전
- 움직이는 부품 주의
- 적절한 토크 설정
- 정기적인 점검
- 마모 부품 교체

---

## 연락처 및 지원

### 기술 지원
- **GitHub Repository**: [링크]
- **문서**: [링크]
- **이슈 트래커**: [링크]

### 커뮤니티
- **Discord**: [링크]
- **Reddit**: [링크]
- **YouTube**: [링크]

### 기여하기
- **코드 기여**: Pull Request 환영
- **버그 리포트**: Issue 생성
- **문서 개선**: 제안 환영

---

*이 매뉴얼은 OpenDog V3 프로젝트의 일부입니다. 최신 버전은 공식 저장소에서 확인하세요.*

**버전**: 1.0
**최종 업데이트**: 2025년 5월 30일
**라이선스**: MIT License
