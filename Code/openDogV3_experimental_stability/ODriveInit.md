// ...existing code...

# ODrive 모터 제어 시스템 분석

## ODrive 하드웨어 구성

### 6개 ODrive 컨트롤러 구조
```
ODrive 1 (Serial1): 힙 관절
├── Axis 0: 오른쪽 앞다리 힙 (offSet10)
└── Axis 1: 오른쪽 뒷다리 힙 (offSet11)

ODrive 2 (Serial2): 오른쪽 앞다리
├── Axis 0: 무릎 관절 (offSet20)
└── Axis 1: 어깨 관절 (offSet21)

ODrive 3 (Serial3): 오른쪽 뒷다리  
├── Axis 0: 무릎 관절 (offSet30)
└── Axis 1: 어깨 관절 (offSet31)

ODrive 4 (Serial4): 힙 관절
├── Axis 0: 왼쪽 앞다리 힙 (offSet40)
└── Axis 1: 왼쪽 뒷다리 힙 (offSet41)

ODrive 5 (Serial5): 왼쪽 앞다리
├── Axis 0: 무릎 관절 (offSet50)
└── Axis 1: 어깨 관절 (offSet51)

ODrive 6 (Serial6): 왼쪽 뒷다리
├── Axis 0: 무릎 관절 (offSet60)
└── Axis 1: 어깨 관절 (offSet61)
```

## 주요 함수 분석

### 1. OdriveInit1() - 초기화 함수
```cpp
void OdriveInit1() {
    // 6개 ODrive 컨트롤러 순차 초기화
    for (int axis = 0; axis < 2; ++axis) {
        // 전류 제한 설정: 20A
        Serial[N] << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
        
        // 닫힌 루프 제어 모드로 설정
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        odrive[N].run_state(axis, requested_state, false);
    }
}
```

**특징:**
- **전류 제한**: 20A로 설정하여 모터 보호
- **속도 제한**: 주석 처리됨 (ODrive 툴에서 무한대로 설정 권장)
- **비동기 실행**: `false` 플래그로 대기하지 않음

### 2. modifyGains() - PID 게인 조정
```cpp
void modifyGains() {
    float posGainKnee = 15.0;      // 무릎 관절 위치 게인
    float posGainHips = 70.0;      // 힙 관절 위치 게인 (높음)
    float posGainShoulder = 15.0;  // 어깨 관절 위치 게인
    float velGain = 0.1;           // 속도 게인
    float integrator = 0.1;        // 적분 게인
}
```

**관절별 게인 특성:**
- **힙 관절**: 70.0 (가장 높음) - 안정성을 위한 강한 제어
- **무릎/어깨**: 15.0 - 부드러운 동작을 위한 낮은 게인
- **속도/적분**: 0.1 - 진동 방지를 위한 낮은 값

### 3. applyOffsets1() - 힙 관절 오프셋 적용
```cpp
void applyOffsets1() {
    odrive1.SetPosition(0, offSet10);  // 오른쪽 앞다리 힙
    odrive1.SetPosition(1, offSet11);  // 오른쪽 뒷다리 힙
    odrive4.SetPosition(0, offSet40);  // 왼쪽 앞다리 힙
    odrive4.SetPosition(1, offSet41);  // 왼쪽 뒷다리 힙
}
```

### 4. applyOffsets2() - 어깨/무릎 관절 오프셋 적용
```cpp
void applyOffsets2() {
    // 어깨 관절 오프셋
    odrive2.SetPosition(1, offSet21);  // 오른쪽 앞다리 어깨
    odrive3.SetPosition(1, offSet31);  // 오른쪽 뒷다리 어깨
    odrive5.SetPosition(1, offSet51);  // 왼쪽 앞다리 어깨
    odrive6.SetPosition(1, offSet61);  // 왼쪽 뒷다리 어깨
    
    // 무릎 관절 오프셋
    odrive2.SetPosition(0, offSet20);  // 오른쪽 앞다리 무릎
    odrive3.SetPosition(0, offSet30);  // 오른쪽 뒷다리 무릎
    odrive5.SetPosition(0, offSet50);  // 왼쪽 앞다리 무릎
    odrive6.SetPosition(0, offSet60);  // 왼쪽 뒷다리 무릎
}
```

### 5. driveJoints() - 관절 구동 함수
```cpp
void driveJoints(int joint, float pos) {
    // 안전 조건: 리모컨 토글 스위치 확인
    if (mydata_remote.toggleTop == 1) {
        // 모든 관절에 ±2.5 회전 제한 적용
        pos = constrain(pos, -2.5, 2.5);
        
        // 관절별 처리 + 오프셋 적용
    }
}
```

## 관절 매핑 및 방향 제어

### Joint ID 시스템
```cpp
// 무릎 관절 (20, 30, 50, 60)
joint == 20: 오른쪽 앞다리 무릎 (정방향)
joint == 30: 오른쪽 뒷다리 무릎 (역방향: pos*-1)
joint == 50: 왼쪽 앞다리 무릎 (역방향: pos*-1)  
joint == 60: 왼쪽 뒷다리 무릎 (정방향)

// 어깨 관절 (21, 31, 51, 61)
joint == 21: 오른쪽 앞다리 어깨 (역방향: pos*-1)
joint == 31: 오른쪽 뒷다리 어깨 (정방향)
joint == 51: 왼쪽 앞다리 어깨 (정방향)
joint == 61: 왼쪽 뒷다리 어깨 (역방향: pos*-1)

// 힙 관절 (10, 11, 40, 41)
joint == 10: 오른쪽 앞다리 힙 (정방향)
joint == 11: 오른쪽 뒷다리 힙 (역방향: pos*-1)
joint == 40: 왼쪽 앞다리 힙 (정방향)
joint == 41: 왼쪽 뒷다리 힙 (역방향: pos*-1)
```

### 방향 제어 패턴
```cpp
// 대각선 대칭성
정방향: 20(RF무릎), 60(LB무릎), 31(RB어깨), 51(LF어깨), 10(RF힙), 40(LF힙)
역방향: 30(RB무릎), 50(LF무릎), 21(RF어깨), 61(LB어깨), 11(RB힙), 41(LB힙)
```

## 안전 시스템

### 1. 모션 제한
```cpp
pos = constrain(pos, -2.5, 2.5);  // ±2.5 회전 제한
```

### 2. 리모컨 안전 스위치
```cpp
if (mydata_remote.toggleTop == 1) {  // 활성화 상태에서만 동작
    // 모터 제어 실행
}
```

### 3. 전류 제한
```cpp
Serial[N] << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
```

## ODrive 통신 프로토콜

### 명령어 형식
```cpp
// 설정 명령어
"w axis[N].controller.config.pos_gain [value]"
"w axis[N].controller.config.vel_gain [value]"  
"w axis[N].controller.config.vel_integrator_gain [value]"
"w axis[N].motor.config.current_lim [value]"

// 위치 제어
odrive[N].SetPosition(axis, position);
odrive[N].run_state(axis, state, wait_flag);
```

### 시리얼 포트 할당
```cpp
Serial1 ↔ ODrive1 (힙 관절 1,4)
Serial2 ↔ ODrive2 (오른쪽 앞다리)
Serial3 ↔ ODrive3 (오른쪽 뒷다리)
Serial4 ↔ ODrive4 (힙 관절 2,3)
Serial5 ↔ ODrive5 (왼쪽 앞다리)
Serial6 ↔ ODrive6 (왼쪽 뒷다리)
```

## 초기화 시퀀스

### 표준 시작 순서
1. **OdriveInit1()**: 모든 ODrive를 닫힌 루프 모드로 초기화
2. **applyOffsets1()**: 힙 관절 기본 위치 설정
3. **applyOffsets2()**: 어깨/무릎 관절 기본 위치 설정  
4. **modifyGains()**: PID 게인 최적화 (선택적)
5. **driveJoints()**: 실시간 관절 제어 시작

## 특징 요약

### 1. 모듈화된 설계
- 각 ODrive와 관절별 독립적인 제어
- 재사용 가능한 함수 구조

### 2. 안전 우선 설계  
- 다중 안전 장치 (제한, 스위치, 전류)
- 점진적 초기화 과정

### 3. 대칭성 기반 제어
- 좌우 대칭을 위한 방향 반전
- 일관된 관절 ID 시스템

### 4. 실시간 제어 최적화
- 비동기 명령 실행
- 낮은 지연시간을 위한 직접 통신

// ...existing code...