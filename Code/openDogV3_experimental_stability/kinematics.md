# OpenDogV3 Experimental Stability - Kinematics 구조 분석

## 개요
OpenDogV3 4족 보행 로봇의 안정성 향상 버전 역기구학(Inverse Kinematics) 시스템 구조 분석

## 함수 시그니처
```cpp
void kinematics(int leg, float xIn, float yIn, float zIn, float roll, float pitch, float yawIn, int interOn, int dur)
```

## 다리 번호 체계
```
다리 1: 앞다리 왼쪽 (Front Left)
다리 2: 앞다리 오른쪽 (Front Right)  
다리 3: 뒷다리 왼쪽 (Back Left)
다리 4: 뒷다리 오른쪽 (Back Right)
```

## 물리적 파라미터
```cpp
#define shinLength 200        // 정강이 길이 (mm)
#define thighLength 200       // 허벅지 길이 (mm)
#define bodyWidth 59          // 몸체 너비의 절반 (힙 피벗 거리)
#define bodyLength 272        // 몸체 길이의 절반 (어깨 피벗 거리)
float hipOffset = 108;        // 힙 피벗에서 다리 중심까지 거리
```

## 계산 순서 (Processing Pipeline)

### 1. 보간(Interpolation) 처리
```cpp
// 보간 활성화 시 각 다리별 독립적인 보간 적용
if (interOn == 1) {
    // leg 1: interpFRZ, interpFRX, interpFRY, interpFRT
    // leg 2: interpFLZ, interpFLX, interpFLY, interpFLT  
    // leg 3: interpBLZ, interpBLX, interpBLY, interpBLT
    // leg 4: interpBRZ, interpBRX, interpBRY, interpBRT
}

// 300ms 필터 안정화 대기
if (currentMillis - previousInterpMillis >= 300) {
    interpFlag = 1;
}
```

### 2. YAW 축 변환 (회전)
```cpp
// 1) 몸체 중심으로부터 오프셋 적용
if (leg == 1) {         // front left
    y = y - (bodyWidth+hipOffset); 
    x = x - bodyLength;      
}
// ... 각 다리별 오프셋

// 2) 기존 각도 및 반지름 계산
existingAngle = atan(y/x);   
radius = y/sin(existingAngle);

// 3) 새로운 좌표 계산
demandYaw = existingAngle + yawAngle;
xx3 = radius * cos(demandYaw);
yy3 = radius * sin(demandYaw);

// 4) 오프셋 제거
```

### 3. PITCH 축 변환 (앞뒤 기울기)
```cpp
// 앞다리 처리: 부호 반전
if (leg == 1 || leg == 2) {
    pitch = pitch * -1;
    xx3 = xx3 * -1;
}

// 삼각법 계산
legDiffPitch = sin(pitchAngle) * bodyLength;
bodyDiffPitch = cos(pitchAngle) * bodyLength;
legDiffPitch = z - legDiffPitch;

// 발 위치 계산
footDisplacementPitch = ((bodyDiffPitch - bodyLength)*-1)+xx3;
footDisplacementAnglePitch = atan(footDisplacementPitch/legDiffPitch);

// 최종 좌표
zz2a = legDiffPitch/cos(footDisplacementAnglePitch);
footWholeAnglePitch = footDisplacementAnglePitch + pitchAngle;
zz2 = cos(footWholeAnglePitch) * zz2a;
xx1 = sin(footWholeAnglePitch) * zz2a;
```

### 4. ROLL 축 변환 (좌우 기울기)
```cpp
// 다리별 부호 처리
if (leg == 2 || leg == 3) {        // 오른쪽 다리들
    roll = 0-roll;
    yy3 = yy3*-1;
}
else if (leg == 1 || leg == 4) {   // 왼쪽 다리들
    roll = 0+roll;       
}

// 삼각법 계산
legDiffRoll = sin(rollAngle) * bodyWidth;
bodyDiffRoll = cos(rollAngle) * bodyWidth;
legDiffRoll = zz2 - legDiffRoll;

// 발 위치 계산
footDisplacementRoll = (((bodyDiffRoll - bodyWidth)*-1)+hipOffset)-yy3;
footDisplacementAngleRoll = atan(footDisplacementRoll/legDiffRoll);

// 최종 좌표
zz1a = legDiffRoll/cos(footDisplacementAngleRoll);
footWholeAngleRoll = footDisplacementAngleRoll + rollAngle;
zz1 = cos(footWholeAngleRoll) * zz1a;
yy1 = (sin(footWholeAngleRoll) * zz1a)-hipOffset;
```

### 5. 병진 이동 (Translation) 계산

#### Y축 (좌우 이동) - 힙 관절
```cpp
// 다리별 부호 처리
if (leg == 1 || leg == 4) {   // 왼쪽 다리들
    hipOffset = hipOffset * -1; 
    yy1 = yy1*-1;     
}

// 2단계 삼각법 계산
yy1 = yy1 + hipOffset;
hipAngle1a = atan(yy1/zz1);
hipHyp = zz1/cos(hipAngle1a);

hipAngle1b = asin(hipOffset/hipHyp);
hipAngle1 = (PI - (PI/2) - hipAngle1b) + hipAngle1a;
hipAngle1 = hipAngle1 - 1.5708;  // 기본 위치 오프셋 제거

z2 = hipOffset/tan(hipAngle1b);  // 새로운 다리 길이
```

#### X축 (전후 이동) - 어깨 관절
```cpp
shoulderAngle2 = atan(xx1/z2);
shoulderAngle2Degrees = shoulderAngle2 * (180/PI);
z3 = z2/cos(shoulderAngle2);
```

#### Z축 (상하 이동) - 무릎 관절
```cpp
z3 = constrain(z3,200,390);  // 안전 제한

// 코사인 법칙 적용
shoulderAngle1a = sq(thighLength) + sq(z3) - sq(shinLength);
shoulderAngle1b = 2 * thighLength * z3;
shoulderAngle1c = shoulderAngle1a / shoulderAngle1b;
shoulderAngle1 = acos(shoulderAngle1c);
kneeAngle = PI - (shoulderAngle1 * 2);
```

## 모터 제어 매핑

### 각도 변환
```cpp
float conversion = 0.02777777777777777777777777777778;  // 도→모터 회전수
shoulderAngleCounts = (shoulderAngle1Degrees-45) * conversion;
kneeAngleCounts = (kneeAngleDegrees-90) * conversion;
hipAngleCounts = hipAngle1Degrees * conversion;
```

### ODrive 모터 ID 매핑
```cpp
// 다리 1 (앞다리 왼쪽)
driveJoints(21, shoulderAngleCounts);  // 어깨
driveJoints(20, kneeAngleCounts);      // 무릎
driveJoints(10, hipAngleCounts);       // 힙

// 다리 2 (앞다리 오른쪽)
driveJoints(51, shoulderAngleCounts);  // 어깨
driveJoints(50, kneeAngleCounts);      // 무릎
driveJoints(40, hipAngleCounts);       // 힙

// 다리 3 (뒷다리 왼쪽)
driveJoints(61, shoulderAngleCounts);  // 어깨 (감산)
driveJoints(60, kneeAngleCounts);      // 무릎
driveJoints(41, hipAngleCounts);       // 힙

// 다리 4 (뒷다리 오른쪽)
driveJoints(31, shoulderAngleCounts);  // 어깨 (감산)
driveJoints(30, kneeAngleCounts);      // 무릎
driveJoints(11, hipAngleCounts);       // 힙
```

### 어깨 관절 계산 차이점
```cpp
// 앞다리 (1,2): 덧셈
shoulderAngleCounts = shoulderAngle1Counts + shoulderAngle2Counts;

// 뒷다리 (3,4): 뺄셈  
shoulderAngleCounts = shoulderAngle1Counts - shoulderAngle2Counts;
```

## 보간 시스템

### 다리별 보간 객체
```cpp
// 각 다리마다 X, Y, Z, Yaw(Twist)에 대한 독립적인 보간
interpFRZ, interpFRX, interpFRY, interpFRT  // 앞다리 오른쪽
interpFLZ, interpFLX, interpFLY, interpFLT  // 앞다리 왼쪽
interpBRZ, interpBRX, interpBRY, interpBRT  // 뒷다리 오른쪽
interpBLZ, interpBLX, interpBLY, interpBLT  // 뒷다리 왼쪽
```

### 보간 제어 플래그
- `interOn`: 보간 활성화/비활성화
- `interpFlag`: 필터 안정화 대기 (300ms)
- `dur`: 보간 지속 시간

## 주요 특징

### 1. 계층적 좌표 변환
- **YAW** → **PITCH** → **ROLL** → **Translation** 순서
- 각 단계의 결과가 다음 단계의 입력이 됨

### 2. 다리별 대칭 처리
- **좌/우 대칭**: 부호 반전으로 미러링
- **앞/뒤 차이**: 어깨 관절 계산에서 덧셈/뺄셈 구분

### 3. 안전 제한
```cpp
z3 = constrain(z3,200,390);  // 다리 길이 제한으로 관절 보호
```

### 4. 실시간 보간
- 부드러운 모션을 위한 독립적인 보간 시스템
- 각 축과 다리에 대한 개별 제어

### 5. 모듈화된 설계
- 역기구학 계산과 모터 제어 분리
- 재사용 가능한 `driveJoints()` 함수

## 실험적 안정성 버전의 특징
- 보간 시스템을 통한 더 부드러운 모션
- 필터 안정화 시간 추가 (300ms)
- 향상된 좌표 변환 정확도