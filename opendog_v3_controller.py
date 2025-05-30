"""
openDogV3 Experimental Stability Control System
4족 보행 로봇의 실시간 제어 시스템

이 모듈은 openDogV3 로봇의 핵심 제어 로직을 Python으로 구현합니다.
Arduino C++ 코드에서 변환된 것으로, 다음 기능들을 포함합니다:

- 실시간 센서 데이터 처리 (IMU, 조이스틱)
- 지수 이동 평균 필터링
- 4족 보행 제어 알고리즘
- 무선 통신 시스템
- 안전성 모니터링
"""

import math
import time
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum


class RunMode(Enum):
    """로봇 운영 모드"""
    SETUP = 0
    DEMO = 1
    GAIT = 2
    EMERGENCY = 3


@dataclass
class RemoteData:
    """리모컨 데이터 구조"""
    menu_down: int = 0
    menu_up: int = 0
    select: int = 0
    toggle_bottom: int = 0
    toggle_top: int = 0
    toggle1: int = 0
    toggle2: int = 0
    mode: int = 0
    RLR: int = 512  # 오른쪽 조이스틱 좌우
    RFB: int = 512  # 오른쪽 조이스틱 전후
    RT: int = 0     # 오른쪽 트리거
    LLR: int = 512  # 왼쪽 조이스틱 좌우
    LFB: int = 512  # 왼쪽 조이스틱 전후
    LT: int = 0     # 왼쪽 트리거


@dataclass
class LegPosition:
    """다리 위치 좌표"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class SignalFilter:
    """지수 이동 평균 필터 클래스"""
    
    def __init__(self):
        self.filtered_values = {}
    
    def filter(self, input_val: float, prev_filtered: float, filter_strength: int) -> float:
        """
        지수 이동 평균 필터 적용
        
        Args:
            input_val: 입력값
            prev_filtered: 이전 필터링된 값
            filter_strength: 필터 강도 (높을수록 더 부드러움)
        
        Returns:
            필터링된 값
        """
        if filter_strength == 0:
            return input_val
        
        alpha = 1.0 / (filter_strength + 1)
        return alpha * input_val + (1 - alpha) * prev_filtered
    
    def apply_deadzone(self, value: int, center: int = 512, deadzone: int = 50) -> int:
        """
        데드존 적용
        
        Args:
            value: 입력값
            center: 중심값
            deadzone: 데드존 범위
        
        Returns:
            데드존이 적용된 값
        """
        if abs(value - center) < deadzone:
            return center
        return value


class OpenDogV3Controller:
    """openDogV3 메인 제어 클래스"""
    
    def __init__(self):
        # 시스템 상태
        self.run_mode = RunMode.SETUP
        self.remote_state = 0
        self.loop_count = 0
        self.start_time = time.time()
        
        # 센서 데이터
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_yaw = 0.0
        self.battery_voltage = 24.0
        
        # 리모컨 데이터
        self.remote_data = RemoteData()
        self.filter = SignalFilter()
        
        # 필터링된 조이스틱 값
        self.rfb_filtered = 512.0
        self.rlr_filtered = 512.0
        self.rt_filtered = 0.0
        self.lfb_filtered = 512.0
        self.llr_filtered = 512.0
        self.lt_filtered = 0.0
        
        # IMU 필터링된 값
        self.leg_trans_x_filtered = 0.0
        self.leg_trans_y_filtered = 0.0
        self.leg_roll_filtered = 0.0
        self.leg_pitch_filtered = 0.0
        
        # 다리 위치
        self.leg_positions = [LegPosition() for _ in range(4)]
        
        # 보행 제어 변수
        self.timer1 = 80  # 보행 타이밍
        self.short_leg1 = 160  # 다리 높이
        self.step_height = 50  # 스텝 높이
        
        # 안전성 파라미터
        self.max_velocity = 100.0
        self.max_acceleration = 50.0
        self.safety_timeout = 5.0  # 5초
        
    def initialize(self):
        """시스템 초기화"""
        print("OpenDogV3 시스템 초기화 중...")
        
        # 센서 초기화
        self._init_imu()
        self._init_communication()
        
        # 초기 자세 설정
        self._set_home_position()
        
        print("초기화 완료")
    
    def _init_imu(self):
        """IMU 센서 초기화"""
        print("IMU 센서 초기화")
        # 실제 구현에서는 IMU 라이브러리 초기화
        pass
    
    def _init_communication(self):
        """통신 시스템 초기화"""
        print("무선 통신 초기화")
        # 실제 구현에서는 nRF24L01 초기화
        pass
    
    def _set_home_position(self):
        """홈 위치 설정"""
        for i, leg in enumerate(self.leg_positions):
            leg.x = 100.0  # 기본 x 위치
            leg.y = 0.0    # 기본 y 위치
            leg.z = -150.0 # 기본 z 위치 (지면으로부터)
    
    def update_sensors(self):
        """센서 데이터 업데이트"""
        # IMU 데이터 읽기 (시뮬레이션)
        self.imu_pitch = self._read_imu_pitch()
        self.imu_roll = self._read_imu_roll()
        self.imu_yaw = self._read_imu_yaw()
        
        # 배터리 전압 읽기
        self.battery_voltage = self._read_battery_voltage()
        
        # 리모컨 데이터 수신
        self._receive_remote_data()
    
    def _read_imu_pitch(self) -> float:
        """IMU 피치 각도 읽기"""
        # 실제 구현에서는 IMU 센서에서 데이터 읽기
        return 0.0  # 시뮬레이션 값
    
    def _read_imu_roll(self) -> float:
        """IMU 롤 각도 읽기"""
        # 실제 구현에서는 IMU 센서에서 데이터 읽기
        return 0.0  # 시뮬레이션 값
    
    def _read_imu_yaw(self) -> float:
        """IMU 요 각도 읽기"""
        # 실제 구현에서는 IMU 센서에서 데이터 읽기
        return 0.0  # 시뮬레이션 값
    
    def _read_battery_voltage(self) -> float:
        """배터리 전압 읽기"""
        # 실제 구현에서는 ADC를 통한 전압 측정
        return 24.0  # 시뮬레이션 값
    
    def _receive_remote_data(self):
        """리모컨 데이터 수신"""
        # 실제 구현에서는 nRF24L01에서 데이터 수신
        # 여기서는 시뮬레이션 데이터 생성
        pass
    
    def apply_signal_filtering(self):
        """신호 필터링 적용"""
        # 조이스틱 데드존 적용
        rfb_deadzone = self.filter.apply_deadzone(self.remote_data.RFB)
        rlr_deadzone = self.filter.apply_deadzone(self.remote_data.RLR)
        rt_deadzone = self.filter.apply_deadzone(self.remote_data.RT, center=0)
        lfb_deadzone = self.filter.apply_deadzone(self.remote_data.LFB)
        llr_deadzone = self.filter.apply_deadzone(self.remote_data.LLR)
        lt_deadzone = self.filter.apply_deadzone(self.remote_data.LT, center=0)
        
        # 운영 모드에 따른 필터 강도 조정
        if self.run_mode == RunMode.DEMO:
            joystick_filter = 40  # 데모 모드: 강한 필터링
        else:
            joystick_filter = 15  # 보행 모드: 빠른 응답
        
        # 조이스틱 필터링
        self.rfb_filtered = self.filter.filter(rfb_deadzone, self.rfb_filtered, joystick_filter)
        self.rlr_filtered = self.filter.filter(rlr_deadzone, self.rlr_filtered, joystick_filter)
        self.rt_filtered = self.filter.filter(rt_deadzone, self.rt_filtered, joystick_filter)
        self.lfb_filtered = self.filter.filter(lfb_deadzone, self.lfb_filtered, joystick_filter)
        self.llr_filtered = self.filter.filter(llr_deadzone, self.llr_filtered, joystick_filter)
        self.lt_filtered = self.filter.filter(lt_deadzone, self.lt_filtered, joystick_filter)
        
        # IMU 기반 다리 변위 계산
        leg_trans_x = (self.imu_pitch + 2.7) * 4
        leg_trans_y = (self.imu_roll - 5) * 4
        leg_roll = self.imu_roll * 0.8
        leg_pitch = self.imu_pitch * 0.8
        
        # IMU 데이터 필터링 (강한 필터링)
        self.leg_trans_x_filtered = self.filter.filter(leg_trans_x, self.leg_trans_x_filtered, 50)
        self.leg_trans_y_filtered = self.filter.filter(leg_trans_y, self.leg_trans_y_filtered, 50)
        self.leg_roll_filtered = self.filter.filter(leg_roll, self.leg_roll_filtered, 60)
        self.leg_pitch_filtered = self.filter.filter(leg_pitch, self.leg_pitch_filtered, 60)
    
    def update_control_mode(self):
        """제어 모드 업데이트"""
        # 토글 스위치 상태 확인
        if not self.remote_data.toggle_top:
            self.run_mode = RunMode.EMERGENCY
            return
        
        # 모드 스위치에 따른 운영 모드 설정
        if self.remote_data.mode == 0:
            self.run_mode = RunMode.DEMO
        elif self.remote_data.mode == 1:
            self.run_mode = RunMode.GAIT
        else:
            self.run_mode = RunMode.SETUP
    
    def execute_control_loop(self):
        """메인 제어 루프 실행"""
        if self.run_mode == RunMode.EMERGENCY:
            self._emergency_stop()
        elif self.run_mode == RunMode.DEMO:
            self._demo_mode_control()
        elif self.run_mode == RunMode.GAIT:
            self._gait_mode_control()
        else:
            self._setup_mode_control()
    
    def _emergency_stop(self):
        """비상 정지"""
        print("비상 정지 모드")
        # 모든 모터 정지
        for leg in self.leg_positions:
            leg.x = 100.0
            leg.y = 0.0
            leg.z = -150.0
    
    def _demo_mode_control(self):
        """데모 모드 제어"""
        # 역기구학 기반 직접 제어
        body_height = self._map_range(self.rt_filtered, 0, 1024, -100, 100)
        body_pitch = self._map_range(self.rfb_filtered, 0, 1024, -30, 30)
        body_roll = self._map_range(self.rlr_filtered, 0, 1024, -30, 30)
        body_yaw = self._map_range(self.lt_filtered, 0, 1024, -30, 30)
        
        # IMU 보정 적용
        body_pitch += self.leg_pitch_filtered
        body_roll += self.leg_roll_filtered
        
        # 모든 다리에 동일한 자세 적용
        for leg in self.leg_positions:
            leg.z = -150 + body_height
            # 피치, 롤, 요 회전 적용 (간단화된 버전)
            leg.x = 100.0 + body_pitch
            leg.y = 0.0 + body_roll
    
    def _gait_mode_control(self):
        """보행 모드 제어"""
        # 보행 알고리즘 실행
        self._update_gait_timing()
        self._calculate_gait_positions()
        
        # 조이스틱 입력을 속도로 변환
        forward_speed = self._map_range(self.rfb_filtered, 0, 1024, -50, 50)
        lateral_speed = self._map_range(self.rlr_filtered, 0, 1024, -50, 50)
        turn_speed = self._map_range(self.lt_filtered, 0, 1024, -30, 30)
        
        # 보행 속도 조정
        if abs(forward_speed) > 10 or abs(lateral_speed) > 10:
            self.timer1 = max(60, 120 - abs(forward_speed) - abs(lateral_speed))
        else:
            self.timer1 = 80  # 기본 보행 주기
    
    def _update_gait_timing(self):
        """보행 타이밍 업데이트"""
        self.loop_count += 1
        # 간단한 트로트 보행 패턴
        gait_phase = (self.loop_count % self.timer1) / self.timer1
        
        # 대각선 다리 쌍의 위상 제어
        self._update_leg_pair_phase(0, 2, gait_phase)          # 다리 1, 3
        self._update_leg_pair_phase(1, 3, gait_phase + 0.5)    # 다리 2, 4
    
    def _update_leg_pair_phase(self, leg1_idx: int, leg2_idx: int, phase: float):
        """다리 쌍의 위상 업데이트"""
        phase = phase % 1.0  # 0-1 범위로 정규화
        
        if phase < 0.5:  # 스윙 페이즈
            height_factor = math.sin(phase * 2 * math.pi)
            swing_height = self.step_height * height_factor
            
            self.leg_positions[leg1_idx].z = -150 + swing_height
            self.leg_positions[leg2_idx].z = -150 + swing_height
        else:  # 스탠스 페이즈
            self.leg_positions[leg1_idx].z = -150
            self.leg_positions[leg2_idx].z = -150
    
    def _calculate_gait_positions(self):
        """보행 시 다리 위치 계산"""
        # IMU 보정 적용
        for leg in self.leg_positions:
            leg.x += self.leg_trans_x_filtered
            leg.y += self.leg_trans_y_filtered
    
    def _setup_mode_control(self):
        """설정 모드 제어"""
        # 기본 자세 유지
        self._set_home_position()
    
    def _map_range(self, value: float, in_min: float, in_max: float, 
                   out_min: float, out_max: float) -> float:
        """값의 범위를 매핑"""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def monitor_system_health(self):
        """시스템 상태 모니터링"""
        # 배터리 상태 확인
        if self.battery_voltage < 22.0:
            print(f"경고: 배터리 전압 낮음 ({self.battery_voltage:.1f}V)")
            return False
        
        # 통신 상태 확인
        current_time = time.time()
        if current_time - self.start_time > self.safety_timeout:
            if self.remote_state == 0:
                print("경고: 리모컨 통신 끊김")
                return False
        
        return True
    
    def display_status(self):
        """상태 정보 출력"""
        print(f"모드: {self.run_mode.name}")
        print(f"배터리: {self.battery_voltage:.1f}V")
        print(f"IMU - 피치: {self.imu_pitch:.1f}°, 롤: {self.imu_roll:.1f}°")
        print(f"조이스틱 - RFB: {self.rfb_filtered:.0f}, RLR: {self.rlr_filtered:.0f}")
    
    def run_main_loop(self):
        """메인 실행 루프"""
        loop_start_time = time.time()
        
        try:
            # 1. 센서 데이터 업데이트
            self.update_sensors()
            
            # 2. 신호 필터링
            self.apply_signal_filtering()
            
            # 3. 제어 모드 업데이트
            self.update_control_mode()
            
            # 4. 제어 루프 실행
            self.execute_control_loop()
            
            # 5. 시스템 상태 모니터링
            if not self.monitor_system_health():
                self.run_mode = RunMode.EMERGENCY
            
            # 6. 상태 출력 (주기적)
            if self.loop_count % 100 == 0:  # 1초마다
                self.display_status()
            
        except Exception as e:
            print(f"제어 루프 오류: {e}")
            self.run_mode = RunMode.EMERGENCY
        
        # 루프 타이밍 조정 (10ms = 100Hz)
        loop_duration = time.time() - loop_start_time
        target_loop_time = 0.01  # 10ms
        
        if loop_duration < target_loop_time:
            time.sleep(target_loop_time - loop_duration)


class PowerManager:
    """전력 관리 시스템"""
    
    def __init__(self):
        self.power_mode = "NORMAL"
        self.battery_voltage = 24.0
        self.total_current = 0.0
    
    def manage_power(self, battery_voltage: float):
        """전력 관리 실행"""
        self.battery_voltage = battery_voltage
        
        if battery_voltage < 22.0:
            self.power_mode = "EMERGENCY"
            self._activate_emergency_mode()
        elif battery_voltage < 23.0:
            self.power_mode = "LOW_POWER"
            self._activate_power_saving_mode()
        elif battery_voltage > 25.0:
            self.power_mode = "HIGH_POWER"
            self._enable_high_performance_mode()
        else:
            self.power_mode = "NORMAL"
    
    def _activate_emergency_mode(self):
        """응급 전력 모드"""
        print("응급 전력 관리 모드 활성화")
        # LCD 밝기 50% 감소, IMU 업데이트 주기 증가 등
    
    def _activate_power_saving_mode(self):
        """절전 모드"""
        print("절전 모드 활성화")
        # 성능 제한으로 전력 절약
    
    def _enable_high_performance_mode(self):
        """고성능 모드"""
        print("고성능 모드 활성화")
        # 모든 시스템 최적 성능으로 동작


def main():
    """메인 함수"""
    print("OpenDogV3 Experimental Stability 시작")
    
    # 제어 시스템 초기화
    controller = OpenDogV3Controller()
    controller.initialize()
    
    # 전력 관리 시스템
    power_manager = PowerManager()
    
    try:
        # 메인 루프 실행
        while True:
            controller.run_main_loop()
            power_manager.manage_power(controller.battery_voltage)
            
    except KeyboardInterrupt:
        print("\n시스템 종료 중...")
    except Exception as e:
        print(f"시스템 오류: {e}")
    finally:
        print("OpenDogV3 시스템 종료")


if __name__ == "__main__":
    main()