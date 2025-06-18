# -*- coding: utf-8 -*-
"""
드론 수직 이륙 및 착륙 제어 시스템
Qualisys 모션 캡처를 사용하여 드론의 위치를 추적하고,
고전적 제어기를 사용하여 수직 이륙 후 5초 대기 후 착륙을 수행합니다.
다운워시 효과를 고려한 제어 구현이 포함되어 있습니다.
"""
import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread

import qtm_rt
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# The name of the rigid body in QTM that represents the Crazyflie
rigid_body_name = 'cf'

# True: send position and orientation; False: send position only
send_full_pose = True

# Orientation standard deviation for noise handling
orientation_std_dev = 8.0e-3

# 제어 파라미터 (고전적 제어기용)
TARGET_HEIGHT = 1.0  # 목표 고도 (미터)
HOVER_TIME = 5.0     # 호버링 시간 (초)

# PID 제어기 파라미터
class PIDController:
    def __init__(self, kp, ki, kd, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01  # 최소 시간 간격
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative
        
        # PID 출력 계산
        output = p_term + i_term + d_term
        
        # 출력 제한
        output = max(-self.max_output, min(self.max_output, output))
        
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class DroneController:
    def __init__(self, cf):
        self.cf = cf
        self.current_position = [0.0, 0.0, 0.0]
        self.target_position = [0.0, 0.0, TARGET_HEIGHT]
        
        # PID 제어기 초기화 (다운워시를 고려한 파라미터)
        # Z축(고도) 제어용 - 다운워시로 인한 지연을 고려하여 약간 강한 게인 사용
        self.z_controller = PIDController(kp=0.8, ki=0.2, kd=0.3, max_output=0.8)
        
        # X, Y축 제어용 - 안정성을 위해 보수적인 게인 사용
        self.x_controller = PIDController(kp=0.5, ki=0.1, kd=0.2, max_output=0.3)
        self.y_controller = PIDController(kp=0.5, ki=0.1, kd=0.2, max_output=0.3)
        
        # 제어 상태
        self.control_enabled = False
        self.landing_mode = False
        
        # 다운워시 보상 파라미터
        self.downwash_compensation = 0.1  # 기본 추력 보상값
        
    def update_position(self, position):
        """QTM에서 받은 위치 정보를 업데이트"""
        self.current_position = position[:3]
        
    def set_target_position(self, x, y, z):
        """목표 위치 설정"""
        self.target_position = [x, y, z]
        
    def enable_control(self):
        """제어 활성화"""
        self.control_enabled = True
        
    def disable_control(self):
        """제어 비활성화"""
        self.control_enabled = False
        
    def set_landing_mode(self, landing=True):
        """착륙 모드 설정"""
        self.landing_mode = landing
        
    def calculate_control_output(self):
        """고전적 제어기를 사용한 제어 출력 계산"""
        if not self.control_enabled:
            return 0.0, 0.0, 0.0, 0.0
        
        # 위치 오차 계산
        x_error = self.target_position[0] - self.current_position[0]
        y_error = self.target_position[1] - self.current_position[1]
        z_error = self.target_position[2] - self.current_position[2]
        
        # PID 제어 출력 계산
        x_output = self.x_controller.update(x_error)
        y_output = self.y_controller.update(y_error)
        z_output = self.z_controller.update(z_error)
        
        # 다운워시 보상 적용
        if not self.landing_mode and self.current_position[2] < 0.5:
            # 낮은 고도에서는 다운워시 효과가 크므로 추가 보상 적용
            z_output += self.downwash_compensation
        
        # 착륙 모드에서는 부드러운 하강을 위해 출력 제한
        if self.landing_mode:
            z_output = min(z_output, -0.1)  # 최대 하강 속도 제한
            
        # 롤, 피치는 X, Y 제어에 사용
        roll = -y_output   # Y 오차는 롤로 보정
        pitch = x_output   # X 오차는 피치로 보정
        yaw = 0.0         # 요 각도는 0으로 유지
        thrust = 32767 + int(z_output * 10000)  # 기본 추력 + Z 제어 출력
        
        # 추력 제한 (안전을 위해)
        thrust = max(10000, min(60000, thrust))
        
        return roll, pitch, yaw, thrust


class QtmWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while (self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        qtm_instance = await self._discover()
        host = qtm_instance.host
        print('Connecting to QTM on ' + host)
        self.connection = await qtm_rt.connect(host)

        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip() for index, label in enumerate(xml.findall('*/Body/Name'))]

        await self.connection.stream_frames(
            components=['6D'],
            on_packet=self._on_packet)

    async def _discover(self):
        async for qtm_instance in qtm_rt.Discover('0.0.0.0'):
            return qtm_instance

    def _on_packet(self, packet):
        header, bodies = packet.get_6d()

        if bodies is None:
            return

        if self.body_name not in self.qtm_6DoF_labels:
            print('Body ' + self.body_name + ' not found.')
        else:
            index = self.qtm_6DoF_labels.index(self.body_name)
            temp_cf_pos = bodies[index]
            x = temp_cf_pos[0][0] / 1000
            y = temp_cf_pos[0][1] / 1000
            z = temp_cf_pos[0][2] / 1000

            r = temp_cf_pos[1].matrix
            rot = [
                [r[0], r[3], r[6]],
                [r[1], r[4], r[7]],
                [r[2], r[5], r[8]],
            ]

            if self.on_pose:
                # Make sure we got a position
                if math.isnan(x):
                    return

                self.on_pose([x, y, z, rot])

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()


def send_extpose_rot_matrix(cf, x, y, z, rot):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotaton matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """
    quat = Rotation.from_matrix(rot).as_quat()

    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
    else:
        cf.extpos.send_extpos(x, y, z)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_attitude_controller(cf):
    """고전적 제어기(attitude controller) 활성화"""
    cf.param.set_value('stabilizer.controller', '1')  # 1: PID controller


def vertical_takeoff_and_land(cf, drone_controller):
    """수직 이륙, 호버링, 착륙 시퀀스 실행"""
    print("드론 제어 시퀀스 시작...")
    
    # 1. 드론 시동
    print("드론 시동 중...")
    cf.platform.send_arming_request(True)
    time.sleep(2.0)
    
    # 2. 초기 위치 설정 (현재 위치를 기준으로)
    initial_x = drone_controller.current_position[0]
    initial_y = drone_controller.current_position[1]
    initial_z = drone_controller.current_position[2]
    
    print(f"초기 위치: ({initial_x:.2f}, {initial_y:.2f}, {initial_z:.2f})")
    
    # 3. 수직 이륙 단계
    print("수직 이륙 시작...")
    drone_controller.set_target_position(initial_x, initial_y, initial_z + TARGET_HEIGHT)
    drone_controller.enable_control()
    
    # 이륙 과정 모니터링 (목표 고도의 95%까지 도달할 때까지)
    takeoff_start_time = time.time()
    while time.time() - takeoff_start_time < 15.0:  # 최대 15초 대기
        current_height = drone_controller.current_position[2] - initial_z
        target_height = TARGET_HEIGHT
        
        # 제어 출력 계산 및 적용
        roll, pitch, yaw, thrust = drone_controller.calculate_control_output()
        cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        
        print(f"현재 고도: {current_height:.2f}m, 목표: {target_height:.2f}m, 추력: {thrust}")
        
        # 목표 고도의 95%에 도달하면 이륙 완료
        if current_height >= target_height * 0.95:
            print("이륙 완료!")
            break
            
        time.sleep(0.1)  # 100Hz 제어 주기
    
    # 4. 호버링 단계
    print(f"{HOVER_TIME}초 동안 호버링...")
    hover_start_time = time.time()
    
    while time.time() - hover_start_time < HOVER_TIME:
        # 제어 출력 계산 및 적용
        roll, pitch, yaw, thrust = drone_controller.calculate_control_output()
        cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        
        current_height = drone_controller.current_position[2] - initial_z
        print(f"호버링 중... 고도: {current_height:.2f}m, 추력: {thrust}")
        
        time.sleep(0.1)  # 100Hz 제어 주기
    
    # 5. 착륙 단계
    print("착륙 시작...")
    drone_controller.set_landing_mode(True)
    drone_controller.set_target_position(initial_x, initial_y, initial_z)
    
    landing_start_time = time.time()
    while time.time() - landing_start_time < 15.0:  # 최대 15초 대기
        current_height = drone_controller.current_position[2] - initial_z
        
        # 제어 출력 계산 및 적용
        roll, pitch, yaw, thrust = drone_controller.calculate_control_output()
        cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        
        print(f"착륙 중... 고도: {current_height:.2f}m, 추력: {thrust}")
        
        # 지면에 가까워지면 착륙 완료
        if current_height <= 0.1:
            print("착륙 완료!")
            break
            
        time.sleep(0.1)  # 100Hz 제어 주기
    
    # 6. 모터 정지
    print("모터 정지...")
    drone_controller.disable_control()
    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(1.0)
    
    # 7. 시동 해제
    cf.platform.send_arming_request(False)
    print("드론 제어 시퀀스 완료!")


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect to QTM
    qtm_wrapper = QtmWrapper(rigid_body_name)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        # 드론 제어기 초기화
        drone_controller = DroneController(cf)

        # Set up a callback to handle data from QTM
        def on_pose_update(pose):
            send_extpose_rot_matrix(cf, pose[0], pose[1], pose[2], pose[3])
            drone_controller.update_position(pose)
        
        qtm_wrapper.on_pose = on_pose_update

        # 시스템 설정
        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        activate_attitude_controller(cf)  # 고전적 제어기 사용
        reset_estimator(cf)
        
        print("시스템 초기화 완료. 3초 후 시작...")
        time.sleep(3.0)

        # 수직 이륙 및 착륙 시퀀스 실행
        vertical_takeoff_and_land(cf, drone_controller)

    qtm_wrapper.close()