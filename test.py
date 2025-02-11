#!/usr/bin/env python3
import logging
import time
from math import radians, degrees

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# 환경 변수 URI (환경에 맞게 수정)
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

class SimpleWallFollower:
    """
    Manhattan world 가정하에서 두 가지 조건을 체크하여 상태를 전환하는 단순화된 벽 추종 로직.
    
    기본 동작:
      - FORWARD 상태:
          - 전방 센서(front_range)가 임계값(threshold_front) 이하이면, 장애물이 있으므로 RIGHT 회전 수행.
          - 좌측 센서(left_range)가 목표 거리(desired_distance)보다 너무 크면(예: desired_distance + left_loss_margin 초과)
            왼쪽 벽이 꺾였다고 판단하여 LEFT 회전 수행.
          - 그 외에는 왼쪽 센서 오차(error)를 기반으로 간단한 보정(lateral_speed)을 적용하며 전진.
      - TURN 상태:
          - TURN 상태에서는 설정한 turn_duration 동안 회전 명령을 유지한 후 FORWARD 상태로 복귀.
          - 회전 방향은 turn_direction ("LEFT" 또는 "RIGHT")에 따라 달라짐.
    """
    def __init__(self, desired_distance=0.25, forward_speed=0.2,
                 turn_rate=radians(90)/1.0, threshold_front=0.3,
                 gain=1.0, turn_duration=1.0, left_loss_margin=0.2):
        self.desired_distance = desired_distance  # 목표 벽과의 거리 (m)
        self.forward_speed = forward_speed        # 전진 속도 (m/s)
        self.turn_rate = turn_rate                # 회전 속도 (rad/s); turn_duration 동안 약 90° 회전
        self.threshold_front = threshold_front    # 전방 센서 임계값 (m)
        self.gain = gain                          # lateral 보정에 대한 비례 게인
        self.turn_duration = turn_duration        # 회전 지속 시간 (초)
        self.left_loss_margin = left_loss_margin  # 좌측 센서 값이 desired_distance보다 이 값만큼 크면 벽이 꺾인 것으로 판단

        self.state = "FORWARD"    # 초기 상태
        self.turn_direction = None  # "LEFT" 또는 "RIGHT"
        self.turn_start_time = 0.0
        self.left_turn_flag = True

    def update(self, front_range, left_range, current_time):
        if self.state == "FORWARD":
            if not self.left_turn_flag:
                if left_range < self.desired_distance + self.left_loss_margin:
                    self.left_turn_flag = True
            # 전방 장애물이 감지되면 RIGHT 회전으로 전환
            if front_range < self.threshold_front:
                self.state = "TURN_RIGHT"
                self.turn_direction = "RIGHT"
                self.turn_start_time = current_time
                return 0.0, 0.0, self.turn_rate  # 오른쪽 회전
            # 좌측 센서 값이 목표치보다 크게 측정되면(즉, 벽이 꺾여서 사라짐) LEFT 회전으로 전환
            elif left_range > self.desired_distance + self.left_loss_margin:
                if self.left_turn_flag:
                    self.state = "TURN_LEFT"
                    self.turn_direction = "LEFT"
                    self.turn_start_time = current_time
                    self.left_turn_flag = False
                    return 0.0, 0.0, -self.turn_rate
                else:
                    return self.forward_speed, 0.0, 0.0
            else:
                # 정상 전진: 좌측 오차(error)를 기반으로 lateral 보정
                error = self.desired_distance - left_range
                lateral_speed = -self.gain * error
                return self.forward_speed, lateral_speed, 0.0

        elif self.state == "TURN_RIGHT":
            if current_time - self.turn_start_time >= self.turn_duration:
                # 회전 시간이 지나면 FORWARD 상태로 복귀
                self.state = "FORWARD"
                self.turn_direction = None
                return self.forward_speed, 0.0, 0.0
            else:
                # TURN 상태에서는 선택한 회전 방향에 따라 회전 명령을 유지
                if self.turn_direction == "RIGHT":
                    return 0.0, 0.0, self.turn_rate
                
        elif self.state == "TURN_LEFT":
            if current_time - self.turn_start_time >= self.turn_duration:
                # 회전 시간이 지나면 FORWARD 상태로 복귀
                self.state = "FORWARD"
                self.turn_direction = None
                return self.forward_speed, 0.0, 0.0
            else:
                # TURN 상태에서는 선택한 회전 방향에 따라 회전 명령을 유지
                if self.turn_direction == "LEFT":
                    return 0.0, 0.0, -self.turn_rate

def main():
    cflib.crtp.init_drivers()
    logging.basicConfig(level=logging.ERROR)

    wall_follower = SimpleWallFollower()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
    lg_stab.add_variable('stabilizer.yaw', 'float')

    cf = Crazyflie(rw_cache='./cache')
    first_run = True
    with SyncCrazyflie(URI, cf=cf) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                with SyncLogger(scf, lg_stab) as logger:
                    print("Simplified Manhattan-world wall following started")
                    try:
                        while True:
                            #Check LiDAR measurment get successfully
                            if first_run:
                                if multiranger.left is None:
                                    continue
                                else:
                                    first_run = False

                            # 센서 데이터 (미터 단위)
                            if multiranger.front is None:
                                front_range = 999
                            else:
                                front_range = multiranger.front  # 전방 센서

                            if multiranger.left is None:
                                left_range = 999
                            else:
                                left_range = multiranger.left   # 좌측 센서

                            t = time.time()
                            # 상태 업데이트: 전진 명령 또는 회전 명령 결정
                            v_x, v_y, yaw_rate = wall_follower.update(front_range, left_range, t)
                        
                            # MotionCommander는 yaw_rate를 deg/s 단위로 받으므로 변환
                            # (부호는 라이브러리 이슈에 따라 조정)
                            yaw_rate_deg = degrees(yaw_rate)
                            
                            motion_commander.start_linear_motion(v_x, v_y, 0, rate_yaw=yaw_rate_deg)
                        
                            # 디버깅 출력
                            print(f"State: {wall_follower.state:7s} | front: {front_range:.2f} | left: {left_range:.2f} | v_x: {v_x:.2f} | v_y: {v_y:.2f} | yaw_rate: {yaw_rate_deg:.2f}")
                        
                            # 상단 센서(up)가 0.2m 미만이면 종료 (예: 착륙)
                            if is_close(multiranger.up):
                                print("Top sensor triggered. Stopping wall following.")
                                motion_commander.land(0.1)
                                break
                            
                            time.sleep(0.1)
                    except KeyboardInterrupt:
                        print("KeyboardInterrupt received. Exiting loop.")
            scf.cf.platform.send_arming_request(False)

if __name__ == '__main__':
    main()