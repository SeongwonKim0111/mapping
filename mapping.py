import logging
import math
import sys
import time
import threading
import numpy as np

from vispy import scene
from vispy.scene import visuals
from vispy.scene.cameras import PanZoomCamera

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# 사용자가 작성한 벽 추종 알고리즘 (예: 간단한 Manhattan-world wall following)
from MW_wall_following import MW_WallFollower, is_close

from PyQt6 import QtWidgets, QtCore
import imageio

import datetime
suffix = datetime.datetime.now().strftime('%y%m%d_%H%M%S')
fileName = suffix + '.jpg'

# 로깅 설정
logging.basicConfig(level=logging.INFO)

# Crazyflie 연결 URI 및 기타 상수
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")
SENSOR_TH = 2000  # mm 단위 센서 임계값
SPEED_FACTOR = 0.2

# 전역으로 공유할 데이터 (로그 콜백에서 업데이트하고, 캔버스에서 읽음)
global_data = {
    'timestamp' : None,
    'position': [0.0, 0.0, 0.0],
    'measurement': None,  # {'roll':..., 'pitch':..., 'yaw':..., 'front':..., 'back':..., 'up':..., 'left':..., 'right':..., 'down':...}
}

########################################################################
# 2D 맵 시각화를 위한 Canvas (PyQt + Vispy)
########################################################################
class Canvas(scene.SceneCanvas):
    def __init__(self):
        scene.SceneCanvas.__init__(self, keys=None, size=(800, 600))
        self.unfreeze()
        self.view = self.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        # 카메라 설정 (2D PanZoom)
        self.view.camera = PanZoomCamera(rect=(-5, -5, 10, 10))
        self.view.camera.set_range()

        # 마지막 위치 (2D: x,y)
        self.last_pos = [0.0, 0.0]

        # 드론 경로 및 센서측정 포인트를 저장할 배열
        self.pos_history = np.empty((0, 2))
        self.meas_history = np.empty((0, 2))

        # 시각화를 위한 마커와 센서선을 생성
        self.pos_markers = visuals.Markers()
        self.meas_markers = visuals.Markers()
        self.lines = []
        for _ in range(4):  # left, right, front, back
            line = visuals.Line(color='black')
            self.lines.append(line)
            self.view.add(line)
        self.view.add(self.pos_markers)
        self.view.add(self.meas_markers)

        # 100ms마다 화면 갱신하는 타이머
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_canvas)
        self.timer.start(100)

        self.file_path = './log/' + suffix + '_log.txt'

        self.freeze()

    def update_canvas(self):
        with open(self.file_path, "a") as f:
            # 전역 변수 global_data에서 최신 position과 measurement를 가져옴
            t_step = global_data.get('timestamp')
            pos = global_data.get('position')
            meas = global_data.get('measurement')
            if t_step is not None:
                f.write("Timestamp: " + str(t_step) + " | ")
            if pos is not None:
                f.write("X_pos: " + str(pos[0]) + " | " + "Y_pos: " + str(pos[1]) + " | ")
                # 2D 위치 (x, y) 업데이트
                self.last_pos = [pos[0], pos[1]]
                self.pos_history = np.append(self.pos_history, [[pos[0], pos[1]]], axis=0)
                self.pos_markers.set_data(self.pos_history, face_color='red', size=5)
            if meas is not None:
                for key in meas:
                    if key in ['roll', 'pitch', 'up']:
                        pass
                    elif key in ['front', 'back', 'left', 'right', 'down']:
                        if meas[key] > 4000:
                            f.write(str(key) + ": None | ")
                        else:
                            f.write(str(key) + ": " + str(meas[key] / 1000) + " | ")
                    else:
                        f.write(str(key) + ": " + str(meas[key]) + " | ")
                points = self.rotate_and_create_points(meas)
                # 센서 측정 선을 현재 위치에서 각 센서측정점으로 연결
                for i in range(4):
                    if i < len(points):
                        self.lines[i].set_data(np.array([self.last_pos, points[i]]))
                    else:
                        self.lines[i].set_data(np.array([self.last_pos, self.last_pos]))
                if points:
                    self.meas_history = np.append(self.meas_history, np.array(points), axis=0)
                    self.meas_markers.set_data(self.meas_history, face_color='blue', size=5)
            f.write('\n')
            self.update()

    def rot2d(self, yaw, origin, point):
        """yaw 각도(도)를 사용하여 origin을 기준으로 point를 회전"""
        rad = math.radians(yaw)
        cos_ = math.cos(rad)
        sin_ = math.sin(rad)
        dx = point[0] - origin[0]
        dy = point[1] - origin[1]
        x_new = origin[0] + (dx * cos_ - dy * sin_)
        y_new = origin[1] + (dx * sin_ + dy * cos_)
        return [x_new, y_new]

    def rotate_and_create_points(self, m):
        """left, right, front, back 센서 데이터를 2D 좌표로 변환"""
        data = []
        o = self.last_pos
        yaw = m['yaw']

        if m['left'] < SENSOR_TH:
            left = [o[0], o[1] + m['left'] / 1000.0]
            data.append(self.rot2d(yaw, o, left))
        if m['right'] < SENSOR_TH:
            right = [o[0], o[1] - m['right'] / 1000.0]
            data.append(self.rot2d(yaw, o, right))
        if m['front'] < SENSOR_TH:
            front = [o[0] + m['front'] / 1000.0, o[1]]
            data.append(self.rot2d(yaw, o, front))
        if m['back'] < SENSOR_TH:
            back = [o[0] - m['back'] / 1000.0, o[1]]
            data.append(self.rot2d(yaw, o, back))
        return data

    def stop_and_save(self):
        """맵을 이미지 파일(map.png)로 저장하고 캔버스를 닫음"""
        img = self.render()
        imageio.imsave('./map/' + suffix + '_map.png', img)
        print("Map saved. Stopping mapping.")
        self.close()

########################################################################
# 로그 콜백 함수 (로그 설정에서 호출)
########################################################################
def update_position_callback(timestamp, data, logconf):
    # position 데이터 (stateEstimate.x, y, z) 업데이트
    global_data['timestamp'] = timestamp
    global_data['position'] = [data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']]

def update_measurement_callback(timestamp, data, logconf):
    # 센서 데이터(roll, pitch, yaw, multiranger의 front, back, up, left, right, zrange)를 업데이트
    global_data['measurement'] = {
        'roll': data['stabilizer.roll'],
        'pitch': data['stabilizer.pitch'],
        'yaw': data['stabilizer.yaw'],
        'front': data['range.front'],
        'back': data['range.back'],
        'up': data['range.up'],
        'left': data['range.left'],
        'right': data['range.right'],
        'down': data.get('range.zrange', 999)  # 값이 없으면 999로 처리
    }

########################################################################
# 벽 추종(wall following) 동작을 수행하는 함수 (Movement 제어 포함)
########################################################################
def wall_following_loop(scf):
    wall_follower = MW_WallFollower()
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
    lg_stab.add_variable('stabilizer.yaw', 'float')

    # 드론 모터 활성화
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)

    try:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                with SyncLogger(scf, lg_stab) as logger:
                    print("Wall following started")
                    first_run = True
                    while True:
                        # 초기 실행 시 left 센서 값이 없으면 대기
                        if first_run:
                            if multiranger.left is None:
                                continue
                            else:
                                first_run = False

                        # 센서 측정 (단위: 미터)
                        front_range = multiranger.front if multiranger.front is not None else 999
                        left_range = multiranger.left if multiranger.left is not None else 999

                        t = time.time()
                        v_x, v_y, yaw_rate = wall_follower.update(front_range, left_range, t)
                        yaw_rate_deg = math.degrees(yaw_rate)
                        motion_commander.start_linear_motion(v_x, v_y, 0, rate_yaw=yaw_rate_deg)

                        print(f"State: {wall_follower.state:7s} | front: {front_range:.2f} | left: {left_range:.2f} | "
                              f"v_x: {v_x:.2f} | v_y: {v_y:.2f} | yaw_rate: {yaw_rate_deg:.2f}")

                        # 위쪽 센서(up)가 일정 거리 이하이면 착륙 후 종료
                        if is_close(multiranger.up):
                            print("Top sensor triggered. Landing.")
                            motion_commander.land(0.2)
                            break

                        time.sleep(0.1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Exiting wall following loop.")
    finally:
        scf.cf.platform.send_arming_request(False)

########################################################################
# 메인 함수: 연결, 로그 설정, wall following, 그리고 시각화 실행
########################################################################
def main():
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # [2] 로그 설정: position
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')
        scf.cf.log.add_config(lpos)
        lpos.data_received_cb.add_callback(update_position_callback)
        lpos.start()

        # [2] 로그 설정: 측정값 (multiranger 및 stabilizer)
        lmeas = LogConfig(name='Meas', period_in_ms=100)
        lmeas.add_variable('range.front')
        lmeas.add_variable('range.back')
        lmeas.add_variable('range.up')
        lmeas.add_variable('range.left')
        lmeas.add_variable('range.right')
        lmeas.add_variable('range.zrange')
        lmeas.add_variable('stabilizer.roll')
        lmeas.add_variable('stabilizer.pitch')
        lmeas.add_variable('stabilizer.yaw')
        scf.cf.log.add_config(lmeas)
        lmeas.data_received_cb.add_callback(update_measurement_callback)
        lmeas.start()

        # [3] 벽 추종 동작을 별도 쓰레드에서 실행
        wall_thread = threading.Thread(target=wall_following_loop, args=(scf,))
        wall_thread.start()
        time.sleep(3)

        # [4] Qt/Vispy를 이용해 2D 맵 실시간 시각화 (이 부분에서는 드론 제어 코드는 없음)
        app = QtWidgets.QApplication(sys.argv)
        canvas = Canvas()
        canvas.show()
        app.exec()

        # Qt 창 종료 후 벽 추종 쓰레드 종료 대기 및 맵 저장
        wall_thread.join()
        canvas.stop_and_save()

if __name__ == '__main__':
    main()