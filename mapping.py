import logging
import math
import sys
import time

import numpy as np
from vispy import scene
from vispy.scene import visuals
from vispy.scene.cameras import PanZoomCamera

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

from wall_following import ManhattanWallFollower

from PyQt6 import QtWidgets, QtCore
import imageio

logging.basicConfig(level=logging.INFO)

# Crazyflie 연결 설정
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")
if len(sys.argv) > 1:
    URI = sys.argv[1]

# Enable plotting of Crazyflie
PLOT_CF = False
# Enable plotting of down sensor
PLOT_SENSOR_DOWN = False
# Set the sensor threshold (in mm)
SENSOR_TH = 2000
# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.2

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, URI):
        QtWidgets.QMainWindow.__init__(self)

        self.resize(700, 500)
        self.setWindowTitle('Multi-ranger point cloud')

        self.canvas = Canvas(self.updateHover)
        self.canvas.create_native()
        self.canvas.native.setParent(self)

        self.setCentralWidget(self.canvas.native)

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)

        # Connect to the Crazyflie
        self.cf.open_link(URI)

        # Arm the Crazyflie
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}

        self.hoverTimer = QtCore.QTimer()
        self.hoverTimer.timeout.connect(self.sendHoverCommand)
        self.hoverTimer.setInterval(100)
        self.hoverTimer.start()

    def sendHoverCommand(self):
        self.cf.commander.send_hover_setpoint(
            self.hover['x'], self.hover['y'], self.hover['yaw'],
            self.hover['height'])

    def updateHover(self, k, v):
        if (k != 'height'):
            self.hover[k] = v * SPEED_FACTOR
        else:
            self.hover[k] += v

    def disconnected(self, URI):
        print('Disconnected')

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        try:
            self.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self.pos_data)
            lpos.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

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

        try:
            self.cf.log.add_config(lmeas)
            lmeas.data_received_cb.add_callback(self.meas_data)
            lmeas.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Measurement log config, bad configuration.')

    def pos_data(self, timestamp, data, logconf):
        position = [
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z']
        ]
        self.canvas.set_position(position)

    def meas_data(self, timestamp, data, logconf):
        measurement = {
            'roll': data['stabilizer.roll'],
            'pitch': data['stabilizer.pitch'],
            'yaw': data['stabilizer.yaw'],
            'front': data['range.front'],
            'back': data['range.back'],
            'up': data['range.up'],
            'down': data['range.zrange'],
            'left': data['range.left'],
            'right': data['range.right']
        }
        self.canvas.set_measurement(measurement)

    def closeEvent(self, event):
        if (self.cf is not None):
            self.cf.close_link()


class Canvas(scene.SceneCanvas):
    def __init__(self, keyupdateCB):
        # Initialize the canvas without key shortcuts.
        scene.SceneCanvas.__init__(self, keys=None)
        self.size = 800, 600
        self.unfreeze()

        self.view = self.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        # Use a 2D camera for a 2D map view.
        self.view.camera = scene.PanZoomCamera()
        self.view.camera.center = (0.0,0.0)
        
        # Set the initial drone position to the center (0, 0)
        self.last_pos = [0, 0]
        self.pos_markers = visuals.Markers()
        self.meas_markers = visuals.Markers()
        self.pos_data = np.array([0, 0], ndmin=2)
        self.meas_data = np.array([0, 0], ndmin=2)
        self.lines = []

        # Add marker visuals to the view.
        self.view.add(self.pos_markers)
        self.view.add(self.meas_markers)
        # Create four sensor lines for left, right, front, and back.
        for i in range(4):
            line = visuals.Line()
            self.lines.append(line)
            self.view.add(line)

        self.keyCB = keyupdateCB
        """
        # --- Add XY Axis lines ---
        # X-axis: red line from -10 to 10 on the x-axis at y = 0.
        x_axis = visuals.Line(color='red', width=2)
        x_axis.set_data(np.array([[-10, 0], [10, 0]]))
        self.view.add(x_axis)
        # Y-axis: green line from -10 to 10 on the y-axis at x = 0.
        y_axis = visuals.Line(color='green', width=2)
        y_axis.set_data(np.array([[0, -10], [0, 10]]))
        self.view.add(y_axis)
        # --- End of XY Axis lines ---
        """
        # Optionally, add the initial drone marker at the center.
        self.set_position(self.last_pos)
        
        self.freeze()

    def on_key_press(self, event):
        if not event.native.isAutoRepeat():
            if event.native.key() == QtCore.Qt.Key.Key_Left:
                self.keyCB('y', 0.5)
            if event.native.key() == QtCore.Qt.Key.Key_Right:
                self.keyCB('y', -0.5)
            if event.native.key() == QtCore.Qt.Key.Key_Up:
                self.keyCB('x', 0.5)
            if event.native.key() == QtCore.Qt.Key.Key_Down:
                self.keyCB('x', -0.5)
            if event.native.key() == QtCore.Qt.Key.Key_A:
                self.keyCB('yaw', -70)
            if event.native.key() == QtCore.Qt.Key.Key_D:
                self.keyCB('yaw', 70)
            if event.native.key() == QtCore.Qt.Key.Key_Z:
                self.keyCB('yaw', -200)
            if event.native.key() == QtCore.Qt.Key.Key_X:
                self.keyCB('yaw', 200)
            if event.native.key() == QtCore.Qt.Key.Key_W:
                self.keyCB('height', 0.1)
            if event.native.key() == QtCore.Qt.Key.Key_S:
                self.keyCB('height', -0.1)
            if event.native.key() == QtCore.Qt.Key.Key_P:
                self.stop_and_save()

    def on_key_release(self, event):
        if not event.native.isAutoRepeat():
            if event.native.key() == QtCore.Qt.Key.Key_Left:
                self.keyCB('y', 0)
            if event.native.key() == QtCore.Qt.Key.Key_Right:
                self.keyCB('y', 0)
            if event.native.key() == QtCore.Qt.Key.Key_Up:
                self.keyCB('x', 0)
            if event.native.key() == QtCore.Qt.Key.Key_Down:
                self.keyCB('x', 0)
            if event.native.key() in (QtCore.Qt.Key.Key_A, QtCore.Qt.Key.Key_D, 
                                        QtCore.Qt.Key.Key_Z, QtCore.Qt.Key.Key_X):
                self.keyCB('yaw', 0)
            if event.native.key() in (QtCore.Qt.Key.Key_W, QtCore.Qt.Key.Key_S):
                self.keyCB('height', 0)

    def set_position(self, pos):
        # In 2D, use only the x and y coordinates.
        pos2d = [pos[0], pos[1]]
        self.last_pos = pos2d
        if PLOT_CF:
            self.pos_data = np.append(self.pos_data, [pos2d], axis=0)
            self.pos_markers.set_data(self.pos_data, face_color='red', size=5)

    def rot2d(self, yaw, origin, point):
        """
        Perform a 2D rotation of a point around a given origin using the yaw angle.
        """
        rad = math.radians(yaw)
        cos_ = math.cos(rad)
        sin_ = math.sin(rad)
        dx = point[0] - origin[0]
        dy = point[1] - origin[1]
        x_new = origin[0] + (dx * cos_ - dy * sin_)
        y_new = origin[1] + (dx * sin_ + dy * cos_)
        return [x_new, y_new]

    def rotate_and_create_points(self, m):
        """
        Convert sensor measurements into rotated 2D points using only the horizontal directions.
        """
        data = []
        o = self.last_pos
        yaw = m['yaw']

        # Only use left, right, front, and back sensors.
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

    def set_measurement(self, measurements):
        data = self.rotate_and_create_points(measurements)
        o = self.last_pos
        for i in range(4):
            if i < len(data):
                # Update each sensor line from the current position to the computed sensor point.
                self.lines[i].set_data(np.array([o, data[i]]))
            else:
                # If a sensor reading is missing, draw a degenerate line.
                self.lines[i].set_data(np.array([o, o]))

        if len(data) > 0:
            self.meas_data = np.append(self.meas_data, data, axis=0)
        self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)

    def stop_and_save(self):
        """ Stop mapping and save the full map view. """
        img = self.render()
        imageio.imsave('map.png', img)
        print("Map saved to 'map.png'. Stopping mapping.")
        self.close()




if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(URI)
    window.show()
    app.exec()
