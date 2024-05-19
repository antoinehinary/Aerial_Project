# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

from agent import Agent
from simple_pid import PID
import numpy as np
import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E707')
HEIGHT_COEFF = 100
start_search = False
logging.basicConfig(level=logging.ERROR)

def is_close(range):
    MIN_DISTANCE = 200  # mm
    return range is not None and range < MIN_DISTANCE

class LoggingExample:
    def __init__(self, link_uri):
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        print('Connecting to %s' % link_uri)
        self._cf.open_link(link_uri)
        self.is_connected = True
        self.sensor_data = {}

    def _connected(self, link_uri):
        print('Connected to %s' % link_uri)
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('range.front')
        self._lg_stab.add_variable('range.back')
        self._lg_stab.add_variable('range.left')
        self._lg_stab.add_variable('range.right')
        self._lg_stab.add_variable('range.up')
        try:
            self._cf.log.add_config(self._lg_stab)
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _stab_log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        for name, value in data.items():
            self.sensor_data[name] = value

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

def snake_creation():
    goal_list = []
    for i in range(10, 30, 3):
        if i % 2 == 1:
            for j in range(2, 28, 3):
                goal_list.append((i, j))
        if i % 2 == 0:
            for j in range(29, 2, -3):
                goal_list.append((i, j))
    return goal_list

class DroneStateMachine:
    def __init__(self, cf, le):
        self.cf = cf
        self.le = le
        self.state = "INIT"
        self.start_search = False
        self.starting_edge = False
        self.ending_edge = False
        self.height_vect = []
        self.avg_height
        self.i = 0
        self.x_landing_center = 0.0
        self.y_landing_center = 0.0
        self.landing_pad_mid_length = 0.05

    def update(self):
        sensor_data = self.le.sensor_data
        z = sensor_data["stateEstimate.z"]

        self.height_vect.append(z * HEIGHT_COEFF)

        if len(self.height_vect) > 8:
            self.height_vect.pop(0)
            for l in range(len(self.height_vect)):
                if l < 4:
                    self.height_vect[l] = 0.3*self.height_vect[l]
                else:
                    self.height_vect[l] = 0.7*self.height_vect[l]
            
            self.avg_height = np.mean(self.height_vect)

        if self.state == "INIT":
            if z > 0.2:
                self.state = "MOVE_FORWARD"
            else:
                self.cf.commander.send_hover_setpoint(0, 0, 0, z + 0.02)

        elif self.state == "MOVE_FORWARD":
            self.cf.commander.send_hover_setpoint(0.2, 0, 0, z)

            if (self.avg_height > 12) and not self.starting_edge and le.sensor_data["stateEstimate.x"] > 1:
                self.state = "EDGE_DETECTED"
                self.starting_edge = True
                self.stop_cycles = 20

        elif self.state == "EDGE_DETECTED":
            if self.stop_cycles > 0:
                self.cf.commander.send_hover_setpoint(0, 0, 0, z)
                self.stop_cycles -= 1
            else:
                self.cf.commander.send_position_setpoint(sensor_data["stateEstimate.x"] + 0.15, sensor_data["stateEstimate.y"], z, 0)
                self.state = "SEARCH_FALLING_EDGE"

        elif self.state == "SEARCH_FALLING_EDGE":
            self.cf.commander.send_hover_setpoint(0, -0.1, 0, z)
            if (self.avg_height > 12) and not self.starting_edge and le.sensor_data["stateEstimate.x"] > 1:
                self.state = "MOVE_TO_LAND_POSITION"

        elif self.state == "MOVE_TO_LAND_POSITION":
            self.cf.commander.send_position_setpoint(sensor_data["stateEstimate.x"], sensor_data["stateEstimate.y"] - 0.15, z)
            if abs(sensor_data["stateEstimate.y"] - (self.y_landing_center - 0.15)) < 0.01:
                self.state = "LANDING"

        elif self.state == "LANDING":
            self.cf.commander.send_hover_setpoint(0, 0, 0, z - 0.02)
            if z < 0.2:
                self.state = "LANDED"

        elif self.state == "LANDED":
            self.cf.commander.send_stop_setpoint()

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    state_machine = DroneStateMachine(cf, le)

    while le.is_connected:
        if is_close(le.sensor_data["range.up"]):
            break
         
        state_machine.update()
        time.sleep(0.1)

    cf.close_link()
