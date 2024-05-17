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
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
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
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 200  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        self.sensor_data = {}

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
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

        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        # t = Timer(50, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            # print(f'{name}: {value:3.3f} ', end='')
            self.sensor_data[name] = value
        # print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


def snake_creation():
    goal_list = []
    for i in range(10, 30, +3):
        if i % 2 == 1:               # must be 4 to be correct
            for j in range(2, 28, +3):
                goal_list.append((i, j))
        if i % 2 == 0:
            for j in range(29, 2, -3):
                goal_list.append((i, j))
    return goal_list


if __name__ == '__main__':

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    crf = Crazyflie(rw_cache='./cache')

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # cf.commander.send_hover_setpoint(0, 0, 0, 0.6)
    # time.sleep(1)

    robot = Agent(le.sensor_data, 0.1)
    robot.update(le.sensor_data, 0.1)

    crf = Crazyflie(rw_cache='./cache')

    # z_pid = PID(2.5, 0.0, 1.0, proportional_on_measurement=True)
    # def send_velocity_world_setpoint(self, vx, vy, vz, yawrate):
    #     """
    #     Send Velocity in the world frame of reference setpoint with yawrate commands

    #     vx, vy, vz are in m/s
    #     yawrate is in degrees/s

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    height_vect = []
    diff_height_vect = []
    i = 0
    goal_idx = 0
    case = "snake_following"
    state = "INIT"
    current_position = (0, 0)

    x_init = 0
    x_end = 0
    avg = 0
    delta_pad = 0
    HALT_SPEED = 0
    RESET = 0
    height_desired = 0.2

    # Bool for landing pad search
    landing_path_found = False
    starting_edge = False
    landing_pos = False
    ending_edge = False
    kill_motor = False
    land = False

    path = None

    list_goal = snake_creation()
    print("snake creation")

    while robot.alive:

        if is_close(le.sensor_data["range.up"]):
            break

        # print(le.sensor_data["range.left"])
        # print(le.sensor_data["range.right"])
        # print(le.sensor_data["range.front"])
        # print(le.sensor_data["range.back"])
#########################################################################
        # if case is None:
        #     case = "snake_following"

        # elif(case == "snake_following"):
        #     print("snake_folowing")

        #     start = (int((le.sensor_data["x_global"]-min_x)/res_pos), int((le.sensor_data["y_global"]-min_y)/res_pos))
        #      # print("map start", occupancy_grid[start[0], start[1]])

        #     # print("goal id :", goal_idx)
        #     goal = list_goal[goal_idx]
        #     ## go to
        #     case = "go"

        # elif(case == "go"):
        #     print("go")

        #     goal_idx += 1


###################################################################

        if le.sensor_data["stateEstimate.z"] > 0.15:
            start_search = True

        if start_search == True:
            height_vect.append(le.sensor_data["stateEstimate.z"]*HEIGHT_COEFF)

            # print(le.sensor_data["stateEstimate.z"])
            if i > 5:
                height_vect.pop(0)
            else:
                i += 1
            # print("vect : ", vect)

            height_diff = height_vect[i-1] - height_vect[0]
            # print("height: ", height_diff)

            diff_height_vect.append(height_diff)

            if i > 30:
                diff_height_vect.pop(0)

            diff_sum = np.sum(diff_height_vect)
            avg = np.mean(height_vect)
            if diff_sum < 0 and starting_edge == False and le.sensor_data["stateEstimate.x"] >= 0.6:
                starting_edge  = True
                land = True
                print("AVG : ", avg)
                state = "LAND"
                x_init = le.sensor_data["stateEstimate.x"]
                print("start, X init is : ", x_init)
                print("Diff is : ", diff_sum)

            if state == "LAND" :
                current_position = (le.sensor_data["stateEstimate.x"], le.sensor_data["stateEstimate.y"])
                if not landing_path_found:
                    diff_valid = True
                    # prev_range = le.sensor_data["stateEstimate.z"]
                    vx = vy = forward_speed = HALT_SPEED
                    landing_path_found = True
                    comp_points = np.zeros(4)
                    landing_position = current_position
                    landing_path = robot.create_landing_path(current_position, height_desired)
                    path = landing_path
                    k = 0
                    print("landing path found, path generated : ", len(landing_path))

                if diff_sum > 0:
                    diff_valid = False
                elif diff_sum < 0 and not diff_valid:
                    diff_valid = True
                    print("DIFF < 0")

                forward_speed = landing_path[k][0]-current_position[0]
                vy = landing_path[k][1]-current_position[1]
                vx = landing_path[k][3]-robot.yaw
                if abs(forward_speed) < 0.2 and abs(vy) < 0.2:
                    print("IN SMTH")
                    if (k+1) % 2 != 0:
                        comp_points[int(k/2)] = 1 if diff_valid else RESET
                    k += 1
                    if k == 8:
                        k = -1
                if k == -1:
                    print("in adjusting landing")
                    landing_position = robot.adjust_landing(comp_points, landing_position)
                    state = "FOUND_LANDING_PAD"
                
                cf.commander.send_hover_setpoint(forward_speed, vy, le.sensor_data["stateEstimate.z"], 0)

            elif state == "FOUND_LANDING_PAD":
                print("IN FOUND LANDING PAD")
                current_position = (le.sensor_data["stateEstimate.x"], le.sensor_data["stateEstimate.y"])
                forward_speed = landing_position[0]-current_position[0]
                vy = landing_position[1]-current_position[1]
                if abs(forward_speed) < 0.2 and abs(vy) < 0.2:
                    state = "LANDING"
                cf.commander.send_hover_setpoint(vx, vy, le.sensor_data["stateEstimate.z"], 0)

            elif state == "LANDING":
                if landing_pos == False:
                    x_land_pos = le.sensor_data["stateEstimate.x"]
                    y_land_pos = le.sensor_data["stateEstimate.y"]
                    print("LAND POS : ", x_land_pos, " ", y_land_pos)
                    landing_pos = True
                #print("LANDING")
                vx = vy = forward_speed = HALT_SPEED
                cf.commander.send_position_setpoint(x_land_pos, y_land_pos, le.sensor_data["stateEstimate.z"]-0.02, 0)
                #cf.commander.send_hover_setpoint(0, 0, le.sensor_data["stateEstimate.z"]-0.2, 0)
                if le.sensor_data["stateEstimate.z"] <= 0.11:
                    cf.commander.send_stop_setpoint()
                    kill_motor = True

                    # counter += 1
                    # vx = vy = forward_speed = HALT_SPEED
                    # if counter > 50:
                    #    counter = RESET
                    #    if final_target:
                    #        vx = vy = forward_speed = HALT_SPEED
                    #    else:
                    #        state = "TAKE_OFF"

            # if starting_edge == True and ending_edge == False and le.sensor_data["stateEstimate.x"] > 1:
            #    if diff_sum > 0:
            #        ending_edge = True
            #        print("end")
            #        x_end = le.sensor_data["stateEstimate.x"]
            #        delta_pad = x_init-x_end
            #        x_landing_center = le.sensor_data["stateEstimate.x"] + delta_pad
            #        print("delta pad", delta_pad)
            #        print("Position in X is : ", le.sensor_data["stateEstimate.x"])
            #        print("Position in X should be : ", x_landing_center)
#
            # if ending_edge == True:
            #    if le.sensor_data["stateEstimate.z"] < 0.14 or kill_motor == True:
            #        kill_motor = True
            #        cf.commander.send_stop_setpoint()
            #        print("kill")
            #    if kill_motor == False:
            #        cf.commander.send_position_setpoint(
            #            x_landing_center, le.sensor_data["stateEstimate.y"], le.sensor_data["stateEstimate.z"] - 0.1, 0)
            #        print("z: ", le.sensor_data["stateEstimate.z"])

        # print(le.sensor_data["v.z"])

        time.sleep(0.01)

        robot.update(le.sensor_data, 0.01)
        if kill_motor == False and state != "LANDING":
            vx, vy, z, yaw_rate = robot.state_update()
        # vz = -(le.sensor_data["stateEstimate.z"] - z)
        # print(le.sensor_data["stateEstimate.z"])
        # cf.commander.send_velocity_world_setpoint(vx, vy, vz , yaw_rate*np.pi/180)
        # print("distance", [np.linalg.norm(robot.pos - x) for x in robot.obst])
        # print("command", [vx, vy, z, yaw_rate])
        # cf.commander.send_hover_setpoint(vx, vy, yaw_rate*180/np.pi, z)
        # cf.commander.send_hover_setpoint(0, 0, yaw_rate*180/np.pi, z)

        if land == False and state != "LANDING":
            if le.sensor_data["stateEstimate.z"] < 0.10 :
                #print(le.sensor_data["stateEstimate.z"])
                cf.commander.send_hover_setpoint(0, 0, 0, z)
            else:
                #print(le.sensor_data["stateEstimate.z"])
                cf.commander.send_hover_setpoint(0.3, 0, 0, z)

    cf.commander.send_stop_setpoint()
    cf.close_link()
