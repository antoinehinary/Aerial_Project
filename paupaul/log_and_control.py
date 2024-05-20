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
import math

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

class DroneStateMachine:
    def __init__(self, cf, le):
        self.cf = cf
        self.le = le
        self.state = "INIT"
        self.start_search = False
        self.starting_edge = False
        self.ending_edge = False
        self.height_vect = []
        self.avg_height = 0
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
                print("MOVE_FORWARD")
            else:
                self.cf.commander.send_hover_setpoint(0, 0, 0, z + 0.02)

        elif self.state == "MOVE_FORWARD":
            self.cf.commander.send_hover_setpoint(0.2, 0, 0, z)

            if (self.avg_height > 12) and not self.starting_edge and le.sensor_data["stateEstimate.x"] > 1:
                self.state = "EDGE_DETECTED"
                print("EDGE_DETECTED")
                self.starting_edge = True
                self.stop_cycles = 20

        elif self.state == "EDGE_DETECTED":
            if self.stop_cycles > 0:
                self.cf.commander.send_hover_setpoint(0, 0, 0, z)
                self.stop_cycles -= 1
            else:
                self.cf.commander.send_position_setpoint(sensor_data["stateEstimate.x"] + 0.15, sensor_data["stateEstimate.y"], z, 0)
                self.state = "SEARCH_FALLING_EDGE"
                print("SEARCH_FALLING_EDGE")

        elif self.state == "SEARCH_FALLING_EDGE":
            self.cf.commander.send_hover_setpoint(0, -0.1, 0, z)
            if (self.avg_height > 12) and not self.starting_edge and le.sensor_data["stateEstimate.x"] > 1:
                self.state = "MOVE_TO_LAND_POSITION"
                print("MOVE_TO_LAND_POSITION")

        elif self.state == "MOVE_TO_LAND_POSITION":
            self.cf.commander.send_position_setpoint(sensor_data["stateEstimate.x"], sensor_data["stateEstimate.y"] - 0.15, z)
            if abs(sensor_data["stateEstimate.y"] - (self.y_landing_center - 0.15)) < 0.01:
                self.state = "LANDING"
                print("LANDING")

        elif self.state == "LANDING":
            self.cf.commander.send_hover_setpoint(0, 0, 0, z - 0.02)
            if z < 0.2:
                self.state = "LANDED"

        elif self.state == "LANDED":
            self.cf.commander.send_stop_setpoint()
def control_law(dx, dy, current_yaw, goal_dist):
    kp_yaw = 0.5
    kp_pos = 0.22  # 0.5 : chao, 0.3 marche
    kd = 0
    max_speed = 1
    # Calculate desired yaw angle
    desired_yaw = math.atan2(dx, dy)

    # Calculate yaw rate command
    yaw_rate = kp_yaw * (desired_yaw - current_yaw)

    # Limit yaw rate to a maximum value
    max_yaw_rate = math.pi / 4  # Example maximum yaw rate (45 degrees per second)
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))

    # Calculate velocity commands
    dpos = np.array([dx, dy])
    v_word = kp_pos * dpos / goal_dist #normalize
    
    vel_rot_mat = np.array(
            [
                [np.cos(-current_yaw), -np.sin(-current_yaw)],
                [np.sin(-current_yaw), np.cos(-current_yaw)],
            ]
        )
    vx_body, vy_body = np.dot(vel_rot_mat, [v_word[0], v_word[1]])

    # vel_x = kp_pos * dx/goal_dist #normalize
    # vel_y = kp_pos * dy/goal_dist

    # Limit velocity commands to maximum speed
    vx_body = max(-max_speed, min(max_speed, vx_body))
    vy_body = max(-max_speed, min(max_speed, vy_body))

    return vx_body, vy_body, yaw_rate

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
            height_vect.append(le.sensor_data["stateEstimate.z"] * HEIGHT_COEFF)

            # Manage height vector size
            if i > 8:
                height_vect.pop(0)
            else:
                i += 1

            # Calculate height difference
            height_diff = height_vect[-1] - height_vect[0]
            diff_height_vect.append(height_diff)

            # Manage difference vector size
            # if i > 30:
            #     diff_height_vect.pop(0)

            # # Calculate sum of height differences and average height
            # diff_sum = np.sum(diff_height_vect)
            if i > 8:
                for l in range(8):
                    if l < 4:
                        height_vect[l] = 0.3*height_vect[l]
                    else:
                        height_vect[l] = 0.7*height_vect[l]

            avg_height = np.mean(height_vect)

            # Detect starting edge
            #diff_sum < 0 or 
            if (avg_height > 12) and not starting_edge and le.sensor_data["stateEstimate.x"] > 1:
                starting_edge = True
                land = True
                state = "END_EDGE_SEARCH"
                x_init = le.sensor_data["stateEstimate.x"]
                x_landing_center = x_init+0.05
                cf.commander.send_hover_setpoint(0, 0, 0, le.sensor_data["stateEstimate.z"])

                print(f"AVG: {avg_height}, start, X init: {x_init}")

            # Detect ending edge
            elif state == "END_EDGE_SEARCH" and le.sensor_data["stateEstimate.x"] > 1:
                print("END_EDGE_SEARCH")
                # print(f"AVG: {avg_height}, Diff: {diff_sum}")
                # if avg_height > 14:
                #     ending_edge = True
                #     x_end = le.sensor_data["stateEstimate.x"]
                #     delta_pad_x = x_end - x_init
                #     x_landing_center = x_end - landing_pad_mid_length
                #     state = "CENTERING_X_LANDING"
                #     print(f"End, delta pad: {delta_pad_x}, Position X: {le.sensor_data['stateEstimate.x']}, Should be: {x_landing_center}")
                #     cf.commander.send_hover_setpoint( 0.0, 0, 0, le.sensor_data["stateEstimate.z"])
                if abs(x_landing_center-le.sensor_data["stateEstimate.x"]) < 0.008:
                    state = "CENTERING_Y_LANDING"
                    cf.commander.send_hover_setpoint( 0.0, 0, 0, le.sensor_data["stateEstimate.z"])
                else :
                    cf.commander.send_position_setpoint(x_landing_center, le.sensor_data["stateEstimate.y"], le.sensor_data["stateEstimate.z"], 0)

            # Detect ending edge for the 2nd axis
            elif state == "CENTERING_Y_LANDING":
                print("entering_y_landing")
                if avg_height > 14:
                    state = "CENTER_DRONE"
                    y_end = le.sensor_data["stateEstimate.y"]
                    y_landing_center = y_end - landing_pad_mid_length
                    print(f"CENTER_DRONE, position delta is {y_landing_center}, ending point is : {y_end} ")
                    cf.commander.send_hover_setpoint( 0.0, 0, 0, le.sensor_data["stateEstimate.z"])
                cf.commander.send_position_setpoint(x_landing_center, le.sensor_data["stateEstimate.y"]+0.02, le.sensor_data["stateEstimate.z"], 0)

            # Perform landing
            # elif state == "CENTERING_X_LANDING":
                
            #     print("FIIIIIIIIIIIIIIIIIIIIIIIIIIIN")
            #     # if le.sensor_data["stateEstimate.z"] < 0.14 or kill_motor:
            #     #     kill_motor = True
            #     #     cf.commander.send_stop_setpoint()
            #     #     if not kill_motor:
            #     #         print("kill")
            #     # else:
            #     cf.commander.send_position_setpoint( le.sensor_data["stateEstimate.x"]-0.02, 
            #         le.sensor_data["stateEstimate.y"], 
            #         le.sensor_data["stateEstimate.z"]-0.02,
            #         0
            #     )
            #     if abs(x_landing_center-le.sensor_data["stateEstimate.x"]) < 0.01:
            #         print("CENTERING_Y_LANDING")
            #         y_init = le.sensor_data["stateEstimate.y"]
            #         state = "CENTERING_Y_LANDING"
            #         cf.commander.send_position_setpoint(le.sensor_data["stateEstimate.x"]-0.01, le.sensor_data["stateEstimate.y"], le.sensor_data["stateEstimate.z"]-0.01, 0)

            elif state == "CENTER_DRONE":
                print(f"Position desired is {x_landing_center} and : {y_landing_center} ")        
                print(f"Position actual is {le.sensor_data["stateEstimate.x"]} and : {le.sensor_data["stateEstimate.y"]} ")                     
                cf.commander.send_position_setpoint(x_landing_center, y_landing_center, le.sensor_data["stateEstimate.z"]-0.006, 0)
                if abs(x_landing_center-le.sensor_data["stateEstimate.x"]) < 0.01 and abs(y_landing_center-le.sensor_data["stateEstimate.y"]) < 0.01:
                    state = "LAND"

            elif state == "LAND":
                cf.commander.send_position_setpoint(x_landing_center, y_landing_center, le.sensor_data["stateEstimate.z"]-0.02, 0)
                if le.sensor_data["stateEstimate.z"] < 0.14:
                    cf.commander.send_stop_setpoint()
                print(f"z: {le.sensor_data['stateEstimate.z']}")


        # print(le.sensor_data["v.z"])

        time.sleep(0.01)

        robot.update(le.sensor_data, 0.01)
        if kill_motor == False and not land:
            vx, vy, z, yaw_rate = robot.state_update()
        # vz = -(le.sensor_data["stateEstimate.z"] - z)
        # print(le.sensor_data["stateEstimate.z"])
        # cf.commander.send_velocity_world_setpoint(vx, vy, vz , yaw_rate*np.pi/180)
        # print("distance", [np.linalg.norm(robot.pos - x) for x in robot.obst])
        # print("command", [vx, vy, z, yaw_rate])
        # cf.commander.send_hover_setpoint(vx, vy, yaw_rate*180/np.pi, z)
        # cf.commander.send_hover_setpoint(0, 0, yaw_rate*180/np.pi, z)

        if land == False:
            if le.sensor_data["stateEstimate.z"] < 0.10 :
                # print(f"STATE IS : {le.sensor_data["stateEstimate.x"]}, {le.sensor_data["stateEstimate.y"]}")
                #print(le.sensor_data["stateEstimate.z"])
                cf.commander.send_hover_setpoint(0, 0, 0, z)
            else:
                # print(f"STATE dans laire : {le.sensor_data["stateEstimate.x"]}, {le.sensor_data["stateEstimate.y"]}")
                #print(le.sensor_data["stateEstimate.z"])
                cf.commander.send_hover_setpoint(0.3, 0, 0, z)

    cf.commander.send_stop_setpoint()
    cf.close_link()
