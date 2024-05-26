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
from cflib.crazyflie import HighLevelCommander
import math
import keyboard
import threading


# uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E707')
uri = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E711')

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

from agent import Agent
import numpy as np
from simple_pid import PID




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


# Control from the exercises
index_current_goal_path = 0
end_path = False
land_on_ground = False
end = False

def path_to_command(path,sensor_data,dt):
    global on_ground, height_desired, index_current_goal_path, timer, timer_done, startpos, end_path #, start_blocked
    global try_1, try_2, look_for_landing, try_3, try_4, case, land_on_ground, home, end
    # Start timer
    if (index_current_goal_path == 1) & (timer is None):
        timer = 0
        # print("Time recording started")
    if timer is not None:
        timer += dt
    # Hover at the final setpoint
    if index_current_goal_path == len(path):
        # Uncomment for KF
        ##control_command = [startpos[0], startpos[1], startpos[2]-0.05, 0.0]
        control_command = [0.0, 0.0, height_desired, 0.0]
        if timer_done is None:
            timer_done = True
            # print("Path planing took " + str(np.round(timer,1)) + " [s]")
        return control_command

   
    # if (start_blocked):
    #     index_current_goal_path += 1
    #     print("start blocked, iteration goal_path")

    # Get the goal position and drone position
    current_goal = path[index_current_goal_path]
    x_drone, y_drone, z_drone, yaw_drone = sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down'], sensor_data['yaw']
    distance_drone_to_goal = np.linalg.norm([current_goal[0] - x_drone, current_goal[1] - y_drone])
    dx= current_goal[0] - x_drone
    dy= current_goal[1] - y_drone

    vx, vy, r = control_law(dx, dy, yaw_drone, distance_drone_to_goal)
    control_command = [vx, vy, height_desired, r]


    if(look_for_landing):
        # print("entre ")
        if distance_drone_to_goal < 0.02:

            if sensor_data['z_global'] - sensor_data['range_down'] > 0.05 :
                control_command = [0.0, 0.0, 0.0, 0.0]
                # print("LANDING")
                look_for_landing = False
                land_on_ground = True
                #case ="stabilization_go_home"
            else:
                # print("change of case")
                control_command = [0.0, 0.0, height_desired, 0.0]


    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    elif distance_drone_to_goal < 0.1:
        # Select the next setpoint as the goal position 
        index_current_goal_path += 1
        # print("lenght path ", len(path))
        # print("index_current_goal_path ", index_current_goal_path)
        # Hover at the final setpoint
        if index_current_goal_path == len(path):
            if(home and index_current_goal_path == len(path)):
                # print("fin path home")
                control_command = [0.0, 0.0, 0.0, 0.0]
                end = True 
            control_command = [0.0, 0.0, height_desired, 0.0]
            # print("FIN")
            end_path = True
            index_current_goal_path = 0

            return control_command

    return control_command


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
    #     yawrate is in degrees/sp
    
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we 
    # are just waiting until we are disconnected.
    height_vect = []
    i=0
    goal_idx = 0
    # case = "snake_following"

    min_x, max_x = 0, 5.0 # meter
    min_y, max_y = 0, 3.0 # meter
    res_pos = 0.1 # meter
    height_desired = 1.0
    avg_height = 0

    x_init = 0
    x_end = 0
    y_end = 0
    dx = 0
    dy = 0
    x_landing_center = 0
    y_landing_center = 0
    x_before_landing = 0
    y_before_landing = 0
    x_offset_landing = 0
    y_offset_landing = 0
    delta_pad = 0 
    k = 100

    # Bool for landing pad search
    starting_edge = False
    ending_edge = False
    kill_motor = False
    started = False
    final_landing = False
    offset_computed = False
    

    heading = "front"
    landing_state = "INIT"


    # def plot_thread():
    #     robot.show_plot()

    # def on_key_event(event):
    #     if event.name == 'p':  # Check if the 'p' key is pressed
    #         threading.Thread(target=plot_thread).start()

    # keyboard.on_press(on_key_event)

    while robot.alive:

        HighCommander = HighLevelCommander(cf)

        z = le.sensor_data["stateEstimate.z"]

        if is_close(le.sensor_data["range.up"]):
            break

        # print(le.sensor_data["range.left"])
        # print(le.sensor_data["range.right"])
        # print(le.sensor_data["range.frppont"])
        # print(le.sensor_data["range.back"]) 

        if le.sensor_data["stateEstimate.z"] > 0.15:
            start_search = True

        if start_search == True:
            height_vect.append(le.sensor_data["stateEstimate.z"] * HEIGHT_COEFF)

            if i > 8:
                height_vect.pop(0)
                for l in range(8):
                    if l < 4:
                        height_vect[l] = 0.3*height_vect[l]
                    else:
                        height_vect[l] = 0.7*height_vect[l]
            else:
                i += 1

            avg_height = np.mean(height_vect)
            # print("ave_height :", avg_height)

            if (avg_height > 12) and le.sensor_data["stateEstimate.x"] > 1 and started == False :
                started = True
                landing_state = "EDGE_DETECTED"
                dx = robot.goal[0] - robot.pos[0]
                dy = robot.goal[1] - robot.pos[1]
                if dx > dy : 
                    if dx > 0 :
                        heading = "front" # front
                    else : 
                        heading = "back" # back
                else :
                    if dy > 0:
                        heading = "left" # left
                    else : 
                        heading = "right" # right
                if heading == "front" :
                    x_landing_center = le.sensor_data["stateEstimate.x"]
                    y_landing_center = le.sensor_data["stateEstimate.y"] +0.05
                elif heading == "back" :
                    x_landing_center = le.sensor_data["stateEstimate.x"]
                    y_landing_center = le.sensor_data["stateEstimate.y"]+0.05
                elif heading == "left" :
                    x_landing_center = le.sensor_data["stateEstimate.x"]
                    y_landing_center = le.sensor_data["stateEstimate.y"]+0.05
                else :
                    x_landing_center = le.sensor_data["stateEstimate.x"]
                    y_landing_center = le.sensor_data["stateEstimate.y"]+0.05

                print("EDGE_DETECTED")
                landing_state = "CENTER_DRONE"
                stop_cycles = 40
                search_cycles = 40
            
            elif landing_state == "EDGE_DETECTED":
                # HighCommander.go_to(x_landing_center, y_landing_center, z, 0, 0.2, relative=False)
                cf.commander.send_position_setpoint(x_landing_center, y_landing_center, z, 0)
                if abs(x_landing_center - robot.pos[0]) < 0.01 and abs(y_landing_center - robot.pos[1]) < 0.01: 
                    cf.commander.send_hover_setpoint(0, 0, 0, z)
                    landing_state = "END_EDGE_SEARCH"
                    print("END_EDGE_SEARCH")
                
            elif landing_state == "END_EDGE_SEARCH" and le.sensor_data["stateEstimate.x"] > 1:
                if heading == "front" or heading == "back" :
                    if abs(x_landing_center-le.sensor_data["stateEstimate.x"]) < 0.008:
                        landing_state = "CENTERING_2nd_LANDING"
                        HighCommander.go_to(0, 0, 0, 0, 0.2, relative=True) # Test instead of hover
                        # cf.commander.send_hover_setpoint( 0.0, 0, 0, le.sensor_data["stateEstimate.z"])
                    else :
                        # HighCommander.go_to(x_landing_center, y_landing_center, z, 0, 0.2, relative=False)
                        cf.commander.send_position_setpoint(x_landing_center, y_landing_center, le.sensor_data["stateEstimate.z"], 0)
                else : 
                    if abs(y_landing_center-le.sensor_data["stateEstimate.y"]) < 0.008:
                        landing_state = "CENTERING_2nd_LANDING"
                        # HighCommander.go_to(0, 0, z, 0, 0.2, relative=True) # Test instead of hover
                        cf.commander.send_hover_setpoint( 0.0, 0, 0, le.sensor_data["stateEstimate.z"])
                    else :
                        HighCommander.go_to(x_landing_center, y_landing_center, z, 0, 0.2, relative=False)
                        # cf.commander.send_position_setpoint(x_landing_center, y_landing_center, le.sensor_data["stateEstimate.z"], 0)

            # Detect ending edge for the 2nd axis
            elif landing_state == "CENTERING_2nd_LANDING":
                if avg_height > 12:
                    print("entering_2nd_axis, stat is CENTER_DRONE")
                    landing_state = "CENTER_DRONE"
                    if heading == "front" or heading == "back" :
                        y_end = le.sensor_data["stateEstimate.y"]
                        y_landing_center = y_end - 0.05
                        print(f"CENTER_DRONE, position delta is {y_landing_center}, ending point is : {y_end} ")
                        while search_cycles > 0:
                            # HighCommander.go_to(0, 0, z, 0, 0.2, relative=True) # Test instead of hover
                            cf.commander.send_hover_setpoint( 0.0, 0, 0, le.sensor_data["stateEstimate.z"])
                            time.sleep(0.1)
                            search_cycles -= 1
                    else :
                        x_end = le.sensor_data["stateEstimate.x"]
                        x_landing_center = x_end - 0.05
                        print(f"CENTER_DRONE, position delta is {x_landing_center}, ending point is : {y_end} ")
                        while search_cycles > 0:
                            HighCommander.go_to(0, 0, 0, 0, 0.2, relative=True) # Test instead of hover
                            time.sleep(0.1)
                            # cf.commander.send_hover_setpoint( 0.0, 0, 0, le.sensor_data["stateEstimate.z"])
                            search_cycles -= 1
                else :
                    if heading == "front" or heading == "back" :
                        HighCommander.go_to(0, 0.01, 0, 0, 0.2, relative=True) # Test instead of hover
                        # cf.commander.send_position_setpoint(le.sensor_data["stateEstimate.x"], y_landing_center+0.01, le.sensor_data["stateEstimate.z"], 0)
                    else:
                        HighCommander.go_to(0.01, 0, 0, 0, 0.2, relative=True) # Test instead of hover
                        # cf.commander.send_position_setpoint(le.sensor_data["stateEstimate.x"]+0.01, y_landing_center, le.sensor_data["stateEstimate.z"], 0)
                
            elif landing_state == "CENTER_DRONE":
                print(f"Position desired is {x_landing_center} and : {y_landing_center} ")        
                print(f"Position actual is {le.sensor_data["stateEstimate.x"]} and : {le.sensor_data["stateEstimate.y"]} ")                     
                cf.commander.send_position_setpoint(x_landing_center, y_landing_center, le.sensor_data["stateEstimate.z"], 0)
                if abs(x_landing_center-le.sensor_data["stateEstimate.x"]) < 0.005 and abs(y_landing_center-le.sensor_data["stateEstimate.y"]) < 0.005:
                    landing_state = "LAND"
                    print("LAND")
                    HighCommander.land(0.1, 1)
                    time.sleep(0.1)
            
            elif landing_state == "LAND":
                if le.sensor_data["stateEstimate.z"] > 0.12:
                    cf.commander.send_position_setpoint(x_landing_center, y_landing_center, le.sensor_data["stateEstimate.z"]-0.06, 0)
                else: 
                    x_before_landing = le.sensor_data["stateEstimate.x"]
                    y_before_landing = le.sensor_data["stateEstimate.y"]
                    cf.commander.send_stop_setpoint()
                    started = False
                    time.sleep(2.5)
                    if final_landing == True : 
                        cf.close_link()
                    else:
                        landing_state = "GO_HOME"
            
            elif landing_state == "GO_HOME":
                if le.sensor_data["stateEstimate.z"] < robot.z_target:
                    print("going home")
                    cf.commander.send_hover_setpoint(0, 0, 0, le.sensor_data["stateEstimate.z"]+0.06)
                else :
                    if not offset_computed :
                        x_offset_landing =  le.sensor_data["stateEstimate.x"]
                        y_offset_landing =  le.sensor_data["stateEstimate.y"]
                        offset_computed = True
                    print("REACHED GOING HOME")
                    robot.goal = (0 - x_offset_landing, 0 - y_offset_landing)
                    vx, vy, z, yaw_rate  = robot.go_home()
                    cf.commander.send_hover_setpoint(vx , vy, yaw_rate*180/np.pi, z)
                    if abs(robot.goal[0]-le.sensor_data["stateEstimate.x"]) < 0.005 and abs(robot.goal[1]-le.sensor_data["stateEstimate.y"]) < 0.005:
                        landing_state = "LAND"
                        print("LAND")
                        x_landing_center = 0 - x_offset_landing
                        y_landing_center = 0 - y_offset_landing
                        final_landing = True
                        cf.commander.send_position_setpoint(x_landing_center, y_landing_center, le.sensor_data["stateEstimate.z"]-0.06, 0)

        time.sleep(0.01)

        robot.update(le.sensor_data, 0.01)
        if le.sensor_data["stateEstimate.z"] < robot.z_target:
            cf.commander.send_hover_setpoint(0, 0,0, le.sensor_data["stateEstimate.z"]+0.1)
        else:
            if started == False : 
                vx, vy, z, yaw_rate = robot.state_update()
                cf.commander.send_hover_setpoint(vx , vy, yaw_rate*180/np.pi, z) 
      
    cf.commander.send_stop_setpoint()
    cf.close_link()

 