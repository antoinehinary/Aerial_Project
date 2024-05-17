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


# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.colors as colors
from skimage.morphology import binary_dilation, square


uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E707')
HEIGHT_COEFF = 100
starting_edge = False
ending_edge = False
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



    #   self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
    #     self._lg_stab.add_variable('stateEstimate.x', 'float')
    #     self._lg_stab.add_variable('stateEstimate.y', 'float')
    #     self._lg_stab.add_variable('stateEstimate.z', 'float')
    #     self._lg_stab.add_variable('stabilizer.yaw', 'float')
    #     self._lg_stab.add_variable('range.front')
    #     self._lg_stab.add_variable('range.back')
    #     self._lg_stab.add_variable('range.left')
    #     self._lg_stab.add_variable('range.right')
    #     self._lg_stab.add_variable('range.up')

# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.1 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting
map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied


def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data["stateEstimate.x"]
    pos_y = sensor_data["stateEstimate.y"]
    yaw = sensor_data["stabilizer.yaw"]
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the current_setpoint is within the map
            if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    # flip the map at bottom left corner to match the coordinate system of matplotlib to pot
    # if t % 50 == 0:
    #     plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
    #     plt.savefig("map.png")
    #     plt.close()
    # t +=1

    return map

def snake_creation():
    goal_list = []
    for i in range(10, 30, +3): 
        if i % 2 ==1:               # must be 4 to be correct 
            for j in range(2, 28, +3):
                goal_list.append((i,j))
        if i % 2 == 0:
            for j in range(29, 2, -3):
                goal_list.append((i,j))
    return goal_list

def _get_movements_4n():
    """
    Get all possible 4-connectivity movements (up, down, left right).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]

def _get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
    (up, down, left, right and the 4 diagonals).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]

def A_Star_function(start, goal, occupancy_grid):
    
    # Dimensions of the occupancy grid
    x_dim = occupancy_grid.shape[0]  
    y_dim = occupancy_grid.shape[1]  

    # Get all the possible components of the occupancy_grid matrix
    x,y = np.mgrid[0:x_dim:1, 0:y_dim:1]
    pos = np.empty(x.shape + (2,))
    pos[:, :, 0] = x
    pos[:, :, 1] = y # superimpose x and y
    pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
    coords = list([(int(x[0]), int(x[1])) for x in pos])

    # Define the heuristic, here = distance to goal ignoring obstacles
    h = np.linalg.norm(pos - goal, axis=-1)
    # Calculate the total heuristic for each cell

    h = dict(zip(coords, h))

    # check errors :
    # Check if the start and goal are within the boundaries of the map
    for point in [start, goal]:
        assert point[0]>=0 and point[0]<occupancy_grid.shape[0],"start or end goal not contained in the map"
        assert point[1]>=0 and point[1]<occupancy_grid.shape[1],"start or end goal not contained in the map"
    
    # check if start and goal nodes correspond to free spaces
    # if occupancy_grid[start[0], start[1]]:
    #     print("start:", start[0], start[1])
    #     raise Exception('Start node is not traversable')

    if occupancy_grid[goal[0], goal[1]]:
        raise Exception('Goal node is not traversable')
    #run A* algorithm
    path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid)
    #print("path", path)

    return path, visitedNodes

def filtered_path(path, occupancy_grid):
    if len(path) < 2:
        return path

    new_filtered_tab = [path[0]]
    prec_dx, prec_dy = path[1][0]- path[0][0] , path[1][1] - path[0][1]

    for i in range(1, len(path) - 1):
        point = path[i]
        next_point = path[i + 1]
        dx, dy = next_point[0] - point[0], next_point[1] - point[1]

        if is_near_obstacle(occupancy_grid, point):
            new_filtered_tab.append(point)
            prec_dx, prec_dy = dx, dy
            continue

        if dx != prec_dx or dy != prec_dy:
            new_filtered_tab.append(point)
            prec_dx, prec_dy = dx, dy

    new_filtered_tab.append(path[-1])

    return new_filtered_tab

def grid_to_meters(path, res_pos, min_x, min_y):
    new_path = []
    for cell in path:
        x = round(cell[0] * res_pos + min_x, 2)
        y = round(cell[1] * res_pos + min_y, 2)
        new_path.append((x, y))
    return new_path


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


# self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
# self._lg_stab.add_variable('stateEstimate.x', 'float')
# self._lg_stab.add_variable('stateEstimate.y', 'float')
# self._lg_stab.add_variable('stateEstimate.z', 'float')
# self._lg_stab.add_variable('stabilizer.yaw', 'float')
# self._lg_stab.add_variable('range.front')
# self._lg_stab.add_variable('range.back')
# self._lg_stab.add_variable('range.left')
# self._lg_stab.add_variable('range.right')
# self._lg_stab.add_variable('range.up')

if __name__ == '__main__':
    # Initialize the low-level drivers
    global on_ground, startpos, case, first_loop_begining, turn_right, turn_left, found_pink_square,first_y_init 
    global start, goal, goal_idx, path, path_meters,occupancy_grid,list_goal,end_path, index_current_goal_path
    global try_1, look_for_landing, land_on_ground, home, pos_landing
    cflib.crtp.init_drivers()
    crf = Crazyflie(rw_cache='./cache')

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # robot = Agent(le.sensor_data, 0.1)
    # robot.update(le.sensor_data, 0.1)

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
    i=0
    height_desired = 1.0
    sensor_data = le.sensor_data
    startpos = True
    goal_idx = 0


    while True:

        if is_close(le.sensor_data["range.up"]):
            break

        # print(le.sensor_data["range.left"])
        # print(le.sensor_data["range.right"])
        # print(le.sensor_data["range.front"])
        # print(le.sensor_data["range.back"])  

###################### state machine #########################
    
    # Take off
        if startpos is None:
            startpos = [sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y'], sensor_data['stateEstimate.z']]   
 
        if on_ground and sensor_data['stateEstimate.z'] < 0.49:
            ### function to go up 
            print("take off")
            cf.commander.send_hover_setpoint(0 , 0, 0, height_desired)
            continue
        else:
            on_ground = False

        # stabilization
        map = occupancy_map(sensor_data)

        occupancy_grid = binary_dilation(map < - 0.1, square(3))
        # print("map start", occupancy_grid[start[0], start[1]])
        # print("map goal", occupancy_grid[goal[0], goal[1]])
        occupancy_grid[0,:] = True
        occupancy_grid[-1,:] = True
        occupancy_grid[:,0] = True
        occupancy_grid[:,-1] = True

        if case is None:
            case = "pass"
        # map = occupancy_map(sensor_data)
            
        elif(case=="pass"):
            print("case pass")
            if(sensor_data["yaw"] > 0.9): #30 degrees
                        
                # print("trun_right :", turn_right)
                # print("turn_left: ", turn_left)
                cf.commander.send_hover_setpoint(0 , 0, 0, height_desired)
                case = "pass2"
            else:
                control_command = [0.0, 0.0, height_desired, 1]
        elif(case =="pass2"):
            if(sensor_data["yaw"] < -0.9): #-30 degrees
            
                # print("trun_right :", turn_right)
                # print("turn_left: ", turn_left)
                cf.commander.send_hover_setpoint(0 , 0, 0, height_desired)
                case = "snake"
                
        elif(case =="snake"):
            list_goal = snake_creation()
            case ="snake_following"

        elif(case == "snake_following"):
            # print("snake_following")
            start = (int((sensor_data["stateEstimate.x"]-min_x)/res_pos), int((sensor_data['stateEstimate.y']-min_y)/res_pos))

            
            # print("goal id :", goal_idx)
            goal = list_goal[goal_idx]
            while(occupancy_grid[goal[0]][goal[1]] != 0): ## add end lenght snake
                goal_idx = goal_idx + 1
                goal = list_goal[goal_idx]
                # print("goal id :", goal_idx)
            # print("current goal id :", goal_idx)    
            if(goal_idx == len(list_goal)-1):
                # print("FIN SNAKE")
                cf.commander.send_hover_setpoint(0 , 0, 0, height_desired)
            # if(occupancy_grid[start[0], start[1]] != 0):
            #     occupancy_grid[start[0], start[1]] = 0
            #     start_blocked = True

            path, visitedNodes = A_Star_function(start, goal, occupancy_grid)
            path_filter = filtered_path(path, occupancy_grid)

            path_meters = grid_to_meters(path_filter, res_pos, min_x, min_y)
            case = "go"

        elif(case == "go"):
            # print("go")
            
            for i in range(1, len(path)):
                (a,b) = path[i]
                if occupancy_grid[a][b] != 0:
                    #return A*
                    index_current_goal_path = 0
                    case = "snake_following"
                    # print("obstacle non detected")
                    control_command = [0.0, 0.0, height_desired, 0.0]



            if case == "go":
                if end_path :
                    # print("goal id + 1")
                    goal_idx = goal_idx + 1 
                    case = "snake_following"
                    end_path = False
                if sensor_data['z_global'] - sensor_data['range_down'] > 0.05 and sensor_data["x_global"]>3.5:
                    pos_landing = [sensor_data['x_global'], sensor_data['y_global']]
                    # print("pos_landing:", pos_landing)
                    case = "go_ground"
                    try_1 = True
                    control_command= [0.0, 0.0, height_desired, 0.0]
                else:
                    # print("sending a command")
                    control_command = path_to_command(path_meters,sensor_data,dt)



####################################################################
        if le.sensor_data["stateEstimate.z"] > 0.5 :
            start_search = True

        if start_search == True:
            height_vect.append(le.sensor_data["stateEstimate.z"]*HEIGHT_COEFF) 

            #print(le.sensor_data["stateEstimate.z"])
            if i > 40:
                height_vect.pop(0)
            else :
                i += 1
            #print("vect : ", vect)

            height_diff = height_vect[i-1] - height_vect[0]
            #print("height: ", height_diff)


            diff_height_vect.append(height_diff)

            if i > 30:
                diff_height_vect.pop(0)

            diff_sum = np.sum(diff_height_vect)
            if diff_sum < 0 :
                print("start")
                if i > 30:
                    starting_edge = True
            if starting_edge == True:
                if diff_sum > 0:
                    ending_edge = True
                    print("end")

        # print(le.sensor_data["v.z"]) 
        time.sleep(0.01)

        ## robot.update(le.sensor_data, 0.01)
        ## vx, vy, z, yaw_rate = robot.state_update()
        # vz = -*(le.sensor_data["stateEstimate.z"] - z)
        # print(le.sensor_data["stateEstimate.z"])
        # cf.commander.send_velocity_world_setpoint(vx, vy, vz , yaw_rate*np.pi/180)
        # print("distance", [np.linalg.norm(robot.pos - x) for x in robot.obst])
        # print("command", [vx, vy, z, yaw_rate])
        # cf.commander.send_hover_setpoint(vx, vy, yaw_rate*180/np.pi, z)
        # cf.commander.send_hover_setpoint(0, 0, yaw_rate*180/np.pi, z)

        cf.commander.send_hover_setpoint(0.3 , 0, 0, z)
        

    cf.commander.send_stop_setpoint()
    cf.close_link()
    

# Global variables
on_ground = True
height_desired = 1.0
timer = None
startpos = None
timer_done = None
case = None
first_loop_begining =True
turn_right = False
turn_left = False
found_pink_square = False
first_y_init = 0.0
i=1
dt = 0.009


##################################################################################
VIEW_ANGLE = 0.7854
PINK_ZONE_DISANCE = 2.3

goal_idx = 0
look_for_landing = False
home = False
# This is the main function where you will implement your control algorithm
def get_command(sensor_data, camera_data, dt):
    global on_ground, startpos, case, first_loop_begining, turn_right, turn_left, found_pink_square,first_y_init 
    global start, goal, goal_idx, path, path_meters,occupancy_grid,list_goal,end_path, index_current_goal_path
    global try_1, look_for_landing, land_on_ground, home, pos_landing
    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    # cv2.imshow('Camera Feed', camera_data)
    # cv2.waitKey(1)
    
    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]    
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, height_desired, 0.0]
        return control_command
    else:
        on_ground = False

    # ---- YOUR CODE HERE ----
    control_command = [0.0, 0.0, height_desired, 0.0]
    on_ground = False
    map = occupancy_map(sensor_data)

    occupancy_grid = binary_dilation(map < - 0.1, square(3))
    # print("map start", occupancy_grid[start[0], start[1]])
    # print("map goal", occupancy_grid[goal[0], goal[1]])
    occupancy_grid[0,:] = True
    occupancy_grid[-1,:] = True
    occupancy_grid[:,0] = True
    occupancy_grid[:,-1] = True

    if case is None:
        case = "stabilization"
    # map = occupancy_map(sensor_data)
    if(case == "stabilization"):
        if(height_desired-sensor_data["range_down"] < 0.05):
            # print("stabilization done")
            case = "pass"
        control_command = [0.0, 0.0, height_desired, 0.0]
        
    elif(case=="pass"):
        if(sensor_data["yaw"] > 0.9): #30 degrees
                    
            # print("trun_right :", turn_right)
            # print("turn_left: ", turn_left)
            control_command = [0.0, 0.0, height_desired, 0.0]
            case = "pass2"
        else:
            control_command = [0.0, 0.0, height_desired, 1]
    elif(case =="pass2"):
        if(sensor_data["yaw"] < -0.9): #-30 degrees
        
            # print("trun_right :", turn_right)
            # print("turn_left: ", turn_left)
            control_command = [0.0, 0.0, height_desired, 0.0]
            case = "snake"
        
        else:
            control_command = [0.0, 0.0, height_desired, -1]

    elif(case =="snake"):
        list_goal = snake_creation()
        # plt.imshow(np.flip(occupancy_grid, 1),
        #                cmap='binary', origin='lower')
        # for cell in list_goal:
        #     plt.plot(len(occupancy_grid[0])-1 -
        #                 cell[1], cell[0], 'o', color='orange')
        # plt.colorbar(label='Binary Map Value')
        # plt.title('Binary Map')
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.show()
        case ="snake_following"

    elif(case == "snake_following"):
        # print("snake_following")
        start = (int((sensor_data["stateEstimate.x"]-min_x)/res_pos), int((sensor_data["stateEstimate.y"]-min_y)/res_pos))
        # print("map start", occupancy_grid[start[0], start[1]])
            

        # print("goal id :", goal_idx)
        goal = list_goal[goal_idx]
        while(occupancy_grid[goal[0]][goal[1]] != 0): ## add end lenght snake
            goal_idx = goal_idx + 1
            goal = list_goal[goal_idx]
            # print("goal id :", goal_idx)
        # print("current goal id :", goal_idx)    
        if(goal_idx == len(list_goal)-1):
            # print("FIN SNAKE")
            control_command = [0.0, 0.0, height_desired, 0.0]

        # if(occupancy_grid[start[0], start[1]] != 0):
        #     occupancy_grid[start[0], start[1]] = 0
        #     start_blocked = True

        path, visitedNodes = A_Star_function(start, goal, occupancy_grid)
        path_filter = filtered_path(path, occupancy_grid)

        # print("path", path_filter)
        # plt.imshow(np.flip(occupancy_grid, 1),
        #             cmap='binary', origin='lower')
        # for cell in path:
        #     plt.plot(len(occupancy_grid[0])-1 -
        #                 cell[1], cell[0], 'o', color='orange')
        # plt.colorbar(label='Binary Map Value')
        # plt.title('Binary Map')
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.show()
        path_meters = grid_to_meters(path_filter, res_pos, min_x, min_y)
        #print("path_meters", path_meters)
        
        # display_optimal_path(start,goal,map)
        case = "go"

    elif(case == "go"):
        # print("go")
        
        for i in range(1, len(path)):
            (a,b) = path[i]
            if occupancy_grid[a][b] != 0:
                #return A*
                index_current_goal_path = 0
                case = "snake_following"
                # print("obstacle non detected")
                control_command = [0.0, 0.0, height_desired, 0.0]



        if case == "go":
            if end_path :
                # print("goal id + 1")
                goal_idx = goal_idx + 1 
                case = "snake_following"
                end_path = False
            if sensor_data['z_global'] - sensor_data['range_down'] > 0.05 and sensor_data["x_global"]>3.5:
                pos_landing = [sensor_data['x_global'], sensor_data['y_global']]
                # print("pos_landing:", pos_landing)
                case = "go_ground"
                try_1 = True
                control_command= [0.0, 0.0, height_desired, 0.0]
            else:
                # print("sending a command")
                control_command = path_to_command(path_meters,sensor_data,dt)


    elif(case == "go_ground"):
        look_for_landing = False

        print("go ground")
        if sensor_data["range_down"] < 0.5:
            control_command = [0.0, 0.0, 0.0, 0.0]
            case = "on_ground"
        else :
            control_command = [0.0, 0.0, 0.4, 0.0]
           
    elif(case == "on_ground"):
        print("on ground")
        if sensor_data["range_down"] < 0.2:
            case = "stabilization_go_home"
        else :
            control_command = [0.0, 0.0, 0.0, 0.0]

    elif(case == "stabilization_go_home"):
       # print("stabilization_go_home")
        if(height_desired-sensor_data["range_down"] < 0.05):
            # print("stabilization done")
            case = "set_home"
        control_command = [0.0, 0.0, height_desired, 0.0]

    elif(case == "set_home"):
        #print("set_home")
        home = True
        start = (int((sensor_data["x_global"]-min_x)/res_pos), int((sensor_data["y_global"]-min_y)/res_pos))
        goal = (int((startpos[0]-min_x)/res_pos), int((startpos[1]-min_y)/res_pos))

        path, visitedNodes = A_Star_function(start, goal, occupancy_grid)
        #print("A* passed")
        path_filter = filtered_path(path, occupancy_grid)
        path_meters = grid_to_meters(path_filter, res_pos, min_x, min_y)

        case = "go_home"

        # plt.imshow(np.flip(occupancy_grid, 1),
        #                cmap='binary', origin='lower')
        # for cell in path:
        #     plt.plot(len(occupancy_grid[0])-1 -
        #                 cell[1], cell[0], 'o', color='orange')
        # plt.colorbar(label='Binary Map Value')
        # plt.title('Binary Map')
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.show()
    
    elif(case == "go_home"):
        # print("going home")
        # print("end", end)
        if end :
            control_command = [0.0, 0.0, 0.4, 0.0]
            case = "home_ground"
        else:
            control_command = path_to_command(path_meters,sensor_data,dt)

    elif(case == "home_ground"):
        # print("on ground")
        if sensor_data["range_down"] < 0.5:
            control_command = [0.0, 0.0, 0.0, 0.0]

        else :
            control_command = [0.0, 0.0, 0.4, 0.0]

    
    return control_command # Ordered as array with: [v_forward_cmd, v_left_cmd, alt_cmd, yaw_rate_cmd]


def yaw_controller(yaw):
    wanted_yaw = 0.0
    Kp = 0.5
    error = wanted_yaw - yaw
    #integral += error

    # PID control law
    control_output = Kp * error #+ Ki * integral
    return control_output



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

def clip_angle(angle):
    angle = angle%(2*np.pi)
    if angle > np.pi:
        angle -= 2*np.pi
    if angle < -np.pi:
        angle += 2*np.pi
    return angle


def is_near_obstacle(occupancy_grid, point):
    x, y = point

    # Check adjacent cells
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            if 0 <= x + dx < len(occupancy_grid) and 0 <= y + dy < len(occupancy_grid[0]):
                if occupancy_grid[x + dx][y + dy] == 1:
                    return True

    return False



def A_Star(start, goal, h, coords, occupancy_grid, movement_type="8N"):
    """
    A* for 2D occupancy grid. Finds a path from start to goal.
    h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    :param start: start node (x, y)
    :param goal_m: goal node (x, y)
    :param occupancy_grid: the grid map
    :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """
    # get the possible movements corresponding to the selected connectivity
    if movement_type == '4N':
        movements = _get_movements_4n()
    elif movement_type == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')

    # --------------------------------------------------------------------------------------------
    # A* Algorithm implementation - feel free to change the structure / use another pseudo-code
    # --------------------------------------------------------------------------------------------

    # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
    # Initially, only the start node is known.
    openSet = [start]

    # The set of visited nodes that no longer need to be expanded.
    closedSet = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    fScore[start] = h[start]

    # while there are still elements to investigate
    while openSet != []:

        # the node in openSet having the lowest fScore[] value
        fScore_openSet = {key: val for (key, val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet

        # If the goal is reached, reconstruct and return the obtained path
        if current == goal:
            return reconstruct_path(cameFrom, current), closedSet

        openSet.remove(current)
        closedSet.append(current)

        # for each neighbor of current:
        for dx, dy, deltacost in movements:

            neighbor = (current[0]+dx, current[1]+dy)

            # if the node is not in the map, skip
            if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (neighbor[0] < 0) or (neighbor[1] < 0):
                continue

            # if the node is occupied or has already been visited, skip
            if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet):
                continue

            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + deltacost

            if neighbor not in openSet:
                openSet.append(neighbor)

            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h[neighbor]

    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], closedSet

def reconstruct_path(cameFrom, current):
    """
    Recurrently reconstructs the path from start node to the current node
    :param cameFrom: map (dictionary) containing for each node n the node immediately 
                     preceding it on the cheapest path from start to n 
                     currently known.
    :param current: current node (x, y)
    :return: list of nodes from start to current node
    """
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.insert(0, cameFrom[current])
        current = cameFrom[current]
    return total_path
