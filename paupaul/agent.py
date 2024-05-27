# AGENT
import numpy as np
# from simple_pid import PID
from collections import deque
from scipy.signal import find_peaks
import os
import time

# STATES
ARISE = 0
LAND = 1

FIND_LANDING = 2
FIND_STARTING = 3

def trapezoidal_waveform(t, T=6, Tr=1, Tf=1, Th=2, Tl=2, H=1, L=-1):
    """
    Generate a trapezoidal waveform value at time t.

    Parameters:
    t (float): The time at which to evaluate the waveform.
    T (float): The period of the waveform.
    Tr (float): The rise time.
    Tf (float): The fall time.
    Th (float): The high level dwell time.
    Tl (float): The low level dwell time.
    H (float): The high level of the waveform.
    L (float): The low level of the waveform.

    Returns:
    float: The waveform value at time t.
    """
    # Normalize time to the period
    t_mod = t % T

    # Determine the value of the waveform based on the phase within the period
    if 0 <= t_mod < Tr:
        # Rising edge
        return L + (H - L) * (t_mod / Tr)
    elif Tr <= t_mod < Tr + Th:
        # High level
        return H
    elif Tr + Th <= t_mod < Tr + Th + Tf:
        # Falling edge
        return H - (H - L) * ((t_mod - Tr - Th) / Tf)
    elif Tr + Th + Tf <= t_mod < T:
        # Low level
        return L
    else:
        # This should not happen
        return L



def snake():

    layers_x = 4
    layers_y = 9
    dx = 0.3
    dy = 0.3
    start_p = np.array([3.7, 0.25])

    goal_list = []

    for i in range(layers_x):
        for j in range(layers_y):
            if i % 2 == 0:
                goal_list.append(start_p + np.array([i*dx, j*dy]))
            else:
                goal_list.append(start_p + np.array([i*dx, (layers_y-j-1)*dy]))

    return goal_list


def direction_vector(theta):
    return np.array([np.cos(theta), np.sin(theta)])


def direction_ab(a, b):
    dp = b-a
    return dp/np.linalg.norm(dp)


def rotmat(theta):
    m = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    return m


class Agent():

    def __init__(self, sensor_data, start_pos):
        self.alive = True
        self.state = [LAND, FIND_STARTING, ARISE, LAND, FIND_LANDING, ARISE]
        self.z_target = 0.4

        self.pos_history = deque(maxlen=150)
        self.time_hist = deque(maxlen=150)

        self.edges = []
        self.starting_pos = np.copy(start_pos)

        self.update(sensor_data)
        self.obst = [2*direction_vector(self.yaw + i*np.pi/2) for i in range(4)]

        self.goals = snake()

        self.datapoints = []
        self.arrived = False


        self.case = 'turning_right' 
        self.computed_yaw_rate = -0.5



        # self.speed_toggle = False
        # self.waiting = False

    def update(self, sensor_data):

        self.sensor_data = sensor_data

        self.pos = self.starting_pos + np.array([sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y']])
        self.height = sensor_data['stateEstimate.z']

        pos_plus = np.concatenate([self.pos, [self.height]])
        self.pos_history.append(pos_plus)
        self.time_hist.append(time.time())

        self.yaw = sensor_data['stabilizer.yaw']*np.pi/180

    def state_update(self):

        self.update_obstacles()

        if self.state[-1] == ARISE:
            control_command = self.arise()

        elif self.state[-1] == LAND:
            control_command = self.land()

        elif self.state[-1] == FIND_LANDING:
            control_command = self.find_landing()

        elif self.state[-1] == FIND_STARTING:
            control_command = self.find_starting()

        return control_command

    def find_starting(self):

        self.goals = [self.starting_pos]

        if np.linalg.norm(self.pos-self.starting_pos) < 0.02:
            self.state.pop()
            self.arrived = True
            return self.state_update()
        else:
            return self.go_to()

    def goal_near(self):
        # checks if the current goal (self.goals[0]) is near an obstacle

        sensors = ['range.front', 'range.left', 'range.back', 'range.right']
        obstacles = [self.pos + self.sensor_data[sensor] *
                     direction_vector(self.yaw + i*np.pi/2)/1000 for i, sensor in enumerate(sensors)]

        for o in obstacles:
            # if the goal is less than 0.4 away from an obstacle and this obstacle is within 1m of the robol (to avoid pbm with far obstacles)
            if np.linalg.norm(o-self.goals[0]) < 0.4 and np.linalg.norm(o-self.pos) < 1.5:
                # if np.linalg.norm(o-self.goals[0]) < 0.4 :
                return True

        return False

    def update_obstacles(self):

        sensors = ['range.front', 'range.left', 'range.back', 'range.right']
        obstacles = [self.pos + self.sensor_data[sensor] *
                     direction_vector(self.yaw + i*np.pi/2)/1000 for i, sensor in enumerate(sensors)]

        self.obst = sorted(np.concatenate([self.obst, obstacles], axis=0).tolist(),
                           key=lambda x: np.linalg.norm(x-self.pos))
        self.obst = np.asarray(self.obst)[0:4]  # keep only 4 nearest

    def arise(self):

        if self.height < 0.9*self.z_target:

            z = np.clip(self.z_target, a_min=None, a_max=self.height+0.3)
            control_command = [0, 0, z, 0]

            return control_command

        else:
            self.state.pop()
            return self.state_update()

    def land(self):

        if self.height > 0.01:
            speed = (self.pos_history[-1][0:2] - self.pos_history[-2][0:2])/(self.time_hist[-1]-self.time_hist[-2])

            if self.height < 0.8*self.z_target or np.linalg.norm(speed) < 0.01:
                z = self.height - 0.25
                control_command = [0, 0, z, 0]
                return control_command

            else:

                v_des = self.goals[0] - self.pos
                v = rotmat(-self.yaw) @ v_des

                control_command = [v[0], v[1], self.z_target, 0]
                return control_command
        else:

            if self.arrived:
                print("Arrived at destination")
                self.alive = False
                return [0, 0, 0, 0]
            else:
                self.state.pop()
                print(self.state[-1])
                return self.state_update()

    def detect_edge(self):

        if np.linalg.norm(self.pos-self.starting_pos) < 1:
            return

        hist = np.asarray(self.pos_history)

        p, info = find_peaks(-hist[:, 2], prominence=0.05)

        if len(p) == 1:

            index = info['left_bases'][0]
            pos = hist[index, 0:2]

            if not len(self.edges) and np.linalg.norm(self.starting_pos-pos) < 1:
                return

            self.datapoints.append(self.time_hist[index])
            # self.datapoints.append(self.time_hist[info['right_bases'][0]])

            # self.edges.append(pos)
            # print("Edge pos:", pos)

            # self.pos_history.clear()
            # self.time_hist.clear()

            self.edges.append(pos)
            dp = self.pos - pos
            dp /= np.linalg.norm(dp)

            self.goals = [pos + 0.1*dp]
            # self.goals = [self.pos]

            # self.goals = [pos + 0.0*dp]

    def find_landing(self, verbose=True):

        # eliminate current goal if near obstacle
        while self.goal_near() and not len(self.edges):
            if len(self.goals) == 1:
                print("Arrived at the end without seeing the pad")
                self.alive = False
                return [0, 0, 0, 0]
            else:
                print("Obstacle near goal:", self.goals[0])
                self.goals.pop(0)

        # eliminate current goal if robot is near the goal
        if np.linalg.norm(self.pos - self.goals[0]) < 0.1 and not len(self.edges):
            if len(self.goals) == 1:
                print("Arrived at the end without seeing the pad")
                self.alive = False
                return [0, 0, 0, 0]
            else:
                print("Robot near goal:", self.goals[0])
                self.goals.pop(0)

        match len(self.edges):
            case 0:
                self.detect_edge()

                # if len(self.edges) == 1:
                #     dp = self.goals[0]-self.pos
                #     dp /= np.linalg.norm(dp) + 0.00001  # prevent 0 division

                #     # self.goals = [self.pos + 0.05*dp]  # new goal 0.05 meters forward

                #     if verbose:
                #         print("First edge detected")
            case 1:
                if np.linalg.norm(self.goals[0]-self.pos) < 0.02:
                    self.state.pop()
                    print(self.state[-1])
                    return self.state_update()

        if len(self.edges):
            control_command = self.go_to(avoid_obstacles=False)
        else:
            control_command = self.go_to()

        return control_command

    def go_to(self, avoid_obstacles=True):

        dp = self.goals[0]-self.pos
        d = np.linalg.norm(dp)+0.0001  # prevent 0 division

        force = 0.35*dp/d

        if avoid_obstacles:
            repulsion_force = self.repulsion_force(self.pos)
            force += repulsion_force

        dp = self.goals[0]-self.pos

        # speed control near goal
        if np.linalg.norm(dp) < 0.05:
            force = dp

        v = (self.pos_history[-1][0:2] - self.pos_history[-2][0:2])/(self.time_hist[-1]-self.time_hist[-2])

        if np.linalg.norm(v) > 0.35:
            force -= 0.5*(v - 0.35*force/np.linalg.norm(force))
            # force -= 2*(v - force)
            # print("Speed limited")

        # if self.pos[1] < 0 and avoid_obstacles: 
        #     print("rep", repulsion_force[1])
        #     print("force", force[1])


        v_des = force

        v = rotmat(-self.yaw) @ v_des

        z = self.z_target

        # yaw_rate = 1*np.sin(time.time())
        # yaw_rate = 1.0*np.sin(time.time())
        # yaw_rate = 1*np.sin(time.time()/2)


        # if self.case == 'turning_right' :
        #     self.computed_yaw_rate = -0.5
        #     if(self.sensor_data['stabilizer.yaw'] > 20): #30 degrees
        #         self.computed_yaw_rate = 0.5
        #         self.case = 'turning_left'
                
        # elif self.case == 'turning_left' :
        #     self.computed_yaw_rate = 0.5
        #     if(self.sensor_data['stabilizer.yaw'] < -20): #-30 degrees
        #         self.computed_yaw_rate = -0.5
        #         self.case = 'turning_right'
        
     

        # # switch alternative
        # yaw_rate = 1
        # if time.time() % 6 >= 3: yaw_rate *= -1

        yaw_rate = trapezoidal_waveform(time.time())

        control_command = [v[0], v[1], z, yaw_rate]

        return control_command

    def repulsion_force(self, pos):

        rep_const = 0.02
        order = 1

        f = np.zeros((2,))

        for obstacle in self.obst:
            do = obstacle - pos
            d = np.linalg.norm(do)

            force = rep_const/d**order

            f -= force * (do/d)

        limx = 1/(np.abs(self.pos[0])+0.00000001) - 1/(np.abs(5-self.pos[0])+0.00000001)
        limy = 1/(np.abs(self.pos[1])+0.00000001) - 1/(np.abs(3-self.pos[1])+0.00000001)

        
        border_force = rep_const*np.array([limx, limy])

        # print(limy)
        # print(border_force[1])
        f += border_force

        return f
