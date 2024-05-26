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


def return_snake(self):

    w = 10
    h = 3
    dx = 0.3
    dy = 0.3
    p = self.pos + np.array([h/2*dx, -w/2*dy])

    goal_list = []

    for i in range(h):
        for j in range(w):
            if i % 2 == 0:
                goal_list.append(p + np.array([i*dx, j*dy]))
            else:
                goal_list.append(p + np.array([i*dx, (w-j-1)*dy]))

    return goal_list

# def snake(self):

#     w = 10
#     h = 3
#     dx = 0.3
#     dy = 0.3
#     p = self.pos + np.array([h/2*dx, -w/2*dy])

#     goal_list = []

#     for i in range(h):
#         for j in range(w):
#             if i % 2 == 0:
#                 goal_list.append(p + np.array([i*dx, j*dy]))
#             else:
#                 goal_list.append(p + np.array([i*dx, (w-j-1)*dy]))

#     return goal_list


def snake():

    layers_x = 5
    layers_y = 8
    dx = 0.3
    dy = 0.3
    start_p = np.array([3.3, 0.3])

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
        sensors = ['range.front', 'range.left', 'range.back', 'range.right']
        obstacles = [self.pos + self.sensor_data[sensor] *
                     direction_vector(self.yaw + i*np.pi/2)/1000 for i, sensor in enumerate(sensors)]

        if self.state[-1] == FIND_LANDING:
            for o in obstacles:
                if np.linalg.norm(o-self.goals[0]) < 0.4:
                    return True

        return False

    def update_obstacles(self):

        sensors = ['range.front', 'range.left', 'range.back', 'range.right']
        obstacles = [self.pos + self.sensor_data[sensor] *
                     direction_vector(self.yaw + i*np.pi/2)/1000 for i, sensor in enumerate(sensors)]

        while self.goal_near():
            self.goals.pop(0)

        self.obst = sorted(np.concatenate([self.obst, obstacles], axis=0).tolist(),
                           key=lambda x: np.linalg.norm(x-self.pos))
        self.obst = np.asarray(self.obst)[0:4]  # keep only 4 nearest

    def arise(self):

        if self.height < 0.9*self.z_target:

            # v_des = self.starting_pos - self.pos
            # v = rotmat(-self.yaw) @ v_des
            z = np.clip(self.z_target, a_min=None, a_max=self.height+0.3)

            # yaw = 0

            # control_command = [v[0], v[1], z, yaw]
            control_command = [0, 0, z, 0]

            return control_command

        else:
            self.state.pop()
            return self.state_update()

    def land(self):

        if self.height > 0.01:
            speed = (self.pos_history[-1][0:2] - self.pos_history[-2][0:2])/(self.time_hist[-1]-self.time_hist[-2])

            if self.height < 0.8*self.z_target or np.linalg.norm(speed) < 0.01:
                z = self.height - 0.2
                control_command = [0, 0, z, 0]
                return control_command

            else:

                v_des = self.goals[0] - self.pos
                v = rotmat(-self.yaw) @ v_des

                control_command = [v[0], v[1], self.z_target, 0]
                return control_command
        else:

            if self.arrived:
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

            self.datapoints.append(self.time_hist[index])
            # self.datapoints.append(self.time_hist[info['right_bases'][0]])

            # self.edges.append(pos)
            # print("Edge pos:", pos)

            # self.pos_history.clear()
            # self.time_hist.clear()
            self.edges.append(pos)
            dp = self.pos - pos
            dp /= np.linalg.norm(dp)
            self.goals = [pos + 0.01*dp]

    # def wait(self, t):
    #     self.stop_time = time.time() + t
    #     self.waiting = True
    #     self.wait_pos = np.copy(self.pos)

    def find_landing(self, verbose=True):

        match len(self.edges):
            case 0:
                self.detect_edge()

                if len(self.edges) == 1:
                    dp = self.goals[0]-self.pos
                    dp /= np.linalg.norm(dp) + 0.00001  # prevent 0 division

                    self.goals = [self.pos + 0.05*dp]  # new goal 0.05 meters forward

                    if verbose:
                        print("First edge detected")
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

        if self.state[-1] == FIND_LANDING and d < 0.1 and len(self.goals) > 1:
            self.goals.pop(0)
            return self.go_to()

        force = 0.3*dp/d

        if avoid_obstacles:
            repulsion_force = self.repulsion_force(self.pos)
            force += repulsion_force


        #print("force amplitude :", force)
        # if force < 0.0001 :
        #     print("stuck")
        #     # problem if in the middle :
        #     force = (np.array([self.pos[0], 1.5]) - self.pos) / np.linalg.norm((np.array([self.pos[0], 1.5]) - self.pos))

        #     # autre méthode :
        #     if 1.5 - self.pos[0] > 0:
        #         print("stuck a droite")
        #         force =  np.linalg.norm(np.array([self.pos[0], self.pos[1] + 0.1]) - self.pos)

        #     else:
        #         print("stuck a gauche")
        #         force =  np.linalg.norm(np.array([self.pos[0], self.pos[1] - 0.1]) - self.pos)
                

        # # force = (np.array([self.pos[0], 1.5]) - self.pos) / np.linalg.norm((np.array([self.pos[0], 1.5]) - self.pos))



        # # if self.state == FIND_LANDING and self.speed_toggle:
        # if self.state == FIND_LANDING and len(self.edges):
        #     force *= 0.2/np.linalg.norm(force)

        dp = self.goals[0]-self.pos

        # speed control
        if np.linalg.norm(dp) < 0.05:
            force = dp

        v_des = force

        # v_feedback = 0.5*((self.pos_history[-1][0:2] - self.pos_history[-2][0:2]) /
        #                   (self.time_hist[-1]-self.time_hist[-2]) - v_des)

        # v = rotmat(-self.yaw) @ (v_des - v_feedback)
        v = rotmat(-self.yaw) @ v_des

        z = self.z_target

        yaw_rate = 1*np.sin(time.time())

        # yaw_rate = 1
        # if str(int(time.time()))[-1] == '0': yaw_rate = -1

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

        limx = 1/(np.abs(self.pos[0])+0.000001) - 1/(np.abs(3-self.pos[0])+0.000001)
        limy = 1/(np.abs(self.pos[1])+0.000001) - 1/(np.abs(5-self.pos[1])+0.000001)

        force += 0.01*np.array([limx, limy])

        return f
