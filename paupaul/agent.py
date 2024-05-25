# AGENT
import numpy as np
from scipy.optimize import minimize
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


def direction_vector(theta):
    return np.array([np.cos(theta), np.sin(theta)])


def direction_ab(a, b):
    dp = b-a
    return dp/np.linalg.norm(dp)


def rotmat(theta):
    m = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    return m


def find_landing_pos(edges):

    def objective(x, points):

        distance = np.empty((len(points),))
        for i, p in enumerate(points):
            dp = p - x
            distance[i] = np.linalg.norm(dp*(1-0.15/np.linalg.norm(dp)))

        return np.sum(distance)

    # Initial guess for the point x
    x0 = np.array([0, 0])
    # Minimize the objective function
    result = minimize(objective, x0, args=(edges,))

    return result.x


class Agent():

    def __init__(self, sensor_data, dt):
        self.alive = True
        self.state = ARISE
        self.next_state = FIND_LANDING
        self.z_target = 0.6

        self.pos_history = deque(maxlen=150)
        self.time_hist = deque(maxlen=150)

        self.edges = []

        self.update(sensor_data, dt)
        self.starting_pos = np.copy(self.pos)
        self.obst = [2*direction_vector(self.yaw + i*np.pi/2) for i in range(4)]

        self.goal = np.array([4, 0])  # to replace

        self.datapoints = []

        # self.speed_toggle = False
        self.waiting = False

    def update(self, sensor_data, dt):

        self.sensor_data = sensor_data
        self.dt = dt

        self.pos = np.array([sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y']])
        self.height = sensor_data['stateEstimate.z']

        pos_plus = np.concatenate([self.pos, [self.height]])
        self.pos_history.append(pos_plus)
        self.time_hist.append(time.time())

        self.yaw = sensor_data['stabilizer.yaw']*np.pi/180

    def state_update(self):

        self.update_obstacles()

        if self.state == ARISE:
            control_command = self.arise()

        elif self.state == LAND:
            control_command = self.land()

        elif self.state == FIND_LANDING:
            control_command = self.find_landing()

        elif self.state == FIND_STARTING:
            control_command = 4*[0]

        return control_command

    def update_obstacles(self):

        sensors = ['range.front', 'range.left', 'range.back', 'range.right']
        obstacles = [self.pos + self.sensor_data[sensor] *
                     direction_vector(self.yaw + i*np.pi/2)/1000 for i, sensor in enumerate(sensors)]
        self.obst = sorted(np.concatenate([self.obst, obstacles], axis=0).tolist(),
                           key=lambda x: np.linalg.norm(x-self.pos))
        self.obst = np.asarray(self.obst)[0:4]  # keep only 4 nearest

    def arise(self):

        if self.height < self.z_target:

            v_des = self.starting_pos - self.pos
            v = rotmat(-self.yaw) @ v_des
            z = np.clip(self.z_target, a_min=None, a_max=self.height+0.3)

            yaw = 0

            # control_command = [v[0], v[1], z, yaw]
            control_command = [0, 0, z, 0]

            return control_command

        else:
            self.state = self.next_state
            return self.state_update()

    def land(self):

        if self.height > 0.05:
            # v_des = self.goal - self.pos
            # v = rotmat(-self.yaw) @ v_des

            z = np.clip(self.height - 0.1, a_min=0., a_max=None)
            # yaw = 0

            control_command = [0, 0, z, 0]

            # control_command = [v[0], v[1], z, yaw]
            return control_command

        else:
            self.alive = False
            control_command = [0, 0, 0, 0]
            return control_command

    def detect_edge(self):

        hist = np.asarray(self.pos_history)

        p, info = find_peaks(-hist[:, 2], prominence=0.05)

        if len(p) == 1:

            if len(self.edges) == 3:
                e1 = self.edges[1] - self.edges[0]
                e2 = self.edges[0] - p[0]
                sign = e1[0]*e2[1] - e1[1]*e2[0]
                if sign < 0:
                    print("False positive")
                    return

            # index = np.argmax(hist[np.clip(p[0]-50, 0, a_max=None):p[0], 2])
            index = info['left_bases'][0]
            pos = hist[index, 0:2]

            self.datapoints.append(self.time_hist[index])
            self.datapoints.append(self.time_hist[info['right_bases'][0]])

            self.edges.append(pos)
            print("Edge pos:", pos)

            np.save(os.path.join("paupaul", "logs", "edge"), hist)

            self.pos_history.clear()
            self.time_hist.clear()

    def wait(self, t):
        self.stop_time = time.time() + t
        self.waiting = True
        self.wait_pos = np.copy(self.pos)

    def find_landing(self, verbose=True):

        # if np.abs(self.sensor_data['stateEstimate.vz']) > 0.1: self.speed_toggle = True

        match len(self.edges):
            case 0:
                self.detect_edge()

                if len(self.edges) == 1:
                    dp = self.goal-self.pos
                    dp /= np.linalg.norm(dp) + 0.00001  # prevent 0 division

                    self.goal = self.pos + 2*dp  # new goal 2 meters forward

                    self.wait(2)

                    if verbose:
                        print("First edge detected")
            case 1:
                self.detect_edge()

                if len(self.edges) == 2:

                    self.goal = np.mean(self.edges, axis=0)
                    self.mean_toggle = True  # for the case 2

                    if verbose:
                        print("Second edge detected")
            case 2:
                if self.mean_toggle and np.linalg.norm(self.pos - self.goal) < 0.01:

                    # self.datapoints.append(self.pos)

                    de = self.edges[1]-self.edges[0]
                    de /= np.linalg.norm(de)
                    self.goal = np.mean(np.asarray(self.edges)[0:2], axis=0) + np.array([-de[1], de[0]])
                    self.mean_toggle = False

                    # self.datapoints.append(self.goal)

                    self.pos_history.clear()
                    self.time_hist.clear()

                    # self.wait(2)

                    if verbose:
                        print("Looking for third edge")

                elif not self.mean_toggle:

                    self.detect_edge()

                    if len(self.edges) == 3:

                        de = self.edges[1]-self.edges[0]
                        de /= np.linalg.norm(de)
                        edges = np.asarray(self.edges)
                        self.goal = np.mean(edges[0:2], axis=0) + np.array([de[1], -de[0]])

                        # self.datapoints.append(self.goal)

                        if verbose:
                            print("Third edge detected")
                            # self.datapoints.append(self.pos)

            case 3:
                self.detect_edge()

                if len(self.edges) == 4:

                    edges = np.asarray(self.edges)
                    self.goal = 0.5*(np.min(edges, axis=0) + np.max(edges, axis=0))
                    # self.speed_toggle = False

                    # self.datapoints.append(self.pos)

                    if verbose:
                        print("Fourth edge detected")

            case 4:
                if np.linalg.norm(self.pos - self.goal) < 0.03:
                    self.state = LAND
                    self.next_state = FIND_STARTING

                    if verbose:
                        print("Arrived at estimated center")

                    control_command = self.state_update()
                    return control_command
            case _:
                print("Wrong number of edges")
                self.alive = False

        if len(self.edges):
            control_command = self.go_to(avoid_obstacles=False)
        else:
            control_command = self.go_to()

        return control_command

    def go_to(self, avoid_obstacles=False):

        if self.waiting:
            if time.time() - self.stop_time > 0:
                self.waiting = False
            else:
                v_des = self.wait_pos - self.pos
                v = rotmat(-self.yaw) @ v_des
                z = self.z_target
                yaw_rate = 0.5
                control_command = [v[0], v[1], z, yaw_rate]
                return control_command

        dp = self.goal-self.pos
        d = np.linalg.norm(dp)+0.0001  # prevent 0 division

        force = 0.4*dp/d

        if avoid_obstacles:
            repulsion_force = self.repulsion_force(self.pos)
            force += repulsion_force

        # if self.state == FIND_LANDING and self.speed_toggle:
        if self.state == FIND_LANDING and len(self.edges):
            force *= 0.2/np.linalg.norm(force)

        dp = self.goal-self.pos
        d_dp = np.linalg.norm(dp) + 0.00001
        # speed control
        if np.linalg.norm(dp) < 0.05:
            # force = dp + 0.025*dp/d_dp
            force = dp

        v_des = force

        v_feedback = 0.5*((self.pos_history[-1][0:2] - self.pos_history[-2][0:2]) /
                          (self.time_hist[-1]-self.time_hist[-2]) - v_des)

        v = rotmat(-self.yaw) @ (v_des - v_feedback)
        z = self.z_target

        yaw_rate = 0.5
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

        return f
