## AGENT
import numpy as np
from scipy.optimize import minimize

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
        
        self.update(sensor_data, dt)
        self.starting_pos = np.copy(self.pos)
        self.obst = [2*direction_vector(self.yaw + i*np.pi/2) for i in range(4)]
        
        self.goal = np.array([4, 0]) # to replace
        
        self.edges = []
        # self.commander_busy = False
       
    def update(self, sensor_data, dt):
        
        self.sensor_data = sensor_data
        self.dt = dt
        
        self.pos = np.array([sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y']])
        
        self.height = sensor_data['stateEstimate.z']
        self.yaw = sensor_data['stabilizer.yaw']*np.pi/180
        
    def state_update(self):
        
        self.update_obstacles()
        
        if self.state == ARISE:
            control_command = self.arise()
            
        elif self.state == LAND:
            control_command = 4*[0]

        elif self.state == FIND_LANDING:
            control_command = self.find_landing()

        elif self.state == FIND_STARTING:
            control_command = 4*[0]
        
        return control_command
    
    def update_obstacles(self):

        sensors = ['range.front', 'range.left', 'range.back', 'range.right']
        obstacles = [self.pos + self.sensor_data[sensor]*direction_vector(self.yaw + i*np.pi/2)/1000 for i, sensor in enumerate(sensors)]
        self.obst = sorted(np.concatenate([self.obst, obstacles], axis=0).tolist(), key=lambda x: np.linalg.norm(x-self.pos))
        self.obst = np.asarray(self.obst)[0:4] # keep only 4 nearest
        
    def arise(self):
        
        if self.height < self.z_target:
            v_des = self.starting_pos - self.pos
            v = rotmat(-self.yaw) @ v_des
            z = np.clip(self.z_target, a_min=None, a_max=self.height+0.3)
            yaw = np.clip(self.height, a_min=0, a_max=0.5)
            
            control_command = [v[0], v[1], z, yaw]
            return control_command
        
        else:
            self.state = FIND_LANDING
            self.commander_busy = False
            # if self.state == FIND_LANDING:
            #     self.state = LAND
            #     self.next_state = LAND
            #     self.events.append([""])
            # else:
            #     self.state = np.copy(self.next_state)
            #     self.next_state = FIND_STARTING
            return self.state_update()
        
    def find_landing(self, verbose=True):
        
        match len(self.edges):
            case 0:
                if self.sensor_data['stateEstimate.vz'] > 0.1:
                    self.edges.append(self.pos)
                    ## continue in the same direction
                    dp = self.goal-self.pos
                    dp /= np.linalg.norm(dp)
                    self.goal = self.pos + 1.7*0.3*dp
                    
                    # self.z_target -= 0.15
                    if verbose: print("First edge detected")
            case 1:
                if self.sensor_data['stateEstimate.vz'] < -0.1:
                    self.edges.append(self.pos)
                    self.goal = np.mean(self.edges, axis=0)
                    self.mean_toggle = True # for the case 2

                    if verbose: print("Second edge detected")

            case 2:
                if self.mean_toggle and np.linalg.norm(self.pos - self.goal) < 0.02:
                    de = self.edges[1]-self.edges[0]
                    de /= np.linalg.norm(de)
                    self.goal = self.pos + 1.5*0.3*np.array(-de[1], de[0])
                    self.mean_toggle = False
                    
                    if verbose: print("Looking for third edge")

                elif not self.mean_toggle and self.sensor_data['stateEstimate.vz'] < -0.1:
                    self.edges.append(self.pos)
                    self.goal = find_landing_pos(self.edges)
                    
                    if verbose: print("Third edge detected")
                    
            case 3:
                if np.linalg.norm(self.pos - self.goal) < 0.02:
                    self.alive = False
                    control_command = [0, 0, 0.6, 0] # last command
                    
                    if verbose: print("Arrived at pseudo-center")
                    
                    return control_command
                
        if len(self.edges):
            control_command = self.go_to(avoid_obstacles=False)
        else:
            control_command = self.go_to()

        return control_command
    
    def go_to(self, avoid_obstacles=False):
        
        dp = self.goal-self.pos
        d = np.linalg.norm(dp)
       
        force = 0.4*dp/d
        
        if avoid_obstacles:
            repulsion_force = self.repulsion_force(self.pos) 
            force += repulsion_force

        d_min = np.min([d, np.linalg.norm(force), np.linalg.norm(self.obst[0]-self.pos)])
        v_des = force * np.clip(d_min/0.3, 0, 1)
        
        ## reduce the speed if trying to find edges
        if self.state == FIND_LANDING and len(self.edges): force = 0.05*dp/d
        
        v = rotmat(-self.yaw) @ v_des
        
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