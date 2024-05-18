## AGENT
import numpy as np

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

class Agent():
    
    # self._lg_stab.add_variable('stateEstimate.x', 'float')
    # self._lg_stab.add_variable('stateEstimate.y', 'float')
    # self._lg_stab.add_variable('stateEstimate.z', 'float')
    # self._lg_stab.add_variable('stabilizer.yaw', 'float')
    
    def __init__(self, sensor_data, dt):
        self.alive = True
        self.state = ARISE
        self.next_state = FIND_LANDING
        self.z_target = 0.6
        self.idx_goal = 0

        
        self.update(sensor_data, dt)
        self.starting_pos = np.copy(self.pos)
        self.obst = [2*direction_vector(self.yaw + i*np.pi/2) for i in range(4)]
        self.prev_force = np.zeros((2,))
        self.goal_list = self.snake_creation()
        print("snake creation")
    

    def snake_creation(self):
        goal_list = []
        # for i in range(10, 30, +3): 
        #     # if i % 2 ==1:               # must be 4 to be correct 
        #     #     for j in range(2, 28, +3):
        #     #         goal_list.append((i,j))
        #     # if i % 2 == 0:
        #     #     for j in range(29, 2, -3):
        #     #         goal_list.append((i,j))
        #     if i % 2 ==1:               # must be 4 to be correct 
        #         for j in range(2, 8, +3):
        #             goal_list.append((i,j))
        #     if i % 2 == 0:
        #         for j in range(9, 2, -3):
        #             goal_list.append((i,j))

        goal_list.append((1,1))
        goal_list.append((1,1.3))
        goal_list.append((1,1.6))
        goal_list.append((1,1.9))
        goal_list.append((2,1.9))
        goal_list.append((2,1.6))
        goal_list.append((2,1.3))
        goal_list.append((2,1))
        print("goal list :", goal_list)


        return goal_list


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
            if self.state == FIND_LANDING:
                self.state = self.next_state
                self.next_state = LAND
            else:
                self.state = self.next_state
                self.next_state = FIND_STARTING
            return self.state_update()
        
    # def send_velocity_world_setpoint(self, vx, vy, vz, yawrate):
    #     """
    #     Send Velocity in the world frame of reference setpoint with yawrate commands

    #     vx, vy, vz are in m/s
    #     yawrate is in degrees/s

        
    def find_landing(self):
        
        # self.goal = self.starting_pos + np.array([4,0])
        # self.goal = np.array([2, 0])
        self.goal = self.goal_list[self.idx_goal]


        control_command = self.go_to()
        return control_command
    
    def go_to(self):
        
        dp = self.goal-self.pos
        d = np.linalg.norm(dp)
        # print("d : ", d)

        if d < 0.1:
            self.idx_goal += 1
            print(" idx increament, go to next goal ")
            print( " goal :", self.goal)
            print("next goal :", self.goal[self.idx_goal])
            print("state : ", self.state)
            if self.idx_goal == len(self.goal_list):
                print("end path")
                return control_command

            # else :
            #     self.state = FIND_LANDING
            #     [0.0, 0.0, self.z_target, 0]
            #     print(" is landing :)")
            #     return control_command
            
        # obstacle dans pt :   
        # if d < 0.4 and 

        force = 0.4*dp/d
        repulsion_force = self.repulsion_force(self.pos)
        
        # repulsion_force_prime = self.repulsion_force(self.pos+self.speed*self.dt)   
        # df = (repulsion_force_prime-repulsion_force)/self.dt

        # print("distances:", [np.linalg.norm(x-self.pos) for x in self.obst])
        # print(force, repulsion_force)
        
        force += repulsion_force
        
        # print("repulsion force", repulsion_force)
        # force -= df
        
        # v_des = self.force_filter(force)
        d_min = np.min([d, np.linalg.norm(force), np.linalg.norm(self.obst[0]-self.pos)])
        v_des = force * np.clip(d_min/0.3, 0, 1)
        v = rotmat(-self.yaw) @ v_des
        # print("d_min: ",d_min)


        z = self.z_target
        # yaw = np.clip(self.height, a_min=0, a_max=0.5)
        yaw_rate = 0.5
        control_command = [v[0], v[1], z, yaw_rate]
                       # roll/pitch/yaw_Rate/thrust
 
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
    
    # def force_filter(self, force):
    #     tau = 0.25
    #     filtered_force = np.copy(force)
    #     filter = 1/tau * np.ones_like(force)/( np.ones_like(force) - self.prev_force * np.exp(-self.dt*tau) )
        
        
    #     filtered_force *= filter
    #     self.prev_force = force + filtered_force
    #     return force + filtered_force