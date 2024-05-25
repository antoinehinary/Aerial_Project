## AGENT
import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.colors as colors
import keyboard
import threading
from skimage.morphology import binary_dilation, square


# STATES
ARISE = 0
LAND = 1

FIND_LANDING = 2
FIND_STARTING = 3
GO_HOME = 4

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
        self.z_target = 0.3
        self.idx_goal = 0

        self.min_x, self.max_x = 0, 5.0 # meter
        self.min_y, self.max_y = 0, 3.0 # meter
        self.range_max = 2.0 # meter, maximum range of distance sensor
        self.res_pos = 0.1 # meter
        self.conf = 0.2 # certainty given by each measurement
        self.t = 0 # only for plotting
        self.map = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 = unknown, 1 = free, -1 = occupied
        self.t = 0

        self.update(sensor_data, dt)
        self.starting_pos = np.copy(self.pos)
        self.obst = [2*direction_vector(self.yaw + i*np.pi/2) for i in range(4)]
        self.prev_force = np.zeros((2,))
        self.goal_list_grid = self.snake_creation()
        print("snake creation")
        # self.start_key_listener()

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
        
    # def start_key_listener(self):
    #     def listen_for_key():
    #         while True:
    #             if keyboard.is_pressed('p'):
    #                 self.show_plot()
    #     thread = threading.Thread(target=listen_for_key)
    #     thread.daemon = True  # Allows the thread to exit when the main program exits
    #     thread.start()


    def snake_creation(self):
        ## meter :
        # goal_list = []
        # width = 2  # Width of the area in meters
        # height = 3  # Height of the area in meters
        # step_size = 0.5  # Step size in meters

        # rows = int(height / step_size)
        # cols = int(width / step_size)
        
        # for i in range(rows):
        #     if i % 2 == 0:
        #         # Move right for even rows
        #         for j in range(cols):
        #             goal_list.append((i * step_size, j * step_size))
        #     else:
        #         # Move left for odd rows
        #         for j in range(cols - 1, -1, -1):
        #             goal_list.append((i * step_size, j * step_size))

        # print("goal list :", goal_list)

        ## using occupancy_map :

        ##############################################################
        # goal_list = []
        # for i in range(37, 50, +3): 
        #     if i % 2 ==1:               # must be 4 to be correct 
        #         for j in range(2, 29, +3):
        #             goal_list.append((i,j))
        #     if i % 2 == 0:
        #         for j in range(29, 2, -3):
        #             goal_list.append((i,j))
        ################################################################

        goal_list = []
        for i in range(37, 50, +3): 
            if i % 2 ==1:               # must be 4 to be correct 
                for j in range(2, 23, +3):
                    goal_list.append((i,j))
            if i % 2 == 0:
                for j in range(23, 2, -3):
                    goal_list.append((i,j))

        #  [(0, 0.5), (0 , 1), (0.5, 1 ),(0.5, 0.5), (0.5, 0), (1, 0), (1, 0.5), (1,1), (1.5,1), (1.5, 0.5), (1.5,0)]
        
        # goal_list = [(0, 5.0), (0, 10), (5.0, 10), (5.0, 5.0), (5.0, 0), (10, 0), (10, 5.0), (10, 10), (15.0, 10), (15.0, 5.0), (15.0, 0)]

        # goal_list = [(0, 10), (5.0, 10), (5.0, 0), (10, 0), (10, 10), (15.0, 10), (15.0, 0)]

        print("goal list :", goal_list)
        return goal_list
    

    def update(self, sensor_data, dt):
            
            self.sensor_data = sensor_data
            self.dt = dt
            
            self.pos = np.array([sensor_data['stateEstimate.x'], sensor_data['stateEstimate.y']])
            
            self.height = sensor_data['stateEstimate.z']
            self.yaw = sensor_data['stabilizer.yaw']*np.pi/180
        

    def occupancy_map(self, map):
        global t
        pos_x = self.pos[0]
        pos_y = self.pos[1]
        yaw = self.sensor_data['stabilizer.yaw']
        
        for j in range(4): # 4 sensors
            yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
            if j == 0:
                measurement = self.sensor_data['range.front']/7250
            elif j == 1:
                measurement = self.sensor_data['range.left']/7250
            elif j == 2:
                measurement = self.sensor_data['range.back']/7250
            elif j == 3:
                measurement = self.sensor_data['range.right']/7250
            
            for i in range(int(self.range_max/self.res_pos)): # range is 2 meters
                dist = i*self.res_pos
                idx_x = int(np.round((pos_x - self.min_x + dist*np.cos(yaw_sensor))/self.res_pos,0))
                idx_y = int(np.round((pos_y - self.min_y + dist*np.sin(yaw_sensor))/self.res_pos,0))

                # make sure the current_setpoint is within the map
                if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > self.range_max:
                    break

                # print("dist ", dist)
                # print("measurment ", measurement)
                # update the map
                if dist < measurement:
                    # if(map[idx_x, idx_y] < -0.6):
                    #     continue
                    # else:
                    map[idx_x, idx_y] += self.conf
                else:
                    
                    # print("obstacle")
                    map[idx_x, idx_y] -= self.conf
                    break
        
        map = np.clip(map, -1, 1) # certainty can never be more than 100%

        
        if self.t % 50 == 0:
            # print(" est dedans")
            # # plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
            # plt.imshow(np.flip(self.map, 1),
            #             cmap='binary', origin='lower')
            # for cell in self.goal_list_grid:
            #     plt.plot(len(self.map[0])-1 -
            #                 cell[1], cell[0], 'o', color='orange')
            # plt.scatter([len(self.map[0])-1 - ((self.pos[1]-self.min_y)/self.res_pos) ], [(self.pos[0]- self.min_x)/self.res_pos ], color='red', marker='x', s=100) 
            # plt.colorbar(label='Binary Map Value')
            # plt.title('Binary Map')
            # plt.xlabel('X')
            # plt.ylabel('Y')
            # plt.savefig('map.png')
            # plt.close()
            plt.imshow(np.flip(self.map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
            for cell in self.goal_list_grid:
                plt.plot(len(self.map[0])-1 -
                            cell[1], cell[0], 'o', color='orange')
            plt.scatter([len(self.map[0])-1 - ((self.pos[1]-self.min_y)/self.res_pos) ], [(self.pos[0]- self.min_x)/self.res_pos ], color='red', marker='x', s=100) 
            plt.savefig("map.png")
            plt.close()
        self.t +=1

    # only plot every Nth time step (comment out if not needed)
    # flip the map at bottom left corner to match the coordinate system of matplotlib to pot
    # if t % 50 == 0:
    #     plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
    #     plt.savefig("map.png")
    #     plt.close()
    # t +=1

        return map


    # # Define the function to show the plot
    # def show_plot(self):
    #     # print("map :", self.map)
    #     plt.imshow(np.flip(self.map, 1),
    #                     cmap='binary', origin='lower')
        
    #     for cell in self.goal_list_grid:
    #         plt.plot(len(self.map[0])-1 -
    #                     cell[1], cell[0], 'o', color='orange')
    #     plt.scatter([len(self.map[0])-1 - ((self.pos[1]-self.min_y)/self.res_pos) ], [(self.pos[0]- self.min_x)/self.res_pos ], color='red', marker='x', s=100) 
    #     print(f"x :{(self.pos[0]- self.min_x)/self.res_pos}, y : {(self.pos[1]-self.min_y)/self.res_pos} ")
    #     plt.colorbar(label='Binary Map Value')
    #     plt.title('Binary Map')
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     # plt.show()
    #     plt.show() 
        


    def grid2meters(self, path):
        new_path = []
        for cell in path:
            x = round(cell[0] * self.res_pos + self.min_x, 2)
            y = round(cell[1] * self.res_pos + self.min_y, 2)
            new_path.append((x, y))
        return new_path

        
    def state_update(self):
        # self.map = self.occupancy_map(self.map)

        self.update_obstacles()
        
        if self.state == ARISE:

            control_command = self.arise()
            
        elif self.state == LAND:
            control_command = 4*[0]

        elif self.state == FIND_LANDING:
            control_command = self.find_landing()

        elif self.state == FIND_STARTING:
            control_command = 4*[0]

        elif self.state == GO_HOME:
            control_command = self.go_home()
        
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
        
        
    def find_landing(self):
        
        # self.goal = self.starting_pos + np.array([4,0])
        # self.goal = np.array([2, 0])
        self.goal_list = self.grid2meters(self.goal_list_grid)
        self.goal = self.goal_list[self.idx_goal]


        control_command = self.go_to()
        return control_command
    
    def go_to(self):
        # print("state : ", self.state)
        dp = self.goal-self.pos
        d = np.linalg.norm(dp) + 0.000000001
        # print("d : ", d)

        
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
        if self.state != GO_HOME:
            if d < 0.3 and min( self.sensor_data["range.right"] , self.sensor_data["range.left"], self.sensor_data["range.back"]) < 500:
                print("goal skiped")
                print("minuimum: ",  min( self.sensor_data["range.right"] , self.sensor_data["range.left"], self.sensor_data["range.back"]) )
                if self.sensor_data["range.front"]  < 2250:
                    print("front")
                if self.sensor_data["range.right"]  < 2250:
                    print("right")
                if self.sensor_data["range.left"]  < 2250:
                    print("left")
                if self.sensor_data["range.back"]  < 2250:
                    print("back")
                if self.idx_goal == len(self.goal_list):
                    # print("end path")
                    control_command = self.land()
                    return control_command 
                else :
                    # print(" idx increament, go to next goal ")
                    # print( " goal :", self.goal)
                    self.idx_goal += 1
                    # self.show_plot()
                    print("idx : ", self.idx_goal)
                    # print("next goal :", self.goal_list[self.idx_goal])


                    # while self.map[self.goal_list[self.idx_goal]] != 0:
                    #     self.idx_goal += 1

                    return control_command

            if d < 0.1:
                    print("goal reached")
                    if self.idx_goal == len(self.goal_list):
                        print("end path")
                        control_command = self.land()
                        return control_command 
                    else :
                        # print(" idx increament, go to next goal ")
                        # print( " goal :", self.goal)
                        self.idx_goal += 1
                        # self.show_plot()
                        print("idx : ", self.idx_goal)
                        # print("next goal :", self.goal_list[self.idx_goal])


                        # while self.map[self.goal_list[self.idx_goal]] != 0:
                        #     self.idx_goal += 1

                        return control_command

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
    
    def land(self):
        print("go on ground")
        if self.sensor_data['stateEstimate.z'] < 0.25:

            return [0.0, 0.0, 0.0, 0.0]

        else :
            return [0.0, 0.0, 0.2, 0.0] 
    
    # def force_filter(self, force):
    #     tau = 0.25
    #     filtered_force = np.copy(force)
    #     filter = 1/tau * np.ones_like(force)/( np.ones_like(force) - self.prev_force * np.exp(-self.dt*tau) )
        
        
    #     filtered_force *= filter
    #     self.prev_force = force + filtered_force
    #     return force + filtered_force
    
    def go_home(self):
        self.state = GO_HOME
        self.goal = (0,0)
        control_command = self.go_to()
        return control_command

    