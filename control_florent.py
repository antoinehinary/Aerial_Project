# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import math

#rendu final


# Global variables
on_ground = True
height_desired = 0.6
timer = None
startpos = None
timer_done = None
phase=0

initial_x=0
initial_y=0

goal=[0,0]
ka=4#gain atractive
kr=-0.085 #gain repulsive


a_force=0
a_force_angle=0
r_force=[0,0,0,0]
r_force_angle=0

r_threshold=2

total_force_x=0
total_force_y=0
total_force_angle=0

goal_x=0
goal_y=0

yaw_gain=-0.8
angle_error=0

speed_gain=0.08

wr=0.01 #wall repulsion
kwr=0.3



exploration_map=0
exploration_goal=[]
current_goal=0


landing=[0,0]
landing_phase=0

kp=1.2 #p coefficient for landing pad finding 
last_speed=0

final_map=0

phase2_init=0
phase3_init=0
phase4_init=0

last_position=[0,0]

timer=0
# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py lines 296-323. 
# The "item" values that you can later use in the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "range_down": Downward range finder distance (Used instead of Global Z distance)
# "range_front": Front range finder distance
# "range_left": Leftward range finder distance 
# "range_right": Rightward range finder distance
# "range_back": Backward range finder distance
# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)

# This is the main function where you will implement your control algorithm
def get_command(sensor_data, camera_data, dt):
    global on_ground,timer, final_map,exploration_goal,last_position, ka, phase2_init, phase3_init, phase4_init, landing, landing_phase, last_speed,  startpos,phase2_init,kr,speed_gain, r_threshold, a_force, r_force, a_force_angle, r_force_angle, phase, total_force_x, total_force_y, total_force_angle, angle_error, speed_gain,wr, goal, kwr, initial_x, initial_y, exploration_map, current_goal

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

    
    control_command=[0,0,height_desired,0]
    on_ground = False
    map = occupancy_map(sensor_data)

#initialisation

    if phase==0: 
        x=sensor_data['x_global']
        y=sensor_data['y_global']
        initial_x=x
        initial_y=y
        phase=1
        
        
#potential field navigation
    
    if phase==1 or phase==2 or phase==3 or phase==4 or phase==5 or phase==1.5:  
        #position update
        x=sensor_data['x_global']
        y=sensor_data['y_global']
        
        if phase==1:
            goal=[4.25,2.2]
        if phase==1.5:
            goal=[4.25,0.3]
        if phase==2:
            goal=exploration_goal[current_goal]
        if phase==4 or phase==5:
            goal=[initial_x,initial_y]
            
        if phase==1000:
            goal=[0,0]
            
        x_error=goal[0]-x
        y_error=goal[1]-y
        
        
        #potential calculation
        a_force=ka

        #attractive force calculation
        
        a_force_angle=math.atan2(y_error,x_error)
        
        #repulsive force calculation
        
        total_force_x=0
        total_force_y=0
        
       
        for mx in range(50):
            for my in range(30):
                if phase!=4:
                    if map[mx,my]<0:
                        rmy=my
                        r_force_angle=math.atan2((rmy*res_pos-y),(mx*res_pos-x))
                    
                        distance=math.sqrt((mx*res_pos-x)**2+(rmy*res_pos-y)**2)
                    
                        total_force_x+=((1/(distance**1.4))*kr*math.cos(r_force_angle))
                        total_force_y+=((1/(distance**1.4))*kr*math.sin(r_force_angle))
                        
                elif phase==4:
                    if final_map[mx,my]<0:
                        rmy=my
                        r_force_angle=math.atan2((rmy*res_pos-y),(mx*res_pos-x))
                    
                        distance=math.sqrt((mx*res_pos-x)**2+(rmy*res_pos-y)**2)
                    
                        total_force_x+=((1/(distance**1.8))*kr*math.cos(r_force_angle))
                        total_force_y+=((1/(distance**1.8))*kr*math.sin(r_force_angle))

                
        #total force computation (magnitude and angle    

            
        total_force_x+=a_force*math.cos(a_force_angle)
        total_force_y+=a_force*math.sin(a_force_angle)  
       
#wall repulsion    
           
        if y<1.5:
            total_force_y+=((1/y))*kwr

        if y>1.5:
            total_force_y-=((1/(3-y)))*kwr

        if x<0.5:
            total_force_x+=((1/x))*kwr

        if x>4.5:
            total_force_x-=((1/(5-x)))*kwr

            
        
        total_force=math.sqrt((total_force_x)**2+(total_force_y)**2)
        
#poffset force if blocked
        if timer>800:
            timer=-800
        else:
            timer+=1
     
    
        if phase==2:
            position_diff=[last_position[0]-x,last_position[1]-y]
            position_diff_total=math.sqrt(position_diff[0]**2+position_diff[1]**2)
             
            if position_diff_total<0.001:
                if timer<0:
                    total_force_y+=1.5
                                     
                elif timer>0:
                    total_force_y-=1.5
                    
           

                    
#command generation
        


        if phase!=3 and phase!=5:
        
        
            control_command[0]=total_force_x*speed_gain*math.cos(sensor_data['yaw'])+total_force_y*speed_gain*math.sin(sensor_data['yaw'])
            control_command[1]=-total_force_x*speed_gain*math.sin(sensor_data['yaw'])+total_force_y*speed_gain*math.cos(sensor_data['yaw'])
            control_command[3]=0.75
            control_command[2]=0.35
            
        if phase==3:
            
            control_command[0]=(kp*x_error*math.cos(sensor_data['yaw'])+kp*y_error*math.sin(sensor_data['yaw']))
            control_command[1]=(-kp*x_error*math.sin(sensor_data['yaw'])+kp*y_error*math.cos(sensor_data['yaw']))
            
        if phase==5:
            control_command[0]=(0.4*x_error*math.cos(sensor_data['yaw'])+0.4*y_error*math.sin(sensor_data['yaw']))
            control_command[1]=(-0.4*x_error*math.sin(sensor_data['yaw'])+0.4*y_error*math.cos(sensor_data['yaw']))
            
            
        if phase==2 and phase2_init<150:
   
            control_command[0]=0
            control_command[1]=0
            phase2_init+=1

        if phase==3 and phase3_init<150:

            control_command[0]=0
            control_command[1]=0
            phase3_init+=1
            
        if phase==4 and phase4_init<400:

            control_command[0]=0
            control_command[1]=0
            control_command[2]=0
            phase4_init+=1







#state machine transition
            
        if phase==1:   
                 
             if math.sqrt((x_error**2)+(y_error**2))<0.5:
                 x_error=100
                 phase=1.5
                 
        if phase==1.5:
            if math.sqrt((x_error**2)+(y_error**2))<0.5:
            
                 phase=2
                 phase2_init=0
                 kr=-0.05

                 if phase2_init==0:
                     exploration_map=map
               
               #obstacle growth
               
                 #for ogx in range (15):
                     #for ogy in range(30):
                         #rogx=ogx+35
                         #if map[rogx,ogy]<-0: 
                             #exploration_map[rogx, ogy]=-1
                             #exploration_map[rogx, ogy-1]=-1
                             #if rogx<49:
                                 #exploration_map[rogx-1, ogy]=-1
                             #if ogy<29:
                                 #exploration_map[rogx, ogy+1]=-1
                             #if ogy>0:
                                 #exploration_map[rogx, ogy-1]=-1    
                               
                           
                                    
                           
               #map build
                  
 
                 for gy in range(30):
                     if gy%3!=0:
                         continue
                     if gy==0:
                         continue
                     if gy%6==0:
                         for gx in range(14,-1,-1):
                             rgx=gx+35
                             if gx%2!=0 or gx==4.9:
                                 continue
                             if exploration_map[rgx,gy]>0:
                                 exploration_goal.append([rgx*res_pos,gy*res_pos])
                     else:
                         for gx in range(15):
                             rgx=gx+35
                             if gx%2!=0 or gx==4.9:
                                 continue
                             if exploration_map[rgx,gy]>0:
                                 exploration_goal.append([rgx*res_pos,gy*res_pos])
                 exploration_goal.append([1000,1000])
                 exploration_goal.append([1000,1000])
                 exploration_goal.append([1000,1000])
                 exploration_goal.append([1000,1000])





        if phase==2:   
       
                         
            if math.sqrt((x_error**2)+(y_error**2))<0.225:
                current_goal+=1
                
            if exploration_goal[current_goal]==[1000,1000]:
                exploration_goal=[]
                exploration_map=map
                current_goal=0
                
            for gy in range(30):
                 if gy%3!=0:
                     continue
                 if gy==0:
                     continue
                 if gy%6==0:
                     for gx in range(14,-1,-1):
                        rgx=gx+35
                        if gx%2!=0 or gx==4.9:
                            continue
                        if exploration_map[rgx,gy]>0:
                            exploration_goal.append([rgx*res_pos,gy*res_pos])
                 else:
                     for gx in range(15):
                         rgx=gx+35
                         if gx%2!=0 or gx==4.9:
                             continue
                         if exploration_map[rgx,gy]>0:
                             exploration_goal.append([rgx*res_pos,gy*res_pos])
                  
                

                
          
         
         
           
        if sensor_data['z_global'] - sensor_data['range_down']>0.05 and x>3.5 and (phase==2 or phase==1 or phase==1.5):
            landing=[x,y]
            goal=[landing[0]-0.1,landing[1]-0.1]
            kr=-0.075
            phase=3
            landing_phase=0
            final_map=map


        if phase==3:
          
            if math.sqrt((x_error**2)+(y_error**2))<0.01 and landing_phase==3:
                if sensor_data['z_global']-height_desired>0.05:
                    phase=4
                    ka=4
                    control_command[2]=0.0
                else:
                    print("c'est la merde")
                    
                
            if math.sqrt((x_error**2)+(y_error**2))<0.01 and landing_phase==2:
                if sensor_data['z_global']-height_desired>0.05:
                    phase=4
                    ka=4
                    control_command[2]=0.0
                else:
                    goal=[landing[0]-0.1,landing[1]+0.1]
                    landing_phase=3
                    print(landing_phase)

            if math.sqrt((x_error**2)+(y_error**2))<0.01 and landing_phase==1:
                if sensor_data['z_global']-height_desired>0.05:
                    phase=4
                    ka=4
                    control_command[2]=0.0
                else:
                    goal=[landing[0]+0.1,landing[1]+0.1]
                    landing_phase=2
                    print(landing_phase)
                    
            if math.sqrt((x_error**2)+(y_error**2))<0.01 and landing_phase==0:
                if sensor_data['z_global']-height_desired>0.05:
                    phase=4
                    ka=4
                    control_command[2]=0.0
                else:
                    goal=[landing[0]+0.1,landing[1]-0.1]
                    landing_phase=1
                    print(landing_phase)
                

             
                    
           
        
        

            

        if math.sqrt((x_error**2)+(y_error**2))<0.5 and phase==4 and x<1.5:
            speed_gain=0.025
            phase=5

        if math.sqrt((x_error**2)+(y_error**2))<0.05 and phase==5:
            speed_gain=0
            control_command[2]=0.01
                

     
    
    last_position=[x,y]
    

    return control_command # Ordered as array with: [v_forward_cmd, v_left_cmd, alt_cmd, yaw_rate_cmd]


def signe(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0





































# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 5.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.1 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
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
    #if t % 50 == 0:
        #plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        #plt.savefig("map.png")
        #plt.close()
    #t +=1

    return map


# Control from the exercises
index_current_setpoint = 0
def path_to_setpoint(path,sensor_data,dt):
    global on_ground, height_desired, index_current_setpoint, timer, timer_done, startpos

    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]    
    if on_ground and sensor_data['range_down'] < 0.49:
        current_setpoint = [startpos[0], startpos[1], height_desired, 0.0]
        return current_setpoint
    else:
        on_ground = False

    # Start timer
    if (index_current_setpoint == 1) & (timer is None):
        timer = 0
        print("Time recording started")
    if timer is not None:
        timer += dt
    # Hover at the final setpoint
    if index_current_setpoint == len(path):
        # Uncomment for KF
        control_command = [startpos[0], startpos[1], startpos[2]-0.05, 0.0]

        if timer_done is None:
            timer_done = True
            print("Path planing took " + str(np.round(timer,1)) + " [s]")
        return control_command

    # Get the goal position and drone position
    current_setpoint = path[index_current_setpoint]
    x_drone, y_drone, z_drone, yaw_drone = sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down'], sensor_data['yaw']
    distance_drone_to_goal = np.linalg.norm([current_setpoint[0] - x_drone, current_setpoint[1] - y_drone, current_setpoint[2] - z_drone, clip_angle(current_setpoint[3]) - clip_angle(yaw_drone)])

    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    if distance_drone_to_goal < 0.1:
        # Select the next setpoint as the goal position
        index_current_setpoint += 1
        # Hover at the final setpoint
        if index_current_setpoint == len(path):
            current_setpoint = [0.0, 0.0, height_desired, 0.0]
            return current_setpoint

    return current_setpoint

def clip_angle(angle):
    angle = angle%(2*np.pi)
    if angle > np.pi:
        angle -= 2*np.pi
    if angle < -np.pi:
        angle += 2*np.pi
    return angle
