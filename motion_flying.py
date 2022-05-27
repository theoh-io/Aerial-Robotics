import logging
from pickle import FALSE
import sys
import time
from threading import Event
from threading import Timer
import math
import numpy as np
#from astar import find_path
import datetime as dt
from zipfile import ZIP_BZIP2
import os
import matplotlib.pyplot as plt


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger  
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander

from drone import Drone

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Unit: meter
DEFAULT_HEIGHT = 0.5 #1

FOV_ZRANGER=math.radians(2.1)
BOX_LIMIT_X = 2.5 #5
BOX_LIMIT_Y = 0.5 #3

START_POS_X = 0
START_POS_Y = 0
GOAL_ZONE_X= 1.5
START_EXPLORE_X = GOAL_ZONE_X-START_POS_X
THRESH_Y = 0.5
#variables needed for obstacle avoidance
VELOCITY = 0.2


TIME_EXPLORE= 15
TIME_EXPLORE= 15

RESOLUTION_GRID=0.20 # m
MIN_DISTANCE_OCCUP_GRIG = 3  # m

## A* star or global nav variables
start = [START_POS_X, START_POS_Y] 
goal = [1,1] # to get from the zranger detection, to get when landing on base done

EPSYLON=0.001

#to be added in parser
verbose = True
#state_zigzag={'start':-1, 'left':0, 'forward1':1, 'right':2, 'forward2':3, 'back2left':4, 'arrived':5}


deck_attached_event = Event()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Logs global variables
#position_estimate = [0, 0, 0, 0, 0]
logs = np.zeros([100000,5])
count = 0

# Edge detection global variables
edge = False
x_edge= 0.0
y_edge= 0.0

def param_deck_flow(_, value_str):
    value = int(value_str) #conversion str to int
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


# Logs functions -------------------------------------------------------------------------------------------
def log_pos_callback(timestamp, data, logconf):
    global dronito
    try:
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        zrange = data['range.zrange']
        yaw = data['stateEstimate.yaw']
        #add a flag to update using est2 on the way back
        
        dronito.update_est(x, y, z, zrange, yaw)
        if dronito.landed ==1:
            #print("successfully found flag")
            dronito.update_est2(x, y, z, zrange, yaw)
    except:
        pass


def stab_log_data(timestamp, data, logconf):
    """Callback from the log API when data arrives"""
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global count
    global dronito
    data_drone=[dronito.est_x, dronito.est_y, dronito.est_z, dronito.z_range, dronito.est_yaw]
    # Save info into log variable
    for idx in range(len(data_drone)):
        if idx < 3:
            logs[count][idx] = data_drone[idx]*1000
        elif idx == 4:
            #multiply yaw by 100
            logs[count][idx] = data_drone[idx]*100
        else:
            logs[count][idx] = data_drone[idx]
    count += 1

def store_log_data():
    # Get timestamp
    filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
    # Save log to file
    if not os.path.exists('logs'):
        os.makedirs('logs')
    filepath = os.path.join(os.getcwd(),'logs',filename)
    np.savetxt(filepath, logs, delimiter=',')

# -------------------------------------------------------------------------------------------------------------

def posestimation_to_grid(position_estimate_x,position_estimate_y):
    return (int((position_estimate_x)/RESOLUTION_GRID), int((position_estimate_y)/RESOLUTION_GRID))
    #return (int((position_estimate_x+START_POS_X)/RESOLUTION_GRID), int((position_estimate_y+START_POS_Y)/RESOLUTION_GRID))

def obstacle_mapping(range_left, range_right, range_front, range_back, occupancy_grid, pos_x, pos_y):

    if(range_front < MIN_DISTANCE_OCCUP_GRIG):
        pos_obsf=(pos_x+range_front)
        if(pos_obsf>BOX_LIMIT_X-START_POS_X):
            print("Obstacle mapped at front")
            idx_x,idx_y = posestimation_to_grid(pos_obsf,pos_y)
            occupancy_grid[idx_x,idx_y]=1
        
    if(range_back < MIN_DISTANCE_OCCUP_GRIG):
        pos_obsb=(pos_x-range_back)
        if(pos_obsb<-START_POS_X):
            print("Obstacle mapped at back")
            idx_x,idx_y = posestimation_to_grid(pos_obsb,pos_y)
            occupancy_grid[idx_x,idx_y]=1

    if(range_left < MIN_DISTANCE_OCCUP_GRIG):
        pos_obsl=pos_y+range_left
        if(pos_obsl>BOX_LIMIT_Y-START_POS_Y):
            print("Obstacle mapped at left")
            idx_x,idx_y = posestimation_to_grid(pos_x,pos_obsl)
            occupancy_grid[idx_x,idx_y]=1

    if(range_right < MIN_DISTANCE_OCCUP_GRIG):
        pos_obsr=pos_y-range_right
        if(pos_obsr<-START_POS_Y):
            print("Obstacle mapped at right")
            idx_x, idx_y = posestimation_to_grid(pos_x,pos_obsr)
            occupancy_grid[idx_x,idx_y]=1
            
    return occupancy_grid

# -------------------------------------------------------------------------------------------------------------
def is_close(range):
    MIN_DISTANCE = 0.5  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def  obstacle_avoid_left_right():
    global velocity_x, velocity_y, pos_estimate_before, state, first_detection, no_detection, from_left, from_right
    global dronito

    
    if (is_close(multiranger.left) & (not from_right) & case==state_zigzag['left']):
        print('state =1 left') 
        from_left = 1
        if (state==1) :
            pos_estimate_before = dronito.est_x 
            pos_estimate_before_y = dronito.est_y
        velocity_y = 0.0
        if (abs(pos_estimate_before_y - (-START_POS_Y) < THRESH_Y) or abs(pos_estimate_before_y - (BOX_LIMIT_Y -START_POS_Y) < THRESH_Y)):
            return False
        if abs(pos_estimate_before - (-START_POS_X)) > abs(pos_estimate_before - (BOX_LIMIT_X - START_POS_X)):
            velocity_x = - VELOCITY
        else :
            velocity_x = VELOCITY
        state = 2
        return True

    if (is_close(multiranger.right)  & (not from_left) & case==state_zigzag['right']):
        print('state =1 right') 
        from_right = 1
        if (state==1) :
            pos_estimate_before = dronito.est_x
            pos_estimate_before_y = dronito.est_y
        if (abs(pos_estimate_before_y - (-START_POS_Y) < THRESH_Y) or abs(pos_estimate_before_y - (BOX_LIMIT_Y -START_POS_Y) < THRESH_Y)):
            return False
        if abs(pos_estimate_before - (-START_POS_X)) > abs(pos_estimate_before - (BOX_LIMIT_X - START_POS_X)):
            velocity_x = - VELOCITY
        else :
            velocity_x = VELOCITY
        velocity_y = 0.0
        state = 2
        return True
    
    if (state == 2): #state 2
        print('state =2')
        velocity_x = 0.0
        if (from_right):
            velocity_y = -VELOCITY
            dist = -0.1
        if (from_left):
            velocity_y = +VELOCITY 
            dist = +0.1    
        if (first_detection):
            if (not(is_close(multiranger.back))): 
                no_detection = no_detection + 1
                if (no_detection >= 2): # for safety
                    state = 3
                    mc.move_distance(0, dist, 0, VELOCITY)
        if (is_close(multiranger.back)):
            first_detection =1
            velocity_x = 0.05         
        return True
        
    if (state == 3): #state 3
        print('state =3')
        velocity_y = 0
        if abs(pos_estimate_before - (-START_POS_X)) > abs(pos_estimate_before - (BOX_LIMIT_X - START_POS_X)):
            velocity_x = VELOCITY
        else :
            velocity_x = - VELOCITY
        if (dronito.est_x < abs(pos_estimate_before + 0.03)):
            print('fin state 3')
            if (is_close(multiranger.right)):
                ('right close going left')
                velocity_y = VELOCITY
                velocity_x = 0
                return True 
            elif (is_close(multiranger.left)):
                ('left close going right')
                velocity_y = -VELOCITY
                velocity_x = 0
                return True
            state = 1
            no_detection = 0
            first_detection = 0
            velocity_y = 0
            velocity_x = 0
            from_left =0
            from_right =0
            return False
        return True  #check this indent
    return False

def  obstacle_avoid_front_back():
    global velocity_x, velocity_y, pos_estimate_before, state, first_detection, no_detection, from_front, from_back
    global dronito

    if (is_close(multiranger.front) & (not from_back) & (case==state_zigzag['forward1'] or case==state_zigzag['start'] or case==state_zigzag['forward2'] )): 
        print('state =1 front')
        from_front = 1
        if (state==1) :
            pos_estimate_before = dronito.est_x
        if abs(pos_estimate_before - (-START_POS_Y)) > abs(pos_estimate_before - BOX_LIMIT_Y):
            velocity_y = - VELOCITY
        else :
            velocity_y = VELOCITY
        velocity_x = 0
        state = 2
        return True

    if (is_close(multiranger.back) & (not from_front) & 0): #jamais pour le moment
        print('state =1 back')
        from_back = 1
        if (state==1) :
            pos_estimate_before = dronito.est_y
        velocity_y = - VELOCITY
        velocity_x = 0
        state = 2
        return True
    
    if (state == 2): #state 2
        print('state =2')
        velocity_y = 0.0
        if (from_front):
            velocity_x = VELOCITY
            dist = 0.1
        if (from_back):
            velocity_x = -VELOCITY
            dist = -0.1      
        if (first_detection):
            if (not(is_close(multiranger.left))): 
                no_detection = no_detection + 1
                if (no_detection >= 2): # for safety
                    state = 3
                    mc.move_distance(dist, 0, 0, VELOCITY)
        if (is_close(multiranger.left)):
            first_detection =1
            velocity_y = - 0.05  
        return True
        
    if (state == 3): #state 3
        print('state =3')
        if abs(pos_estimate_before - (-START_POS_Y)) > abs(pos_estimate_before - BOX_LIMIT_Y):
            velocity_y = + VELOCITY
        else :
            velocity_y = - VELOCITY
        velocity_x = 0
        print(dronito.est_y)
        print(pos_estimate_before)
        if (dronito.est_y < abs(pos_estimate_before + 0.03)):
            print('fin state 3')
            if (is_close(multiranger.back)):
                velocity_y = 0
                velocity_x = VELOCITY
                return True
            elif (is_close(multiranger.front)):
                velocity_y = 0
                velocity_x = -VELOCITY
                return True
            state = 1
            no_detection = 0
            first_detection = 0
            velocity_y = 0
            velocity_x = 0
            from_front =0
            from_back =0
            return False
        return True  #check this indent
    return False

def obstacle_avoidance():
    #print(case)
#il faut penser au cas ou la vitesse n'est pas dans la direction de l'obstacle
    if ((is_close(multiranger.left) or is_close(multiranger.right) or from_left or from_right) & (from_front ==0) & (from_back == 0)):# & (case==state_zigzag["left"] or case==state_zigzag["right"])):
        return obstacle_avoid_left_right()
    elif ((is_close(multiranger.front) or is_close(multiranger.back) or from_front or from_back) & (from_right ==0) & (from_left == 0)):# & (case==state_zigzag["forward1"] or case==state_zigzag["forward2"])): 
        #print(case)
        return obstacle_avoid_front_back()
    return False


# Edge detection functions ---------------------------------------------------------------------------------

def is_edge_2():
    global logs

    MIN_EDGE2 = 50  # mm
    logs_copy2=logs[~np.all(logs == 0, axis=1)]

    if len(logs_copy2) > 100:
        #z_2=logs_copy2[-1,3]
        #z_1=logs_copy2[-50,3]
        
        z_2=np.max(logs_copy2[-100:,3])
        idx_2=np.argmax(logs_copy2[-100:,3])
        z_1=np.min(logs_copy2[-100:,3])
        idx_1=np.argmin(logs_copy2[-100:,3])
        x1=logs_copy2[len(logs_copy2)-100+idx_1,0]/1000
        y1=logs_copy2[len(logs_copy2)-100+idx_1,1]/1000

        if abs(z_1-z_2) > MIN_EDGE2:
            print('abs edge 2: ', abs(z_1-z_2))
            #print('z1: ',z_1)
            #print('z2: ',z_2)
            #print('idx_1: ',idx_1)
            #print('idx_2: ',idx_2)
            #print('x1: ',x1)
            #print('y1: ',y1)
            return True, x1, y1
        else:
            return False, 0, 0
    else:
        print('Not enough data')
        return False, 0, 0

def find_platform_center():
    global edge, case, logs, state_zigzag, x_edge, y_edge 

    x1=x_edge
    y1=y_edge
    
    """
    x1_bis=position_estimate[0]
    y1_bis=dronito.est_y

    print(x1,' ',x1_bis)
    print(y1,' ',y1_bis)

    plt.figure()
    plt.axis('equal')
    plt.scatter([x1,x1_bis],[y1,y1_bis])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x1_bis',(x1_bis,y1_bis))
    plt.savefig('first edge')
    """

    if case == state_zigzag["right"]:
        mc.right(0.25)
        time.sleep(1)

    if case == state_zigzag["left"]:
        mc.left(0.25)
        time.sleep(1)

    """
    while(edge == True):
        print('still close')
        edge= is_edge_2()[0]
    """
    
    #logs = np.zeros([100000,4])
    mc.back(0.4)
    time.sleep(1)

    if case == state_zigzag["right"]:
        mc.start_left()
        while(dronito.est_y<(y1-0.18)):
            print('going left ',dronito.est_y,' ',y1)
            continue
        x2_before=dronito.est_x
        y2_before=dronito.est_y

    if case == state_zigzag["left"]:
        mc.start_right()
        while(dronito.est_y>y1+0.18):
            print('going right ',dronito.est_y,' ',y1)
            continue
        x2_before=dronito.est_x
        y2_before=dronito.est_y

    mc.start_forward()
    print('going forward')
    edge=False

    while(edge == False):
        [edge,x_edge,y_edge]=is_edge_2()
        if (edge==True):
            """
            print('Edge 2 detected!')
            x2_bis=position_estimate[0]
            y2_bis=position_estimate[1]
            """
            x2=x_edge
            y2=y_edge

            """
            print('x1: ',x1,'x1_bis: ',x1_bis)
            print('y1: ',y1,'y1_bis: ',y1_bis)
            print('x2: ',x2,'x2_bis: ',x2_bis)
            print('y2: ',y2,'y2_bis: ',y2_bis)

            plt.figure()
            plt.axis('equal')
            plt.scatter([x1,x1_bis,x2,x2_bis],[y1,y1_bis,y2,y2_bis])
            plt.annotate('x1',(x1,y1))
            plt.annotate('x1_bis',(x1_bis,y1_bis))
            plt.annotate('x2',(x2,y2))
            plt.annotate('x2_bis',(x2_bis,y2_bis))
            plt.savefig('first edge & second edge')
            """
            
        if dronito.est_x > BOX_LIMIT_X - START_POS_X:
            print("No center found, limite arene x reached, let's land for safety")
            mc.land()
            case =state_zigzag["arrived"]
            return

    dX=0.15
    dY=0

    #if case == state_zigzag["right"]:
    #    dY=y1-y2-0.15
    #if case == state_zigzag["left"]:
    #    dY=-(y1-y2)-0.15
    
    #mc.move_distance(dX,dY,0)

    mc.forward(0.02)
    goal_x=dronito.est_x
    goal_y=dronito.est_y

    time.sleep(2)
    #default velocity 
    # VELOCITY = 0.2
    mc.land(velocity=0.1)
    case =state_zigzag["arrived"]

    x0=x2+dX
    y0=y2+dY
    print('x1: ',x1,'y1: ',y1)
    print('x2: ',x2,'y2: ',y2)
    print('x0: ',x0,'y0: ',y0)
    print('goal_x: ',goal_x,'goal_y: ',goal_y)

    plt.figure()
    plt.axis('equal')
    plt.plot(logs[:,0]/1000,logs[:,1]/1000)
    plt.scatter([x1,x2_before,x2,x0,goal_x],[y1,y2_before,y2,y0,goal_y])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x2_before',(x2_before,y2_before))
    plt.annotate('x2',(x2,y2))
    plt.annotate('x0',(x0,y0))
    plt.annotate('goal',(goal_x,goal_y))
    rectangle = plt.Rectangle((x0-0.15,y0-0.15), 0.30, 0.30,fill=None)
    plt.gca().add_patch(rectangle)
    plt.savefig('platform center')    


if __name__ == '__main__':

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # scf.cf.param.set_value('kalman.resetEstimation', '1')
        # time.sleep(0.1)
        # scf.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(2)

        #want to know if the flow deck is correctly attached before flying,
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                         cb=param_deck_flow)
        #or
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('range.zrange', 'uint16_t')
        logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.data_received_cb.add_callback(stab_log_data)

        #start logging
        logconf.start()

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            with Multiranger(scf) as multiranger:
                dronito=Drone(mc, start_x=START_POS_X, start_y=START_POS_Y, x_offset=0.25)
                #little sleep needed for takeoff
                time.sleep(1)
                
                
                #variables needed for global nav
                len_x, len_y = (BOX_LIMIT_X, BOX_LIMIT_Y)
                # occupancy_grid = np.zeros((len_x,len_y))
                # explored_list = []
                
                #variables needed for obstacle avoidance
                pos_estimate_before = 0
                velocity_y = 0
                velocity_x = 0
                state = 1
                first_detection = 0
                no_detection = 0
                from_front =0
                from_back =0
                from_left =0
                from_right =0
                
                #temporary intentional disturbance to regulate yaw
                time.sleep(1)
                mc.turn_left(7)
                dronito.clean_takeoff()
                logs = np.zeros([100000,5])
                #parameter to run the main while
                freq_main=0.1


                while(1):
                    #print(obstacle_avoidance())
                    if True:#(obstacle_avoidance() == False):
                        #if no obstacle is being detected let zigzag manage the speeds
                        if not dronito.is_arrived():

                            # #explored list filling
                            # if not((pos_to_grid(position_estimate[0],position_estimate[1])) in explored_list):
                            #     explored_list.append(pos_to_grid(position_estimate[0],position_estimate[1]))
                            # #occupancy_grid filling
                            # occupancy_grid = obstacle_mapping(multiranger.left, multiranger.right, multiranger.front, multiranger.back, occupancy_grid, position_estimate[0], position_estimate[1])
                            #print(explored_list)
                            
                            #if not dronito.is_starting():
                                #[edge,x_edge,y_edge] = is_edge_2()
                            dronito.zigzag()
                        else:
                            print("here!!!")
                            if not dronito.is_arrived2():
                                dronito.zigzag_back()
                            else:
                                break
                        
                        time.sleep(freq_main)
                    else:
                        print("obstacle av = True")
                        #obstacle detected then gives manually the speeds defined by obstacle avoidance
                        mc.start_linear_motion(velocity_x, velocity_y, 0)
                        time.sleep(0.1)
        #stop logging
        print("in logcong")
        logconf.stop()
        store_log_data()

        

