import logging
import sys
import time
from threading import Event
import math
import numpy as np
from astar import find_path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Unit: meter
DEFAULT_HEIGHT = 0.5 #1
FOV_ZRANGER = math.radians(2.1)
BOX_LIMIT_X = 1 #5
BOX_LIMIT_Y = 0.5 #3
START_POS_X = 0
START_POS_Y = 0
GOAL_ZONE_X=0.1
START_EXPLORE_X = GOAL_ZONE_X-START_POS_X

TIME_EXPLORE = 3

RESOLUTION_GRID=20

## A* star or global nav variables
start = [START_POS_X, START_POS_Y] # to get before the start of the drone
goal = [100,100] # to get from the zranger detection, to get when landing on base done
len_x, len_y = (500, 200)

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]

def param_deck_flow(_, value_str):
    value = int(value_str) #conversion str to int
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def log_pos_callback(timestamp, data, logconf):
    #print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def zigzag_nonblocking():
    global case, x_offset 
    #to test way_back
    global start_time, goal_x, goal_y
    if case==-1:
        mc.start_forward()
        case =0
    if (case==0 and position_estimate[0]>START_EXPLORE_X) or case ==4:
        mc.start_left()
        case=1
    elif case==1 and position_estimate[1] > BOX_LIMIT_Y-START_POS_Y:
        print("!!!!!!!!!!!!reached bbox")
        mc.forward(x_offset)
        case=2
    elif case == 2:
        mc.start_right()
        case = 3
    elif case == 3 and  position_estimate[1] < -START_POS_Y:
        mc.forward(x_offset)
        case=4
    if position_estimate[0] > BOX_LIMIT_X - START_POS_X:
        mc.land()
    #just to test the way back
    if(time.time()-start_time>TIME_EXPLORE):
        mc.land()
        goal_x=position_estimate[0]
        goal_y=position_estimate[1]
        mc.take_off(DEFAULT_HEIGHT)
        case =5 #to get out of zigzag

def go_back(occupancy_grid,explored_list):
    global goal_x, goal_y
    path=find_path(start, goal, occupancy_grid, len_x+1, len_y+1, explored_list)
    mc.land()

def compute_offset():
    offset=math.tan(FOV_ZRANGER)/DEFAULT_HEIGHT
    print(offset)
    return offset

def pos_to_grid(position_estimate_x,position_etsimate_y):
    return (int(position_estimate_x/RESOLUTION_GRID), int(position_etsimate_y/RESOLUTION_GRID))

if __name__ == '__main__':

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        #want to know if the flow deck is correctly attached before flying,
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                         cb=param_deck_flow)
        time.sleep(1)
        #or
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        #start logging
        logconf.start()

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            time.sleep(1)
            #test way back
            start_time=time.time()
            print(start_time)
            goal_x=0
            goal_y=0
            occupancy_grid = np.zeros((len_x,len_y))
            explored_list = []
            case=-1
            x_offset=0.25

            while(1):
                print(time.time()-start_time)
                if case != 5:
                    if not((pos_to_grid(position_estimate[0],position_estimate[1])) in explored_list):
                        explored_list.append(pos_to_grid(position_estimate[0],position_estimate[1]))
                        
                    zigzag_nonblocking()
                else:
                    go_back()
                time.sleep(1)
                #TO BE ADDED
                #if(is_close()):
                #    obstacle avoidance();
                #if bbox intersection => case +=1

        #stop logging
        logconf.stop()

        mc.land()