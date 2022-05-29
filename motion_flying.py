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
import argparse



import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger  
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from obs_avoid import *
from drone import Drone

import edge_detection


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Unit: meter
DEFAULT_HEIGHT = 0.5 #1

FOV_ZRANGER=math.radians(2.1)

RESOLUTION_GRID=0.20 # m
MIN_DISTANCE_OCCUP_GRIG = 3  # m

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
        
        dronito.update_est_global(x, y, z, zrange, yaw)
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

# def posestimation_to_grid(position_estimate_x,position_estimate_y):
#     return (int((position_estimate_x)/RESOLUTION_GRID), int((position_estimate_y)/RESOLUTION_GRID))
#     #return (int((position_estimate_x+START_POS_X)/RESOLUTION_GRID), int((position_estimate_y+START_POS_Y)/RESOLUTION_GRID))

# def obstacle_mapping(range_left, range_right, range_front, range_back, occupancy_grid, pos_x, pos_y):

#     if(range_front < MIN_DISTANCE_OCCUP_GRIG):
#         pos_obsf=(pos_x+range_front)
#         if(pos_obsf>BOX_LIMIT_X-START_POS_X):
#             print("Obstacle mapped at front")
#             idx_x,idx_y = posestimation_to_grid(pos_obsf,pos_y)
#             occupancy_grid[idx_x,idx_y]=1
        
#     if(range_back < MIN_DISTANCE_OCCUP_GRIG):
#         pos_obsb=(pos_x-range_back)
#         if(pos_obsb<-START_POS_X):
#             print("Obstacle mapped at back")
#             idx_x,idx_y = posestimation_to_grid(pos_obsb,pos_y)
#             occupancy_grid[idx_x,idx_y]=1

#     if(range_left < MIN_DISTANCE_OCCUP_GRIG):
#         pos_obsl=pos_y+range_left
#         if(pos_obsl>BOX_LIMIT_Y-START_POS_Y):
#             print("Obstacle mapped at left")
#             idx_x,idx_y = posestimation_to_grid(pos_x,pos_obsl)
#             occupancy_grid[idx_x,idx_y]=1

#     if(range_right < MIN_DISTANCE_OCCUP_GRIG):
#         pos_obsr=pos_y-range_right
#         if(pos_obsr<-START_POS_Y):
#             print("Obstacle mapped at right")
#             idx_x, idx_y = posestimation_to_grid(pos_x,pos_obsr)
#             occupancy_grid[idx_x,idx_y]=1
            
#     return occupancy_grid

# -------------------------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------------------------
#Parser function
def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    #General Arguments
    parser.add_argument('-x', '--arenaX', default='3', type=float,
                        help='size of the arena in X')
    parser.add_argument('-y', '--arenaY', default='1.2', type=float,
                        help='size of the arena in Y')
    parser.add_argument('--startX', default='0.3', type=float,
                        help='start position in X')
    parser.add_argument('--startY', default='0.6', type=float,
                        help='start position in Y')
    parser.add_argument('--goal_zone', default='2', type=float,
                        help='goal zone: landing pad inside, to start exploring')
    parser.add_argument('--start_zone', default='1', type=float,
                        help='start zone: landing pad inseinde, exploring on way back')
    parser.add_argument('--vel_takeoff', default='0.6', type=float,
                        help='Velocity used for takeoff')
    parser.add_argument('--vel_landing', default='0.1', type=float,
                        help='velocity used for landing')

    #Obstacle Avoidance Arguments
    parser.add_argument('--vel_obst', default='0.5', type=float,
                        help='Velocity for obstacle avoidance')
    parser.add_argument('--low_vel_obst', default='0.05', type=float,
                        help='threshold pour la distance au mur')
    parser.add_argument('--thresh_y', default='0.9', type=float,
                        help='threshold pour la distance au mur')
    parser.add_argument('--min_dist', default='0.5', type=float,
                        help='min distance for an obstacle to be detected')
    parser.add_argument('--margin', default='2', type=int,
                        help='safety margin for no detection before switching obst_avoid cases')
    parser.add_argument('--small_dist', default='0.4', type=float,
                        help='Rayon de tolérance pour avoir fini de contourner l obstacle  /!\ a tune en fonction de la vitesse')
    
    #Edge Detection Arguments
    parser.add_argument('--delta_x', default='0.1', type=float,
                        help='x distance between the edge detec and the landing')
    parser.add_argument('--delta_y', default='0.25', type=float,
                        help='y distance between the edge detec and the landing')
    parser.add_argument('--vel_edge', default='0.2', type=float,
                        help='vel after platform edge det for calib of platform center')
    parser.add_argument('--vel_edge_goal', default='0.1', type=float,
                        help='vel after second platform edge det until landing')

    #ZigZag Arguments
    parser.add_argument('--x_offset', default='0.25', type=float,
                        help='offset used in zigzag')
    parser.add_argument('--box_x', default='0.2', type=float,
                        help='x dim for box used in zigzagbox')
    parser.add_argument('--box_y', default='0.2', type=float,
                        help='y dim for box used in zigzagbox')
    parser.add_argument('--time_exp', default='400', type=int,
                        help='temporary argument to end the first exploration based on timing condition')
    parser.add_argument('--time_exp_2', default='400', type=int,
                        help='temporary argument to end the second exploration based on timing condition')
    parser.add_argument('--time_exp_box', default='400', type=int,
                        help='temporary argument to end the box exploration based on timing condition')
    parser.add_argument('--thresh_back', default='0', type=float,
                        help='temporary argument to add margin to allow going out of the box compensating drift error')

    args = parser.parse_args()

    return args

def print_config(args):
    print(f"Size of the Arena: {args.arenaX}, {args.arenaY}")
    print(f"Start Position: {args.startX}, {args.startY}")
    print(f"Goal Zone: {args.goal_zone}, Start Zone: {args.start_zone}")
    print(f"takeoff: {args.vel_takeoff}, landing: {args.vel_landing}")


# Edge detection functions ---------------------------------------------------------------------------------

if __name__ == '__main__':
    args=get_args()
    print_config(args)

    # arena_dim=[args.arenaX, args.arenaY, args.goal_zone, args.start_zone]
    # start_pos=[args.startX, args.startY]
    # args_vel=[args.vel, args.vel_takeoff, args.vel_landing]
    # args_drone=[args.x_offset, args.box_x, args]
    # args_drone=[args.arenaX, args.arenaY, args.startX, args.startY, args.goal_zone, args.start_zone, args.vel, args.vel_takeoff, args.vel_landing]
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        dronito=Drone(None,args)
  
        #want to know if the flow deck is correctly attached before flying,
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                         cb=param_deck_flow)
        #or
        # if not deck_attached_event.wait(timeout=5):
        #     print('No flow deck detected!')
        #     sys.exit(1)

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
        time.sleep(1)

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            with Multiranger(scf) as multiranger:
                mc._reset_position_estimator()
                
                dronito.mc=mc

                #little sleep needed for takeoff
                time.sleep(1)
                
                #variables needed for global nav
                #len_x, len_y = (BOX_LIMIT_X, BOX_LIMIT_Y)
                # occupancy_grid = np.zeros((len_x,len_y))
                # explored_list = []

                
                #temporary intentional disturbance to regulate yaw
                #time.sleep(1)
                #mc.turn_left(7)
                #dronito.clean_takeoff()
                logs = np.zeros([100000,5])
                #parameter to run the main while
                freq_main=0.1


                while(1):
                    if is_close(multiranger.up):
                        mc.land()
                        break
                    #print(obstacle_avoidance())
                    if (obstacle_avoidance(multiranger.left, multiranger.right, multiranger.front, multiranger.back, dronito) == False):
                        #if no obstacle is being detected let zigzag manage the speeds
                        if not dronito.is_arrived():

                            # #explored list filling
                            # if not((pos_to_grid(position_estimate[0],position_estimate[1])) in explored_list):
                            #     explored_list.append(pos_to_grid(position_estimate[0],position_estimate[1]))
                            # #occupancy_grid filling
                            # occupancy_grid = obstacle_mapping(multiranger.left, multiranger.right, multiranger.front, multiranger.back, occupancy_grid, position_estimate[0], position_estimate[1])
                            #print(explored_list)
                            
                            dronito.zigzag()
                            if not dronito.is_starting():
                                [dronito.edge,dronito.x_edge,dronito.y_edge] = edge_detection.is_edge(logs,first_edge=True)
                                #dronito.edge = False ## to remove
                                if dronito.edge == True:
                                    edge_detection.find_platform_center(logs,dronito)
                                    dronito.goal_reached()
                            
                        else:
                            # print("here!!!")

                            if not dronito.is_arrived2():
                                dronito.zigzag_back()
                                
                                if not dronito.is_starting2():
                                    [dronito.edge,dronito.x_edge,dronito.y_edge] = edge_detection.is_edge(logs,first_edge=True)
                                    if dronito.edge == True:
                                        edge_detection.find_platform_center2(logs,dronito)
                                        dronito.goal_reached2()
                            else:
                                break
 
                        time.sleep(freq_main)
                    else:
                        print("obstacle av = True")
                        #obstacle detected then gives manually the speeds defined by obstacle avoidance
                        print(dronito.velocity_front, dronito.velocity_left)
                        mc.start_linear_motion(dronito.velocity_front, dronito.velocity_left, 0)
                        time.sleep(freq_main)
        #stop logging
        print("in logcong")
        logconf.stop()
        store_log_data()

        

