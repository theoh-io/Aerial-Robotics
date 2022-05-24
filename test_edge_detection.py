#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  8 16:52:23 2022

@author: camilleguillaume
"""

import logging
import time
from threading import Event
from threading import Timer
import math
import datetime as dt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig

import numpy as np
import os



URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

deck_attached_event = Event()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0, 0, 0]
logs = np.zeros([100000,4])
count = 0

edge = False

def is_edge(z_1,z_2):
    MIN_EDGE = 0.05  # m
    print(abs(z_1-z_2))

    if abs(z_1-z_2) > MIN_EDGE :
        return True
    else:
        return False


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def log_pos_callback(timestamp, data, logconf):
    #print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    position_estimate[3] = data['range.zrange']

def stab_log_data(timestamp, data, logconf):
    """Callback froma the log API when data arrives"""
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global count
    
    # Save info into log variable
    for idx, i in enumerate(list(data)):
        #print('enumerate ', enumerate(list(data)))
        if idx < 3:
            logs[count][idx] = data[i]*1000
        else:
            logs[count][idx] = data[i]
        #print(logs[count][idx])
    count += 1
    
    #print(count)

case = 1 
# -1 start 
# 0 left    zizag
# 1 forward1 offset 
# 2 right   zizag
# 3 forward2 offset


def edge_detection(timestamp, data, logconf):
    global edge
    #print('function edge')
    z_1 = multiranger.down
    time.sleep(0.2)
    z_2 = multiranger.down
    
    if is_edge(z_1,z_2) :
        edge = True
        print('Edge detected')
    else:
        edge = False

    if is_close(multiranger.up):
        print('Stop by hand')
        mc.land()

def is_edge_2():
    MIN_EDGE2 = 50  # mm
    #print("function is edge 2")
    logs_copy2=logs[~np.all(logs == 0, axis=1)]
    #print(len(logs_copy2))

    if len(logs_copy2) > 50:
        z_2=logs_copy2[-1,3]
        #print(z_2)
        z_1=logs_copy2[-50,3]
        #print(z_2)

        if abs(z_1-z_2) > MIN_EDGE2 :
            print(abs(z_1-z_2))
            return True
        else:
            return False
    else:
        return False

def edge_detection_2(timestamp, data, logconf):
    global edge
    #print('function edge 2')
    
    if is_edge_2():
        edge = True
        print('------ Edge detected 2')
    else:
        edge = False





if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        #want to know if the flow deck is correctly attached before flying,
        #scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
        #                                 cb=param_deck_flow)
        time.sleep(1)
        #or
        #if not deck_attached_event.wait(timeout=5):
        #   print('No flow deck detected!')
        #  sys.exit(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('range.zrange', 'uint16_t')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.data_received_cb.add_callback(stab_log_data)

        #start logging
        logconf.start()

        with MotionCommander(scf, default_height=0.5) as mc:
            #with PositionHlCommander(scf, default_velocity=0.2, default_height=1, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
            with Multiranger(scf) as multiranger:

                time.sleep(2)
                #logconf.data_received_cb.add_callback(edge_detection)
                mc.forward(0.5)
                #logconf.data_received_cb.add_callback(edge_detection_2)
                
                mc.start_forward()
                loop = True
                while(loop):
                    edge = is_edge_2()
                    if edge == True:
                        loop = False
                        if case == 0:
                            #left
                            #mc.left(0.15)
                            mc.land()
                        if case == 1:
                            #forward1
                            #print('------------- forward 15cm 2')
                            #mc.forward(0.15)
                            print('------------- Land 2')
                            mc.land()
                        if case == 2:
                            #right
                            #mc.right(0.15)
                            mc.land()
                        if case == 3:
                            #forward2
                            #mc.forward(0.15)
                            mc.land()

            #mc.land()

        #stop logging
        logconf.stop()

        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(),'logs',filename)
        np.savetxt(filepath, logs, delimiter=',')

