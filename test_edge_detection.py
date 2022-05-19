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

def is_edge(z_1,z_2):
    MIN_EDGE = 0.05  # m

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
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    position_estimate[3] = data['range.zrange']

def _stab_log_data(timestamp, data, logconf):
    """Callback froma the log API when data arrives"""
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    
    # Save info into log variable
    for idx, i in enumerate(list(data)):
        logs[count][idx] = data[i]
    count += 1


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
        logconf.data_received_cb.add_callback(_stab_log_data)

        #start logging
        logconf.start()

        with MotionCommander(scf, default_height=1) as mc:
            #with PositionHlCommander(scf, default_velocity=0.2, default_height=1, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
            with Multiranger(scf) as multiranger:
        
                time.sleep(2)
    
                # Go to a coordinate and use default height
                #pc.go_to(0.0, 0.0)
    
                # Go to a coordinate
                #pc.go_to(1.0, 1.0, 1.0)
                # Go slowly to a coordinate
                #pc.go_to(1.0, 1.0, velocity=0.2)
    
                # Set new default velocity and height
                #pc.set_default_velocity(0.3)
                #pc.set_default_height(1.0)
                #pc.go_to(0.0, 0.0)
                
                keep_flying = True

                #mc.up(1)
                
                while keep_flying:
                    
                    z_1 = multiranger.down
                    time.sleep(0.1)
                    z_2 = multiranger.down
                    
                    if is_edge(z_1,z_2):
                        keep_flying = False
                    
                    if is_close(multiranger.up):
                        keep_flying = False
                    
                    time.sleep(0.1)

            mc.stop()

        #stop logging
        logconf.stop()

        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(),'logs',filename)
        np.savetxt(filepath, logs, delimiter=',')
