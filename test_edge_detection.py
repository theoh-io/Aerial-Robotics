#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  8 16:52:23 2022

@author: camilleguillaume
"""

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_edge(z_1,z_2):
    MIN_EDGE = 0.1  # m

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
    
    
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as mc:
            #with PositionHlCommander(scf, default_velocity=0.2, default_height=1, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
            with Multiranger(scf) as multiranger:
        
                time.sleep(4)
    
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

                mc.up(1)
                
                while keep_flying:
                    
                    z_1 = multiranger.down
                    time.sleep(0.1)
                    z_2 = multiranger.down
                    
                    if is_edge(z_1,z_2):
                        mc.stop()
                    
                    if is_close(multiranger.up):
                        keep_flying = False
                    
                    time.sleep(0.1)
        
