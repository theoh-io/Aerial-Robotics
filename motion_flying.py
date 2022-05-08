import logging
import sys
import time
from threading import Event
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Unit: meter
DEFAULT_HEIGHT = 1 #1
FOV_ZRANGER=math.radians(2.1)
BOX_LIMIT_X = 1.5 #5
BOX_LIMIT_Y = 1 #3
START_POS_X = 0
START_POS_Y = 0.8
GOAL_ZONE_X=1
START_EXPLORE_X = GOAL_ZONE_X-START_POS_X
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

def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)

        #or mc.back()
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        #mc.up(0.3) # to go even higher then the default height or change default height
        time.sleep(3)
        mc.stop()

def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2

        while (1):
            if position_estimate[0] > BOX_LIMIT_X-START_POS_X:
                body_x_cmd=-max_vel
            elif position_estimate[0] < -START_POS_X:
                body_x_cmd=max_vel
            if position_estimate[1] > BOX_LIMIT_Y-START_POS_Y:
                body_y_cmd=-max_vel
            elif position_estimate[1] < -START_POS_Y:
                body_y_cmd=max_vel

            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)


def zigzag_nonblocking():
    global case, x_offset
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

def compute_offset():
    offset=math.tan(FOV_ZRANGER)/DEFAULT_HEIGHT
    print(offset)
    return offset

    
    


#def FSM():

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
            case=-1
            x_offset=0.25#compute_offset()
            #mc.start_forward()
            while(1):
                zigzag_nonblocking()
                time.sleep(1)
                #TO BE ADDED
                #if(is_close()):
                #    obstacle avoidance();
                #if bbox intersection => case +=1

        #stop logging
        logconf.stop()