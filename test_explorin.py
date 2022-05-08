import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

DEFAULT_HEIGHT = 0.2
BOX_LIMIT = 0.2

#test comment

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
            if position_estimate[0] > BOX_LIMIT:
                 body_x_cmd=-max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd=max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd=-max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd=max_vel

            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)

# def sidestep(direction, offset):
#     if direction = 'x':
#         while(position_estimate[0]<)
###############
# Class Drone #
class Drone():
    def __init__(self, x_pos, y_pos, x_vel, y_vel):
        self.x_pos=x_pos
        self.y_pos=y_pos
        self.x_vel=x_vel
        self.y_vel=y_vel
        #Flags for limits being reached

    def check_bbox():
        print("checking position wrt to bbox")
    
    def offset_left():
        #move to a desired y position
    def zigzag():
        self.check_bbox()
        #modify the velocity command

    def send_commands():
        print("actually sending commands")



def zigzag(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        
        body_x_cmd = 0.2
        body_y_cmd = 0
        max_vel = 0.2

        start_y=0
        y_offset=0.5
        x_reached=False
        #zigzag in x when reach limit of bounding box go y +5 and then back
        while (1):
            if position_estimate[0] > BOX_LIMIT and x_reached is False:
                # y_pos_current=position_estimate[1]
                # while(position_estimate[1]<y_pos_current+y_offset and not position_estimate[1] > BOX_LIMIT ):
                body_x_cmd=0
                body_y_cmd=max_vel
                x_reached=True
                y_reached=False
                start_y=position_estimate[1]

            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd=max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd=-max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd=max_vel

            if x_reached is True:
                if(position_estimate[1]>start_y+y_offset):
                    x_reached=False
                    body_y_cmd=0
                    body_x_cmd=-max_vel
                else:
                    body_y_cmd=max_vel
                    body_x_cmd=0              


            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)



if __name__ == '__main__':

    dronito=Drone
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
        
        while(1):
            #get_pos
            zigzag(scf)
            #send_commmand()

        #stop logging
        logconf.stop()