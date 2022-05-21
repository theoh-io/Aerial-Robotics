import logging
from pickle import FALSE
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
from cflib.utils.multiranger import Multiranger

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Unit: meter
DEFAULT_HEIGHT = 0.25 #1
FOV_ZRANGER=math.radians(2.1)
BOX_LIMIT_X = 2 #5
BOX_LIMIT_Y = 1 #3
START_POS_X = 0
START_POS_Y = 0
GOAL_ZONE_X=0.1
START_EXPLORE_X = GOAL_ZONE_X-START_POS_X

TIME_EXPLORE= 90


#to be added in parser
verbose = True
state={"start":-1, "left":0, "forward1":1, "right":2, "forward2":3, "back2left":4, "arrived":5}

## A* star or global nav variables
start = [START_POS_X, START_POS_Y] # to get before the start of the drone
goal = [100,100] # to get from the zranger detection, to get when landing on base done
len_x, len_y = (500, 200)

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0, 0, 0]

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
    #print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    position_estimate[3] = data['stateEstimate.yaw']


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
    global case, x_offset, yaw_landing
    #to test way_back
    global start_time, goal_x, goal_y
    if case==state["start"]:
        mc.start_forward()
        case=state["left"]
    if (case==state["left"] and position_estimate[0]>START_EXPLORE_X) or case ==4:
        mc.start_left()
        case=state["forward1"]
    elif case==state["forward1"] and position_estimate[1] > BOX_LIMIT_Y-START_POS_Y:
        print("!!!!!!!!!!!!reached bbox")
        mc.forward(x_offset)
        case=state["right"]
    elif case == state["right"]:
        mc.start_right()
        case = state["forward2"]
    elif case == state["forward2"] and  position_estimate[1] < -START_POS_Y:
        mc.forward(x_offset)
        case=state["back2left"]
    #Temporaire: condition d'arret si la limite de l'arene en x 
    if position_estimate[0] > BOX_LIMIT_X - START_POS_X:
        mc.land()
    #Temporaire condition de retour basé sur le temps de vol
    if(time.time()-start_time>TIME_EXPLORE):
        print(" Exploration time exceeded")
        yaw_landing=position_estimate[2]
        print("yaw during landing", yaw_landing)
        mc.land()
        goal_x=position_estimate[0]
        goal_y=position_estimate[1]
        mc.take_off(DEFAULT_HEIGHT)
        time.sleep(1)
        case =state["arrived"] #to get out of zigzag

def go_back():
    global goal_x, goal_y
    #first goes to 0 in x
    dist_x=goal_x
    mc.back(dist_x)
    #goes to 0 in y
    dist_y=goal_y
    mc.right(dist_y)
    mc.land()

def compute_offset():
    offset=math.tan(FOV_ZRANGER)/DEFAULT_HEIGHT
    print(offset)
    return offset

def clean_takeoff(mc): 
    time.sleep(1)
    mc._reset_position_estimator()
    time.sleep(1)
    init_x = position_estimate[0]
    init_y = position_estimate[1]
    init_yaw = position_estimate[3]
    print("Start pos:",START_POS_X+position_estimate[0]-init_x,START_POS_Y+position_estimate[1]-init_y)
    print("Start yaw:", position_estimate[3]-init_yaw)
    time.sleep(3)
    regulate_yaw(mc, init_yaw, position_estimate[3])
    return init_x, init_y, init_yaw

# regulate yaw to init angle
def regulate_yaw(mc, init_yaw, curr_yaw):
    print("in regulate yaw")
    if init_yaw - curr_yaw >= 0:
        mc.turn_left(init_yaw - curr_yaw)
    if init_yaw - curr_yaw < 0:
        mc.turn_right(curr_yaw - init_yaw)

def is_close(range):
    MIN_DISTANCE = 0.5  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def obstacle_avoidance():
    global velocity_x, velocity_y, pos_estimate_before, state, first_detection, no_detection 

    if (is_close(multiranger.left)):  #state 1
        print(multiranger.left)
        print("object detected on the left")
        if state==1 :
            pos_estimate_before = position_estimate[0]
        velocity_y = 0.0
        velocity_x = VELOCITY
        print(velocity_x)
        state = 2
        return True
    
    if (state == 2): #state 2
        print('state =2')
        velocity_x = 0.0
        velocity_y = VELOCITY
        if (first_detection):
            if (not(is_close(multiranger.back))): 
                no_detection = no_detection + 1
                print(no_detection)
                if (no_detection >= 2): # for safety
                    state = 3
        if (is_close(multiranger.back)):
            first_detection =1
            print('first_detection')
            print(first_detection)
        return True
        
    if (state == 3): #state 3
        print('state =3')
        velocity_y = 0
        velocity_x = - VELOCITY
        if (position_estimate[0] < abs(pos_estimate_before + 0.01)):
            state = 1
            return False
        return True  #check this indent

    return False


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
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        #start logging
        logconf.start()

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            with Multiranger(scf) as multiranger:
                #little sleep needed for takeoff
                time.sleep(1)
                #variables used for the wayback test based on time
                start_time=time.time()
                print(start_time)
                goal_x=0
                goal_y=0
                #variables needed for zigzag
                case=-1
                x_offset=0.25#compute_offset() test with 30cm
                #variables needed for obstacle avoidance
                VELOCITY = 0.2
                pos_estimate_before = 0
                yaw_landing=0
                velocity_y = 0
                velocity_x = 0
                state = 1
                first_detection = 0
                no_detection = 0

                while(1):
                    print(obstacle_avoidance())
                    if (obstacle_avoidance() == False):
                        print('obs false')
                        #if no obstacle is being detected let zigzag manage the speeds
                        if case != 5:
                            zigzag_nonblocking()
                        else:
                            regulate_yaw(mc, yaw_landing, position_estimate[2]) #compensate the error in yaw during landing
                            print("yaw after regulate:", position_estimate[3])
                            go_back()
                        time.sleep(1)
                    else:
                        print("obstacle av = True")
                        #print(velocity_x, velocity_y)
                        #obstacle detected then gives manually the speeds defined by obstacle avoidance
                        mc.start_linear_motion(velocity_x, velocity_y, 0)
                        time.sleep(0.1)
        #stop logging
        logconf.stop()

        