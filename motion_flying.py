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
DEFAULT_HEIGHT = 1 #1
FOV_ZRANGER=math.radians(2.1)
BOX_LIMIT_X = 2 #5
BOX_LIMIT_Y = 0.5 #3
START_POS_X = 0
START_POS_Y = 0
GOAL_ZONE_X=0.1
START_EXPLORE_X = GOAL_ZONE_X-START_POS_X

TIME_EXPLORE= 30


#to be added in parser
verbose = True
state_zigzag={'start':-1, 'left':0, 'forward1':1, 'right':2, 'forward2':3, 'back2left':4, 'arrived':5}

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
    global case, x_offset, yaw_landing, state_zigzag
    #to test way_back
    global start_time, goal_x, goal_y
    print(state_zigzag['start'])
    if case==state_zigzag["start"]:
        mc.start_forward()
        case=state_zigzag["left"]
    if (case==state_zigzag["left"] and position_estimate[0]>START_EXPLORE_X) or case ==4:
        mc.start_left()
        case=state_zigzag["forward1"]
    elif case==state_zigzag["forward1"] and position_estimate[1] > BOX_LIMIT_Y-START_POS_Y:
        print("!!!!!!!!!!!!reached bbox")
        mc.forward(x_offset)
        case=state_zigzag["right"]
    elif case == state_zigzag["right"]:
        mc.start_right()
        case = state_zigzag["forward2"]
    elif case == state_zigzag["forward2"] and  position_estimate[1] < -START_POS_Y:
        mc.forward(x_offset)
        case=state_zigzag["back2left"]
    #Temporaire: condition d'arret si la limite de l'arene en x 
    if position_estimate[0] > BOX_LIMIT_X - START_POS_X:
        mc.land()
    #Temporaire condition de retour basé sur le temps de vol
    if(time.time()-start_time>TIME_EXPLORE):
        print(" Exploration time exceeded")
        yaw_landing=position_estimate[2]
        print("yaw during landing", yaw_landing)
        #must record goal pos before landing because variation can occur
        goal_x=position_estimate[0]
        goal_y=position_estimate[1]
        mc.land()
        time.sleep(1)
        mc.take_off(DEFAULT_HEIGHT)
        #clean_takeoff(mc, [goal_x, goal_y, yaw_landing])
        case =state_zigzag["arrived"] #to get out of zigzag

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

def clean_takeoff(mc, init_coord=None): 
    #au début pas de coordonnées initiales
    if init_coord is None:
        time.sleep(0.1)
        #nécéssaire ???
        mc._reset_position_estimator()
        time.sleep(0.2)
        init_x = START_POS_X+position_estimate[0]
        init_y = START_POS_Y+position_estimate[1]
        init_yaw = position_estimate[3]
        print("Start pos (x, y):", init_x, init_y)
        print("Start yaw:", position_estimate[3]-init_yaw)
        regulate_x(mc, START_POS_X, init_x)
        time.sleep(0.1)
        regulate_y(mc, START_POS_Y, init_y)
        time.sleep(0.1)
        regulate_yaw(mc, init_yaw, position_estimate[3])
        time.sleep(0.1)
        return init_x, init_y, init_yaw
    #apres le landing on veut controler la position après le redécollage
    # else:
    #     print("in regulate re-takeoff")
    #     time.sleep(1)
    #     curr_x = position_estimate[0]
    #     curr_y = position_estimate[1]
    #     curr_yaw = position_estimate[3]
    #     print("current pos (x, y, yaw):", curr_x, curr_y, curr_yaw)
    #     print("before landing pos (x, y, yaw):", init_coord[0], init_coord[1], init_coord[2])
    #     #regulate_x(mc, init_coord[0], curr_x)
    #     #regulate_y(mc, init_coord[1], curr_y)
    #     regulate_yaw(mc, init_coord[2], curr_yaw)
    #     return init_x, init_y, init_yaw

def regulate_x(mc, init_x, curr_x):
    print("in regulate_x")
    error_x=curr_x-init_x
    if error_x>0:
        mc.back(error_x)
    if error_x<0:
        mc.forward(-error_x)
    else:
        print("zero_error")

def regulate_y(mc, init_y, curr_y):
    print("in regulate_y")
    error_y=curr_y-init_y
    if error_y>0:
        mc.right(error_y)
    if error_y<0:
        mc.left(-error_y)
    else:
        print("zero_error")



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

def  obstacle_avoid_left_right():
    global velocity_x, velocity_y, pos_estimate_before, state, first_detection, no_detection, from_left, from_right

    if (is_close(multiranger.left)): 
        from_left = 1
        if (state==1) :
            pos_estimate_before = position_estimate[0]
        velocity_y = 0.0
        velocity_x = VELOCITY
        state = 2
        return True

    if (is_close(multiranger.right)): 
        from_right = 1
        if (state==1) :
            pos_estimate_before = position_estimate[0]
        velocity_y = 0.0
        velocity_x = VELOCITY
        state = 2
        return True
    
    if (state == 2): #state 2
        print('state =2')
        velocity_x = 0.0
        if (from_right):
            velocity_y = VELOCITY
        if (from_left):
            velocity_y = -VELOCITY      
        if (first_detection):
            if (not(is_close(multiranger.back))): 
                no_detection = no_detection + 1
                if (no_detection >= 2): # for safety
                    state = 3
        if (is_close(multiranger.back)):
            first_detection =1
        return True
        
    if (state == 3): #state 3
        print('state =3')
        velocity_y = 0
        velocity_x = - VELOCITY
        if (position_estimate[0] < abs(pos_estimate_before + 0.01)):
            state = 1
            no_detection = 0
            first_detection = 0
            velocity_y = 0
            velocity_x = 0
            from_left =0
            from_right =0
            return False
        return True  #check this indent

def  obstacle_avoid_front_back():
    global velocity_x, velocity_y, pos_estimate_before, state, first_detection, no_detection, from_front, from_back

    if (is_close(multiranger.front)): 
        from_front = 1
        if (state==1) :
            pos_estimate_before = position_estimate[0]
        velocity_y = - VELOCITY
        velocity_x = 0
        state = 2
        return True

    if (is_close(multiranger.back)): 
        from_back = 1
        if (state==1) :
            pos_estimate_before = position_estimate[0]
        velocity_y = - VELOCITY
        velocity_x = 0
        state = 2
        return True
    
    if (state == 2): #state 2
        print('state =2')
        velocity_y = 0.0
        if (from_front):
            velocity_x = VELOCITY
        if (from_back):
            velocity_x = -VELOCITY      
        if (first_detection):
            if (not(is_close(multiranger.left))): 
                no_detection = no_detection + 1
                if (no_detection >= 2): # for safety
                    state = 3
        if (is_close(multiranger.left)):
            first_detection =1
        return True
        
    if (state == 3): #state 3
        print('state =3')
        velocity_y = VELOCITY
        velocity_x = 0
        if (position_estimate[0] < abs(pos_estimate_before + 0.01)):
            state = 1
            no_detection = 0
            first_detection = 0
            velocity_y = 0
            velocity_x = 0
            from_front =0
            from_back =0
            return False
        return True  #check this indent

def obstacle_avoidance():

    if ((is_close(multiranger.left) or is_close(multiranger.right) or from_left or from_right) & (from_front ==0) & (from_back == 0)):  #state 1
        return obstacle_avoid_left_right()
    elif ((is_close(multiranger.front) or is_close(multiranger.back) or from_front or from_back) & (from_right ==0) & (from_left == 0)):
        return obstacle_avoid_front_back()
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
                time.sleep(0.1)
                #function to reset the estimations
                clean_takeoff(mc)
    
                #variables used for the wayback test based on time
                start_time=time.time()
                print(start_time)
                goal_x=0
                goal_y=0
                #variables needed for zigzag
                case=state_zigzag["start"]
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
                from_front =0
                from_back =0
                from_left =0
                from_right =0


                while(1):
                    print(obstacle_avoidance())
                    if (obstacle_avoidance() == False):
                        print('obs false')
                        #if no obstacle is being detected let zigzag manage the speeds
                        if case != state_zigzag['arrived']:
                            zigzag_nonblocking()
                        else:
                            regulate_yaw(mc, yaw_landing, position_estimate[3]) #compensate the error in yaw during landing
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

        