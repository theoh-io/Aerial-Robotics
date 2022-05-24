import logging
from pickle import FALSE
import sys
import time
from threading import Event
from threading import Timer
import math
import datetime as dt
from zipfile import ZIP_BZIP2
import numpy as np
import os
import matplotlib.pyplot as plt
#from regex import R

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger  
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Unit: meter
DEFAULT_HEIGHT = 0.5 #1
FOV_ZRANGER=math.radians(2.1)
BOX_LIMIT_X = 1.5 #5
BOX_LIMIT_Y = 0.15 #3
START_POS_X = 0
START_POS_Y = 0
GOAL_ZONE_X= 0.5
START_EXPLORE_X = GOAL_ZONE_X-START_POS_X

TIME_EXPLORE= 15

EPSYLON=0.001


#to be added in parser
verbose = True
state_zigzag={'start':-1, 'left':0, 'forward1':1, 'right':2, 'forward2':3, 'back2left':4, 'arrived':5}

## A* star or global nav variables
start = [START_POS_X, START_POS_Y] # to get before the start of the drone
goal = [100,100] # to get from the zranger detection, to get when landing on base done
len_x, len_y = (500, 200)

deck_attached_event = Event()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Logs global variables
position_estimate = [0, 0, 0, 0, 0]
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
    #print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    position_estimate[3] = data['range.zrange']
    position_estimate[4] = data['stateEstimate.yaw']

def stab_log_data(timestamp, data, logconf):
    """Callback froma the log API when data arrives"""
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global count
    
    # Save info into log variable
    for idx, i in enumerate(list(data)):
        if idx < 3:
            logs[count][idx] = data[i]*1000
        if idx == 4:
            #multiply yaw by 100
            logs[count][idx] = data[i]*100
        else:
            logs[count][idx] = data[i]
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
    global edge, position_estimate

    #print(state_zigzag['start'])
    if case==state_zigzag["start"]:
        mc.start_forward()
        print('start')
    if (case==state_zigzag["start"] and position_estimate[0]>START_EXPLORE_X) or case == state_zigzag["back2left"]:
        regulate_yaw(mc, 0, position_estimate[4])
        case=state_zigzag["left"]
        mc.start_left()
        print('left')
    elif (case==state_zigzag["left"] and position_estimate[1] > BOX_LIMIT_Y-START_POS_Y) :
        regulate_yaw(mc, 0, position_estimate[4])
        case=state_zigzag["forward1"]
        print('forward 1')
        mc.forward(x_offset)
    elif case == state_zigzag["forward1"]:
        regulate_yaw(mc, 0, position_estimate[4])
        case=state_zigzag["right"]
        print('right')
        mc.start_right()
    elif case == state_zigzag["right"] and  position_estimate[1] < -START_POS_Y:
        regulate_yaw(mc, 0, position_estimate[4])
        case = state_zigzag["forward2"]
        print('forward 2')
        mc.forward(x_offset)
        case=state_zigzag["back2left"]
        print('back2left')

    #Temporaire: condition d'arret si la limite de l'arene en x 
    if position_estimate[0] > BOX_LIMIT_X - START_POS_X:
        print("Limite arene x reached")
        mc.land()
        time.sleep(1)
        case =state_zigzag["arrived"]
    
    #Temporaire condition de retour basé sur le temps de vol
    if(time.time()-start_time>TIME_EXPLORE):
        print(" Exploration time exceeded")
        yaw_landing=position_estimate[4]
        print("yaw during landing", yaw_landing)
        #must record goal pos before landing because variation can occur
        goal_x=position_estimate[0]
        goal_y=position_estimate[1]
        mc.land()
        time.sleep(1)
        mc.take_off(DEFAULT_HEIGHT)
        #clean_takeoff(mc, [goal_x, goal_y, yaw_landing])
        case =state_zigzag["arrived"] #to get out of zigzag

    if ( edge == True and (case != state_zigzag["start"]) and (case != state_zigzag["arrived"]) ):
        print('Edge far detected!')
        #yaw_landing=position_estimate[2]
        #must record goal pos before landing because variation can occur
        #goal_x=position_estimate[0]
        #goal_y=position_estimate[1]
        
        find_platform_center()
        #mc.land()
        #time.sleep(1)
        #mc.take_off(DEFAULT_HEIGHT)
        #clean_takeoff(mc, [goal_x, goal_y, yaw_landing])
        #case =state_zigzag["arrived"] #to get out of zigzag

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
    #print(offset)
    return offset

def clean_takeoff(mc, init_coord=None): 
    #au début pas de coordonnées initiales
    if init_coord is None:
        time.sleep(0.1)
        #nécéssaire ???
        #mc._reset_position_estimator()
        init_x = START_POS_X+position_estimate[0]
        init_y = START_POS_Y+position_estimate[1]
        #init_yaw = 0    
        print("Start pos (x, y):", init_x, init_y)
        print("Start yaw:", position_estimate[4])
        regulate_x(mc, START_POS_X, init_x)
        time.sleep(1)
        regulate_y(mc, START_POS_Y, init_y)
        time.sleep(1)
        regulate_yaw(mc, 0, position_estimate[4])
        time.sleep(1)
        return init_x, init_y
    #apres le landing on veut controler la position après le redécollage
    else:
        print("in regulate re-takeoff")
        time.sleep(1)
        curr_x = position_estimate[0]
        curr_y = position_estimate[1]
        curr_yaw = position_estimate[4]
        print("current pos (x, y, yaw):", curr_x, curr_y, curr_yaw)
        print("before landing pos (x, y, yaw):", init_coord[0], init_coord[1], init_coord[2])
        #regulate_x(mc, init_coord[0], curr_x)
        #regulate_y(mc, init_coord[1], curr_y)
        regulate_yaw(mc, init_coord[2], curr_yaw)
        return init_x, init_y

def regulate_x(mc, init_x, curr_x):
    print("in regulate_x")
    error_x=curr_x-init_x
    if error_x>EPSYLON:
        mc.back(error_x)
    if error_x<-EPSYLON:
        mc.forward(-error_x)
    else:
        print("zero_error")

def regulate_y(mc, init_y, curr_y):
    print("in regulate_y")
    error_y=curr_y-init_y
    if error_y>EPSYLON:
        mc.right(error_y)
    if error_y<-EPSYLON:
        mc.left(-error_y)
    else:
        print("zero_error")



# regulate yaw to init angle
def regulate_yaw(mc, init_yaw, curr_yaw):
    print("in regulate yaw")
    if init_yaw - curr_yaw >= EPSYLON:
        mc.turn_left(init_yaw - curr_yaw)
    if init_yaw - curr_yaw < -EPSYLON:
        mc.turn_right(curr_yaw - init_yaw)
    else:
        print("zero error")

def is_close(range):
    MIN_DISTANCE = 0.5  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def  obstacle_avoid_left_right():
    global velocity_x, velocity_y, pos_estimate_before, state, first_detection, no_detection, from_left, from_right

    if (is_close(multiranger.left) & (not from_right) & case==state_zigzag['left'] ):
        print('state =1 left') 
        from_left = 1
        if (state==1) :
            pos_estimate_before = position_estimate[0]
        velocity_y = 0.0
        velocity_x = VELOCITY
        state = 2
        return True

    if (is_close(multiranger.right)  & (not from_left) & case==state_zigzag['right']):
        print('state =1 right') 
        from_right = 1
        if (state==1) :
            pos_estimate_before = position_estimate[0]
            print(pos_estimate_before)
            print()
        velocity_y = 0.0
        velocity_x = VELOCITY
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
        velocity_x = - VELOCITY
        if (position_estimate[0] < abs(pos_estimate_before + 0.03)):
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

    if (is_close(multiranger.front) & (not from_back) & (case==state_zigzag['forward1'] or case==state_zigzag['start'] or case==state_zigzag['forward2'] )): 
        print('state =1 front')
        from_front = 1
        if (state==1) :
            pos_estimate_before = position_estimate[0]
            print(pos_estimate_before)
        velocity_y = - VELOCITY
        velocity_x = 0
        state = 2
        return True

    if (is_close(multiranger.back) & (not from_front) & 0): #jamais pour le moment
        print('state =1 back')
        from_back = 1
        if (state==1) :
            pos_estimate_before = position_estimate[1]
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
        velocity_y = VELOCITY
        velocity_x = 0
        print(position_estimate[1])
        print(pos_estimate_before)
        if (position_estimate[1] < abs(pos_estimate_before + 0.03)):
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
            print('abs edge 2: ',abs(z_1-z_2))
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
    global edge, case, logs, state_zigzag, position_estimate, x_edge, y_edge 

    #x1=position_estimate[0]
    #y1=position_estimate[1]
    x1=x_edge
    y1=y_edge
    
    """
    x1_bis=position_estimate[0]
    y1_bis=position_estimate[1]

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
        while(position_estimate[1]<(y1-0.15)):
            print('going left ',position_estimate[1],' ',y1)
        x2_before=position_estimate[0]
        y2_before=position_estimate[1]

    if case == state_zigzag["left"]:
        mc.start_right()
        while(position_estimate[1]>y1+0.15):
            print('going right ',position_estimate[1],' ',y1)
        x2_before=position_estimate[0]
        y2_before=position_estimate[1]

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
            
        if position_estimate[0] > BOX_LIMIT_X - START_POS_X:
            print("Limite arene x reached")
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

    mc.forward(0.05)
    goal_x=position_estimate[0]
    goal_y=position_estimate[1]

    time.sleep(2)
    mc.land()
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

# ------------------------------------------------------------------------------------------------------------

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
        logconf.add_variable('range.zrange', 'uint16_t')
        logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.data_received_cb.add_callback(stab_log_data)

        #start logging
        logconf.start()

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            with Multiranger(scf) as multiranger:
                #little sleep needed for takeoff
                time.sleep(0.1)
                #function to reset the estimations
                #clean_takeoff(mc)
                #variables used for the wayback test based on time
                start_time=time.time()
                print(start_time)
                goal_x=0
                goal_y=0
                #variables needed for zigzag
                case=state_zigzag["start"]
                x_offset=0.25 #compute_offset() test with 30cm
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
                #temporary intentional disturbance to regulate yaw
                # time.sleep(1)
                # mc.turn_left(7)
                clean_takeoff(mc)

                while(1):
                    #print(obstacle_avoidance())
                    #if (obstacle_avoidance() == False):
                    if True:
                        print('obs false')
                        #if no obstacle is being detected let zigzag manage the speeds
                        if case != state_zigzag['arrived']:
                            #[edge,x_edge,y_edge] = is_edge_2()
                            zigzag_nonblocking()
                            #print("yaw :", position_estimate[4])
                            #regulate_yaw(mc,0, position_estimate[4])
                        else:
                            #regulate_yaw(mc, yaw_landing, position_estimate[3]) #compensate the error in yaw during landing
                            #print("yaw after regulate:", position_estimate[3])
                            go_back()
                            logconf.stop()
                            store_log_data()
                            break
                        time.sleep(1)
                    else:
                        print("obstacle av = True")
                        #print(velocity_x, velocity_y)
                        #obstacle detected then gives manually the speeds defined by obstacle avoidance
                        mc.start_linear_motion(velocity_x, velocity_y, 0)
                        time.sleep(0.1)
        #stop logging
        #logconf.stop()
        #store_log_data()

        