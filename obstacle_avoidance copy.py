from cflib.utils.multiranger import Multiranger
from drone import Drone


# const
VELOCITY = 0.2
THRESH_Y = 0.2

#variables
pos_estimate_before_x = 0
pos_estimate_before_y = 0
yaw_landing=0
velocity_left = 0
velocity_front = 0
state = 1
first_detection = 0
no_detection = 0
from_front =0
from_back =0
from_left =0
from_right =0
obstacle_at_left = 0
obstacle_at_right = 0




def is_close(range):
    MIN_DISTANCE = 0.5  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def  obstacle_avoid_left_right(multiranger_left, multiranger_right, multiranger_front, multiranger_back):
    global velocity_front, velocity_left, pos_estimate_before_x, state, first_detection, no_detection, from_left, from_right, pos_estimate_before_y, obstacle_at_front,obstacle_at_back
    global case
    
    if (is_close(multiranger_left) & (not (from_right or from_front or from_back)) & case==Drone.state_zigzag['left'] ):
        print('state =1 left') 

        from_left = 1

        if (state==1) :
            pos_estimate_before_x = Drone.est_x #x front/back
            pos_estimate_before_y = Drone.est_y #y left/right
        if (abs(pos_estimate_before_y - (Drone.boxborder_right) < THRESH_Y) or abs(pos_estimate_before_y - (Drone.boxborder_left) < THRESH_Y)): #obstacle very close to y border
            case=Drone.state_zigzag["forward1"]
            return False
        if abs(pos_estimate_before_x - (Drone.boxborder_back)) > abs(pos_estimate_before_x - (Drone.boxborder_front)): #back better than front?
            velocity_front = - VELOCITY
        else :
            velocity_front = VELOCITY
        velocity_left = 0.0
        state = 2
        return True

    if (is_close(multiranger_right)  & (not (from_left or from_front or from_back)) & case==Drone.state_zigzag['right']):
        print('state =1 right') 
        
        from_right = 1
        
        if (state==1) :
            pos_estimate_before_x = Drone.est_x
            pos_estimate_before_y = Drone.est_y
        if (abs(pos_estimate_before_y - (Drone.boxborder_right) < THRESH_Y) or abs(pos_estimate_before_y - (Drone.boxborder_left) < THRESH_Y)): #obstacle very close to y border
            case=Drone.state_zigzag["forward2"]
            return False
        if abs(pos_estimate_before_x - (Drone.boxborder_back)) > abs(pos_estimate_before_x - (Drone.boxborder_front)): #back better than front?
            obstacle_at_front = 1
            velocity_front = - VELOCITY
        else :
            obstacle_at_back = 1
            velocity_front = VELOCITY
        velocity_left = 0.0
        state = 2
        return True
    
    if (state == 2): #state 2
        print('state =2')
        velocity_front = 0.0
        if (from_right):
            velocity_left = -VELOCITY
            dist = -0.1
        if (from_left):
            velocity_left = +VELOCITY 
            dist = +0.1
        if(obstacle_at_back):
            if (first_detection):
                if (not(is_close(multiranger_back))): 
                    no_detection = no_detection + 1
                    if (no_detection >= 2): # for safety
                        state = 3
                        mc.move_distance(0, dist, 0, VELOCITY)
            if (is_close(multiranger_back)):
                first_detection =1
                velocity_front = 0.05
        if(obstacle_at_front):
            if (first_detection):
                if (not(is_close(multiranger_front))): 
                    no_detection = no_detection + 1
                    if (no_detection >= 2): # for safety
                        state = 3
                        mc.move_distance(0, dist, 0, VELOCITY)
            if (is_close(multiranger_front)):
                first_detection =1
                velocity_front = - 0.05
                
        return True
        
    if (state == 3): #state 3
        print('state =3')
        velocity_left = 0
        if abs(pos_estimate_before_x - (Drone.boxborder_back)) > abs(pos_estimate_before_x - (Drone.boxborder_front)):
            velocity_front = VELOCITY
        else :
            velocity_front = - VELOCITY
        if (Drone.est_x < abs(pos_estimate_before_x + 0.03)):
            print('fin state 3')
            if (is_close(multiranger_right)):
                ('right close going left')
                velocity_left = VELOCITY
                velocity_front = 0
                return True 
            elif (is_close(multiranger_left)):
                ('left close going right')
                velocity_left = -VELOCITY
                velocity_front = 0
                return True
            state = 1
            no_detection = 0
            first_detection = 0
            velocity_left = 0
            velocity_front = 0
            from_left =0
            from_right =0
            pos_estimate_before_x = 0
            pos_estimate_before_y = 0
            return False
        return True  #check this indent
    return False

def  obstacle_avoid_front_back(multiranger_left, multiranger_right, multiranger_front, multiranger_back):
    global velocity_front, velocity_left, pos_estimate_before_y, state, first_detection, no_detection, from_front, from_back, obstacle_at_left, obstacle_at_right

    if (is_close(multiranger_front) & (not from_back) & (case==Drone.state_zigzag['forward1'] or case==Drone.state_zigzag['start'] or case==Drone.state_zigzag['forward2'] )): 
        print('state =1 front')
        from_front = 1
        if (state==1) :
            pos_estimate_before_y = Drone.est_y #y pos left/right
            if abs(pos_estimate_before_y - (Drone.boxborder_right)) > abs(pos_estimate_before_y - (Drone.boxborder_left)): # if distance plus grande a droite ? 
                obstacle_at_left =1
                velocity_left = - VELOCITY
            else :
                obstacle_at_right =1
                velocity_left = VELOCITY
        velocity_front = 0
        state = 2
        return True

    if (is_close(multiranger_back) & (not from_front) & case==Drone.state_zigzag['arrived']): #jamais pour le moment
        print('state =1 back')
        from_back = 1
        if (state==1) :
            pos_estimate_before_y = Drone.est_y #y pos left/right
            if abs(pos_estimate_before_y - (Drone.boxborder_right)) > abs(pos_estimate_before_y - Drone.boxborder_left): # if distance plus grande a droite ? 
                obstacle_at_left =1
                velocity_left = - VELOCITY
            else :
                obstacle_at_right =1
                velocity_left = VELOCITY
        velocity_front = 0
        state = 2
        return True
    
    if (state == 2): #state 2
        print('state =2')
        velocity_left = 0.0
        if (from_front):
            velocity_front = VELOCITY
            # dist = 0.1
        if (from_back):
            velocity_front = -VELOCITY
            # dist = -0.1
        if (obstacle_at_left):      
            if (first_detection):
                if (not(is_close(multiranger_left))): 
                    no_detection = no_detection + 1
                    if (no_detection >= 2): # for safety
                        state = 3
                        # mc.move_distance(dist, 0, 0, VELOCITY)
            if (is_close(multiranger_left)):
                first_detection =1
                velocity_left = - 0.05 
        if (obstacle_at_right):      
            if (first_detection):
                if (not(is_close(multiranger_right))): 
                    no_detection = no_detection + 1
                    if (no_detection >= 2): # for safety
                        state = 3
                        # mc.move_distance(dist, 0, 0, VELOCITY)
            if (is_close(multiranger_right)):
                first_detection =1
                velocity_left = 0.05  
        return True
        
    if (state == 3): #state 3
        print('state =3')
        if (obstacle_at_left):
            velocity_left = + VELOCITY
        else :
            velocity_left = - VELOCITY
        velocity_front = 0
        if (Drone.est_y < abs(pos_estimate_before_y + 0.03)):
            print('fin state 3')
            if (is_close(multiranger_back)):
                velocity_left = 0
                velocity_front = VELOCITY
                return True
            elif (is_close(multiranger_front)):
                velocity_left = 0
                velocity_front = -VELOCITY
                return True
            state = 1
            no_detection = 0
            first_detection = 0
            velocity_left = 0
            velocity_front = 0
            from_front =0
            from_back =0
            obstacle_at_left =0
            obstacle_at_right =0
            pos_estimate_before_y =0
            pos_estimate_before_x =0
            return False
        return True  #check this indent
    return False

def obstacle_avoidance(multiranger_left, multiranger_right, multiranger_front, multiranger_back):
    #print(case)
#il faut penser au cas ou la vitesse n'est pas dans la direction de l'obstacle
    if ((is_close(multiranger_left) or is_close(multiranger_right) or from_left or from_right) & (from_front ==0) & (from_back == 0)):
        return obstacle_avoid_left_right()
    elif ((is_close(multiranger_front) or is_close(multiranger_back) or from_front or from_back) & (from_right ==0) & (from_left == 0)): 
        #print(case)
        return obstacle_avoid_front_back()
    return False
