from cflib.utils.multiranger import Multiranger
from drone import Drone 


# const
VELOCITY = 0.2
THRESH_Y = 0.2
LOW_VELOCITY = 0.05
BOX_LIMIT_X = 3 #5
BOX_LIMIT_Y = 1 #3

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
obstacle_at_front = 0
obstacle_at_back = 0
 


def is_close(range):
    MIN_DISTANCE = 0.4  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def  obstacle_avoid_left_right(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito):
    global velocity_front, velocity_left, pos_estimate_before_x, state, first_detection, no_detection, from_left, from_right, pos_estimate_before_y, obstacle_at_front, obstacle_at_back
    global case
    
    if (is_close(multiranger_left) and (not (from_right)) and case==dronito.state_zigzag['left'] ):
        print('state =1 left') 
        print(multiranger_left)
        print(is_close(multiranger_left))
        from_left = 1

        if (state==1) :
            pos_estimate_before_x = dronito.est_x #x front/back
            pos_estimate_before_y = dronito.est_y #y left/right
        
        if (abs(pos_estimate_before_y - (BOX_LIMIT_Y)) < THRESH_Y): #obstacle very close to y border prendre en cond min dist
            print("too close to border")
            case=dronito.state_zigzag["forward1"]
            from_left = 0
            return False
        
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (BOX_LIMIT_X )): #back better than front?
            obstacle_at_front = 1
            velocity_front = - VELOCITY
        else :
            obstacle_at_back = 1
            velocity_front = VELOCITY
        velocity_left = 0.0
        state = 2
        return True

    if (is_close(multiranger_right)  and (not (from_left)) and case==dronito.state_zigzag['right']):
        print('state =1 right') 
        print(multiranger_left)
        print(is_close(multiranger_left))
        from_right = 1
        
        if (state==1) :
            pos_estimate_before_x = dronito.est_x
            pos_estimate_before_y = dronito.est_y
        
        if (abs(pos_estimate_before_y) < THRESH_Y): #obstacle very close to y border
            print("too close to border")
            case=dronito.state_zigzag["forward2"]
            from_right = 0
            return False  
        
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (BOX_LIMIT_X)): #back better than front?
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
            #dist = -0.1
        if (from_left):
            velocity_left = +VELOCITY 
            #dist = +0.1
        if(obstacle_at_back):
            print('obstacle_at_back')
            if (first_detection):
                print('first_detection')
                if (not(is_close(multiranger_back))):
                    print('not close back')
                    no_detection = no_detection + 1
                    if (no_detection >= 2): # for safety
                        state = 3
                        #mc.move_distance(0, dist, 0, VELOCITY)
            if (is_close(multiranger_back)):
                first_detection =1
                velocity_front = LOW_VELOCITY
        if(obstacle_at_front):
            print('obstacle_at_front')
            if (first_detection):
                print('first_detection')
                if (not(is_close(multiranger_front))): 
                    print('not close front')
                    no_detection = no_detection + 1
                    if (no_detection >= 1): # for safety
                        state = 3
                        #mc.move_distance(0, dist, 0, VELOCITY)
            if (is_close(multiranger_front)):
                first_detection =1
                velocity_front = - LOW_VELOCITY
                
        return True
        
    if (state == 3): #state 3
        print('state =3')
        velocity_left = 0
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (BOX_LIMIT_X)): #back better than front?
            velocity_front = VELOCITY
        else :
            velocity_front = - VELOCITY
        if (dronito.est_x < abs(pos_estimate_before_x + 0.03)):
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
            obstacle_at_back = 0
            obstacle_at_front = 0
            return False
        return True  #check this indent
    return False

def  obstacle_avoid_front_back(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito):
    global velocity_front, velocity_left, pos_estimate_before_y, state, first_detection, no_detection, from_front, from_back, obstacle_at_left, obstacle_at_right

    if (is_close(multiranger_front) and (not from_back) and (case==dronito.state_zigzag['forward1'] or case==dronito.state_zigzag['start'] or case==dronito.state_zigzag['forward2'] )): 
        print('state =1 front')
        from_front = 1
        if (state==1) :
            pos_estimate_before_y = dronito.est_y #y pos left/right
            if abs(pos_estimate_before_y) > abs(pos_estimate_before_y - (BOX_LIMIT_Y)): # if distance plus grande a droite ? 
                obstacle_at_left =1
                velocity_left = - VELOCITY
            else :
                obstacle_at_right =1
                velocity_left = VELOCITY
        velocity_front = 0
        state = 2
        return True

    if (is_close(multiranger_back) and (not from_front) and (case==dronito.state_zigzag['arrived'])): #jamais pour le moment juste arrived
        print('state =1 back')
        from_back = 1
        print(case)
        if (state==1) :
            pos_estimate_before_y = dronito.est_y #y pos left/right
            if (abs(pos_estimate_before_y) > abs(pos_estimate_before_y - BOX_LIMIT_Y)): # if distance plus grande a droite ? 
                obstacle_at_left = 1
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
                velocity_left = - LOW_VELOCITY 
        if (obstacle_at_right):      
            if (first_detection):
                if (not(is_close(multiranger_right))): 
                    no_detection = no_detection + 1
                    if (no_detection >= 2): # for safety
                        state = 3
                        # mc.move_distance(dist, 0, 0, VELOCITY)
            if (is_close(multiranger_right)):
                first_detection =1
                velocity_left = LOW_VELOCITY  
        return True
        
    if (state == 3): #state 3
        print('state =3')
        if (obstacle_at_left):
            velocity_left = + VELOCITY
        else :
            velocity_left = - VELOCITY
        velocity_front = 0
        if (dronito.est_y < abs(pos_estimate_before_y + 0.03)):
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
            return False
        return True  #check this indent
    return False

def obstacle_avoidance(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito):
    if ((is_close(multiranger_left) or is_close(multiranger_right) or from_left or from_right) and (from_front ==0) and (from_back == 0)):
        return obstacle_avoid_left_right(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito)
    elif ((is_close(multiranger_front) or is_close(multiranger_back) or from_front or from_back) and (from_right ==0) and (from_left == 0)): 
        return obstacle_avoid_front_back(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito)
    return False

