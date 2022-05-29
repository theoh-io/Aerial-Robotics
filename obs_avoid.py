

# const
VELOCITY = 0.5
THRESH_Y = 0.9
LOW_VELOCITY = 0.05
MIN_DISTANCE = 0.5  # m
SAFETY_MARGIN = 2
SMALL_DIST = 0.2
EDGE_VELOCITY = 0

#variables
pos_estimate_before_x = 0
pos_estimate_before_y = 0
state_obs= 1
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

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def  obstacle_avoid_left_right(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito):
    global pos_estimate_before_x, state_obs, first_detection, no_detection, from_left, from_right, pos_estimate_before_y, obstacle_at_front, obstacle_at_back
    
    if (is_close(multiranger_left) and (not (from_right)) and dronito.case==dronito.state_zigzag['left'] or (dronito.case==dronito.state_zigzag['arrived'] and dronito.case2== dronito.state_zigzag['left'])): #left state 1
        print('state_obs=1 left') 
        print(multiranger_left)
        print(is_close(multiranger_left))
        from_left = 1

        if (state_obs==1) :
            pos_estimate_before_x = dronito.est_x #x front/back
            pos_estimate_before_y = dronito.est_y #y left/right
        
        if (abs(pos_estimate_before_y - (dronito.boxborder_left)) < THRESH_Y): #obstacle very close to y border from left
            print("too close to border")
            if (dronito.case==dronito.state_zigzag['arrived']):
                dronito.case2=dronito.state_zigzag["forward1"]
                from_left = 0
                dronito.start_forward=dronito.est_x
                dronito.mc.start_back()
            else:
                dronito.case=dronito.state_zigzag["forward1"]
                from_left = 0
                dronito.start_forward=dronito.est_x
                dronito.mc.start_forward()
            return False  
        
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (dronito.boxborder_front )): #back better than front?
            obstacle_at_front = 1
            dronito.velocity_front = - VELOCITY
        else :
            obstacle_at_back = 1
            dronito.velocity_front = VELOCITY
        dronito.velocity_left = 0.0
        state_obs= 2
        return True

    if (is_close(multiranger_right)  and (not (from_left)) and dronito.case==dronito.state_zigzag['right'] or (dronito.case==dronito.state_zigzag['arrived'] and dronito.case2== dronito.state_zigzag['right'])): #right state 
        print('state_obs=1 right') 
        print(multiranger_left)
        print(is_close(multiranger_left))
        from_right = 1
        
        if (state_obs==1) :
            pos_estimate_before_x = dronito.est_x
            pos_estimate_before_y = dronito.est_y
        
        if (abs(pos_estimate_before_y) < THRESH_Y): #obstacle very close to y border from right
            print("too close to border")
            if (dronito.case==dronito.state_zigzag['arrived']):
                dronito.case2=dronito.state_zigzag["forward2"]
                from_right = 0
                dronito.start_forward=dronito.est_x
                dronito.mc.start_back()
            else:
                dronito.case=dronito.state_zigzag["forward2"]
                from_right = 0
                dronito.start_forward=dronito.est_x
                dronito.mc.start_forward()
            return False  
        
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (dronito.boxborder_front)): #back better than front?
            obstacle_at_front = 1
            dronito.velocity_front = - VELOCITY
        else :
            obstacle_at_back = 1
            dronito.velocity_front = VELOCITY
        dronito.velocity_left = 0.0
        state_obs= 2
        return True
    
    if (state_obs== 2): #state_obs2
        print('state_obs=2')
        dronito.velocity_front = 0.0
        if (from_right):
            dronito.velocity_left = -VELOCITY
            #dist = -0.1
        if (from_left):
            dronito.velocity_left = +VELOCITY 
            #dist = +0.1
        if(obstacle_at_back):
            print('obstacle_at_back')
            if (first_detection):
                print('first_detection')
                if (not(is_close(multiranger_back))):
                    print('not close back')
                    no_detection = no_detection + 1
                    if (no_detection >= SAFETY_MARGIN): # for safety
                        state_obs= 3
                        #mc.move_distance(0, dist, 0, VELOCITY)
            if (is_close(multiranger_back)):
                first_detection =1
                dronito.velocity_front = LOW_VELOCITY
        if(obstacle_at_front):
            print('obstacle_at_front')
            if (first_detection):
                print('first_detection')
                if (not(is_close(multiranger_front))): 
                    print('not close front')
                    no_detection = no_detection + 1
                    if (no_detection >= SAFETY_MARGIN): # for safety
                        state_obs= 3
                        #mc.move_distance(0, dist, 0, VELOCITY)
            if (is_close(multiranger_front)):
                first_detection =1
                dronito.velocity_front = - LOW_VELOCITY
                
        return True
        
    if (state_obs== 3): #state_obs3
        print('state_obs=3')
        dronito.velocity_left = 0
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (dronito.boxborder_front)): #back better than front?
            dronito.velocity_front = VELOCITY - (VELOCITY - EDGE_VELOCITY)
        else :
            dronito.velocity_front = - VELOCITY + (VELOCITY - EDGE_VELOCITY)
        if (dronito.est_x < abs(pos_estimate_before_x + SMALL_DIST)):
            print('fin state_obs3')
            if (is_close(multiranger_right)):
                ('right close going left')
                dronito.velocity_left = VELOCITY
                dronito.velocity_front = 0
                return True 
            elif (is_close(multiranger_left)):
                ('left close going right')
                dronito.velocity_left = -VELOCITY
                dronito.velocity_front = 0
                return True
            state_obs= 1
            no_detection = 0
            first_detection = 0
            dronito.velocity_left = 0
            dronito.velocity_front = 0
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
    global pos_estimate_before_y, state_obs, first_detection, no_detection, from_front, from_back, obstacle_at_left, obstacle_at_right


    if (is_close(multiranger_front) and (not from_back) and (dronito.case==dronito.state_zigzag['forward1'] or dronito.case==dronito.state_zigzag['start'] or dronito.case==dronito.state_zigzag['forward2'] )) :
        print('state_obs=1 front')
        print(multiranger_front)
        from_front = 1
        if (state_obs==1) :
            pos_estimate_before_y = dronito.est_y #y pos left/right
        if abs(pos_estimate_before_y) > abs(pos_estimate_before_y - (dronito.boxborder_left)): # if distance plus grande a droite ? 
            obstacle_at_left =1
            dronito.velocity_left = - VELOCITY
        else :
            obstacle_at_right =1
            dronito.velocity_left = VELOCITY
        dronito.velocity_front = 0
        state_obs= 2
        return True

    if (is_close(multiranger_back) and (not from_front) and (dronito.case==dronito.state_zigzag['arrived'])): #jamais pour le moment juste arrived
        print('state_obs=1 back')
        from_back = 1
        print(multiranger_back)
        if (state_obs==1) :
            pos_estimate_before_y = dronito.est_y #y pos left/right
        if (abs(pos_estimate_before_y) > abs(pos_estimate_before_y - dronito.boxborder_left)): # if distance plus grande a droite ? 
            obstacle_at_left = 1
            dronito.velocity_left = - VELOCITY
        else :
            obstacle_at_right =1
            dronito.velocity_left = VELOCITY
        dronito.velocity_front = 0
        state_obs= 2
        return True
    
    if (state_obs== 2): #state_obs2
        print('state_obs=2')
        dronito.velocity_left = 0.0
        if (from_front):
            dronito.velocity_front = VELOCITY
            # dist = 0.1
        if (from_back):
            dronito.velocity_front = -VELOCITY
            # dist = -0.1
        if (obstacle_at_left):      
            if (first_detection):
                if (not(is_close(multiranger_left))): 
                    no_detection = no_detection + 1
                    if (no_detection >= SAFETY_MARGIN): # for safety
                        state_obs= 3
                        # mc.move_distance(dist, 0, 0, VELOCITY)
            if (is_close(multiranger_left)):
                first_detection =1
                dronito.velocity_left = - LOW_VELOCITY 
        if (obstacle_at_right):      
            if (first_detection):
                if (not(is_close(multiranger_right))): 
                    no_detection = no_detection + 1
                    if (no_detection >= SAFETY_MARGIN): # for safety
                        state_obs= 3
                        # mc.move_distance(dist, 0, 0, VELOCITY)
            if (is_close(multiranger_right)):
                first_detection =1
                dronito.velocity_left = LOW_VELOCITY  
        return True
        
    if (state_obs== 3): #state_obs3
        print('state_obs=3')
        if (obstacle_at_left):
            dronito.velocity_left = VELOCITY 
        else :
            dronito.velocity_left = - VELOCITY 
        dronito.velocity_front = 0
        if (dronito.est_y < abs(pos_estimate_before_y + SMALL_DIST)):
            print('fin state_obs3')
            if (is_close(multiranger_back)):
                dronito.velocity_left = 0
                dronito.velocity_front = VELOCITY
                return True
            elif (is_close(multiranger_front)):
                dronito.velocity_left = 0
                dronito.velocity_front = -VELOCITY
                return True
            state_obs= 1
            no_detection = 0
            first_detection = 0
            dronito.velocity_left = 0
            dronito.velocity_front = 0
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

