


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
 

# obstacle detection with MIN_DISTANCE to obstacle
def is_close(range):
    MIN_DISTANCE = 0.5  # m
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

# go around obstacles in motion direction detected at left or right.
def  obstacle_avoid_left_right(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito):
    global pos_estimate_before_x, state_obs, first_detection, no_detection, from_left, from_right, pos_estimate_before_y, obstacle_at_front, obstacle_at_back
    
    #left state 1 : smartly go front or back wehen obstacle detected at front
    if (is_close(multiranger_left) and (not (from_right)) and dronito.case==dronito.state_zigzag['left'] or (dronito.case==dronito.state_zigzag['arrived'] and dronito.case2== dronito.state_zigzag['left'])): 
        print('state_obs=1 left') 
        print(multiranger_left)
        print(is_close(multiranger_left))
        from_left = 1

        #save position when obstacle is first detected 
        if (state_obs==1) :
            pos_estimate_before_x = dronito.est_x 
            pos_estimate_before_y = dronito.est_y 
        
        #continue when bstacle very close to y border from left
        if (abs(pos_estimate_before_y - (dronito.boxborder_left)) < dronito.thresh_y): 
            print("too close to border")
            if (dronito.case==dronito.state_zigzag['arrived']):
                dronito.case2=dronito.state_zigzag["forward1"]
                from_left = 0
                # dronito.start_forward=dronito.est_x
                # dronito.mc.start_back()
            else:
                dronito.case=dronito.state_zigzag["forward1"]
                from_left = 0
                # dronito.start_forward=dronito.est_x
                # dronito.mc.start_forward()
            return False  

        #when back safer than front go back
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (dronito.boxborder_front )): 
            obstacle_at_front = 1
            dronito.velocity_front = - dronito.vel_obst
        else :
            obstacle_at_back = 1
            dronito.velocity_front = dronito.vel_obst
        dronito.velocity_left = 0.0
        state_obs= 2
        return True

    #right state 1 : smartly go front or back wehen obstacle detected at front
    if (is_close(multiranger_right)  and (not (from_left)) and dronito.case==dronito.state_zigzag['right'] or (dronito.case==dronito.state_zigzag['arrived'] and dronito.case2== dronito.state_zigzag['right'])): #right state 
        print('state_obs=1 right') 
        print(multiranger_left)
        print(is_close(multiranger_left))
        from_right = 1
        
        #save position when obstacle is first detected 
        if (state_obs==1) :
            pos_estimate_before_x = dronito.est_x
            pos_estimate_before_y = dronito.est_y
        
        #obstacle very close to y border from right
        if (abs(pos_estimate_before_y) < dronito.thresh_y): 
            print("too close to border")
            if (dronito.case==dronito.state_zigzag['arrived']):
                dronito.case2=dronito.state_zigzag["forward2"]
                from_right = 0
                dronito.start_forward=dronito.est_x
                # dronito.mc.start_back()
            else:
                dronito.case=dronito.state_zigzag["forward2"]
                from_right = 0
                dronito.start_forward=dronito.est_x
                # dronito.mc.start_forward()
            return False  

        #when back safer than front go back
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (dronito.boxborder_front)): 
            obstacle_at_front = 1
            dronito.velocity_front = - dronito.vel_obst
        else :
            obstacle_at_back = 1
            dronito.velocity_front = dronito.vel_obst
        dronito.velocity_left = 0.0
        state_obs= 2
        return True
    
    #left/right state 2 : advance in intial motion direction front/back untill obstacle is no more detected around
    if (state_obs== 2): 
        print('state_obs=2')
        dronito.velocity_front = 0.0
        if (from_right):
            dronito.velocity_left = -dronito.vel_obst
            
        if (from_left):
            dronito.velocity_left = +dronito.vel_obst 
            
        if(obstacle_at_back):
            print('obstacle_at_back')
            if (first_detection):
                print('first_detection')
                if (not(is_close(multiranger_back))):
                    print('not close back')
                    no_detection = no_detection + 1
                    if (no_detection >= dronito.margin): # for safety
                        state_obs= 3
                        
            if (is_close(multiranger_back)):
                first_detection =1
                dronito.velocity_front = dronito.low_vel_obst
        if(obstacle_at_front):
            print('obstacle_at_front')
            if (first_detection):
                print('first_detection')
                if (not(is_close(multiranger_front))): 
                    print('not close front')
                    no_detection = no_detection + 1
                    if (no_detection >= dronito.margin): # for safety
                        state_obs= 3
                        
            if (is_close(multiranger_front)):
                first_detection =1
                dronito.velocity_front = - dronito.low_vel_obst
                
        return True

    #left/right state 3: when obstacle avoided go front/back untill it's possible to retake intial trajectory     
    if (state_obs== 3): 
        print('state_obs=3')
        dronito.velocity_left = 0
        if abs(pos_estimate_before_x) > abs(pos_estimate_before_x - (dronito.boxborder_front)): 
            dronito.velocity_front = dronito.vel_obst 
        else :
            dronito.velocity_front = - dronito.vel_obst 
        print(dronito.est_x, pos_estimate_before_x )
        if (dronito.est_x > abs(pos_estimate_before_x - dronito.small_dist) and (dronito.est_x < abs(pos_estimate_before_x + dronito.small_dist))):
            print('fin state_obs3')
            if (is_close(multiranger_right)):
                ('right close going left')
                dronito.velocity_left = dronito.vel_obst
                dronito.velocity_front = 0
                return True 
            elif (is_close(multiranger_left)):
                ('left close going right')
                dronito.velocity_left = -dronito.vel_obst
                dronito.velocity_front = 0
                return True
            # reinitialize all variables
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
        return True  
    return False

# go around obstacles in motion direction detected at front or back.
def  obstacle_avoid_front_back(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito):
    global pos_estimate_before_y, state_obs, first_detection, no_detection, from_front, from_back, obstacle_at_left, obstacle_at_right

    #front state 1 : smartly go left or right when obstacle is detected at front
    if (is_close(multiranger_front) and (not from_back) and (dronito.case==dronito.state_zigzag['forward1'] or dronito.case==dronito.state_zigzag['start'] or dronito.case==dronito.state_zigzag['forward2'] )) :
        print('state_obs=1 front')
        print(multiranger_front)
        from_front = 1
        if (state_obs==1) :
            pos_estimate_before_y = dronito.est_y #y pos left/right
        if abs(pos_estimate_before_y) > abs(pos_estimate_before_y - (dronito.boxborder_left)): # if distance plus grande a droite ? 
            obstacle_at_left =1
            dronito.velocity_left = - dronito.vel_obst
        else :
            obstacle_at_right =1
            dronito.velocity_left = dronito.vel_obst
        dronito.velocity_front = 0
        state_obs= 2
        return True

    #back state 1 : smartly go left or right wehen obstacle is detected at back
    if (is_close(multiranger_back) and (not from_front) and (dronito.case==dronito.state_zigzag['arrived'])): 
        print('state_obs=1 back')
        from_back = 1
        print(multiranger_back)
        if (state_obs==1) :
            pos_estimate_before_y = dronito.est_y #y pos left/right
        if (abs(pos_estimate_before_y) > abs(pos_estimate_before_y - dronito.boxborder_left)): 
            obstacle_at_left = 1
            dronito.velocity_left = - dronito.vel_obst
        else :
            obstacle_at_right =1
            dronito.velocity_left = dronito.vel_obst
        dronito.velocity_front = 0
        state_obs= 2
        return True
    

    #front/back state 2 : advance in intial motion direction front/back untill obstacle is no more detected around
    if (state_obs== 2): #state_obs2
        print('state_obs=2')
        dronito.velocity_left = 0.0
        if (from_front):
            dronito.velocity_front = dronito.vel_obst
            # dist = 0.1
        if (from_back):
            dronito.velocity_front = -dronito.vel_obst
            # dist = -0.1
        if (obstacle_at_left):      
            if (first_detection):
                if (not(is_close(multiranger_left))): 
                    no_detection = no_detection + 1
                    if (no_detection >= dronito.margin): # for safety
                        state_obs= 3
                        # mc.move_distance(dist, 0, 0, dronito.vel_obst)
            if (is_close(multiranger_left)):
                first_detection =1
                dronito.velocity_left = - dronito.low_vel_obst 
        if (obstacle_at_right):      
            if (first_detection):
                if (not(is_close(multiranger_right))): 
                    no_detection = no_detection + 1
                    if (no_detection >= dronito.margin): # for safety
                        state_obs= 3
                        # mc.move_distance(dist, 0, 0, dronito.vel_obst)
            if (is_close(multiranger_right)):
                first_detection =1
                dronito.velocity_left = dronito.low_vel_obst  
        return True
    
    #front/back state 3: when obstacle avoided go left/right untill it's possible to retake intial trajectory            
    if (state_obs== 3): #state_obs3
        print('state_obs=3')
        if (obstacle_at_left):
            print("obstacle_at_left")
            dronito.velocity_left = dronito.vel_obst 
        else :
            print("obstacle_at_right")
            dronito.velocity_left = - dronito.vel_obst 
        dronito.velocity_front = 0
        print(dronito.est_y, pos_estimate_before_y )
        if ((dronito.est_y > abs(pos_estimate_before_y - dronito.small_dist)) and (dronito.est_y < abs(pos_estimate_before_y + dronito.small_dist))):
            print('fin state_obs3')
            if (is_close(multiranger_back)):
                dronito.velocity_left = 0
                dronito.velocity_front = dronito.vel_obst
                return True
            elif (is_close(multiranger_front)):
                dronito.velocity_left = 0
                dronito.velocity_front = -dronito.vel_obst
                return True
            # reinitialize all variables
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
        return True  
    return False

# obstacle avoidance function always called in the while loop : returns true if obtsacle in motion direction  
def obstacle_avoidance(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito):

    if ((is_close(multiranger_left) or is_close(multiranger_right) or from_left or from_right) and (from_front ==0) and (from_back == 0)):
        return obstacle_avoid_left_right(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito)
    elif ((is_close(multiranger_front) or is_close(multiranger_back) or from_front or from_back) and (from_right ==0) and (from_left == 0)): 
        return obstacle_avoid_front_back(multiranger_left, multiranger_right, multiranger_front, multiranger_back, dronito)
    return False

