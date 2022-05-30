import time


#Threshold for Comparision
EPSYLON=0.0001


class Drone():
    def __init__(self, mc, args, verbose=True):
        self.verbose=verbose
        self.mc = mc
        #velocity attributes
        # self.vel_takeoff=args.vel_takeoff
        # self.vel_landing=args.vel_landing
        #estimation attributes
        self.est_x=0
        self.est_y=0
        self.est_z=0
        self.z_range=0
        self.est_yaw=0
        #start position attributes
        self.start_x=args.startX
        self.start_y=args.startY
        #goal position attributes
        self.goal_x=0
        self.goal_y=0
        #Box dimensions attributes
        self.boxborder_left=args.arenaY
        self.boxborder_right=0
        self.boxborder_front=args.arenaX
        self.boxborder_back=0
        self.dist_explore=args.goal_zone
        self.dist_explore2=args.start_zone
        #zigzag attributes
        self.state_zigzag={'start':-1, 'left':0, 'forward1':1, 'right':2, 'forward2':3, 'back2left':4, 'arrived':5}
        self.case=self.state_zigzag["start"]
        self.case2=self.state_zigzag["start"]
        self.start_time=time.time()
        self.start_time2=0
        self.start_forward=0
        self.start_back=0
        self.x_offset=args.x_offset
        ##temporary attributes
        self.time_explore=args.time_exp
        self.time_explore2=args.time_exp_2
        self.time_explore_box=args.time_exp_box
        self.thresh_back=args.thresh_back

        #attributes: back_searching
        self.zone_x=args.box_x
        self.zone_y=args.box_y
        self.pos_x=0
        self.pos_y=0
        self.x_reached=0
        self.y_reached=0

        #attributes freq reg yaw
        self.time_yaw=0 #to ompare with timestep
        self.timestep=1

        #attributes takeoff 2
        self.yaw_landing=0
        self.landed=0

        #attributes edge detection
        self.edge=0
        self.x_edge=0
        self.y_edge=0
        self.delta_x=args.delta_x
        self.delta_y=args.delta_y
        self.vel_edge=args.vel_edge
        self.vel_edge_goal=args.vel_edge_goal

        #attributes obs avoidance
        self.velocity_left = 0
        self.velocity_front = 0
        self.vel_obst=args.vel_obst
        self.low_vel_obst=args.low_vel_obst
        self.thresh_y=args.thresh_y
        #self.min_dist=args.min_dist
        self.margin=args.margin
        self.small_dist=args.small_dist
        

    def update_est(self, x, y, z, z_range, yaw):
        #estimation in local frame
        self.est_x=x
        self.est_y=y
        self.est_z=z
        self.z_range=z_range
        self.est_yaw=yaw

    def update_est_global(self, x, y, z, z_range, yaw):
        #estimation in global frame
        self.est_x=x+self.start_x
        self.est_y=y+self.start_y
        self.est_z=z
        self.z_range=z_range
        self.est_yaw=yaw
    
    def get_est(self):
        return self.est_x, self.est_y, self.est_z, self.z_range, self.est_yaw

    def update_goal(self, goal_x, goal_y):
        self.goal_x=goal_x
        self.goal_y=goal_y

    def update_est2(self, x, y, z, z_range, yaw):
        #update estimation after being reinitialized after landing
        if (self.goal_x==0 and self.goal_y==0):
            print("goal are not set => not able ")
        else:
            self.est_x=self.goal_x+x
            self.est_y=self.goal_y+y
            self.est_z=z
            self.z_range=z_range
            self.est_yaw=yaw


    def zigzag(self):
        #FSM to implement zigzag behavior in the exploration zone (non blocking movements)
        if self.case==self.state_zigzag["start"]:
            self.mc.start_forward()
            #print('start')
        if (self.case==self.state_zigzag["start"] and self.est_x>self.dist_explore) or (self.case == self.state_zigzag["back2left"] and self.est_x > self.start_forward+self.x_offset):
            self.regulate_yaw(0, self.est_yaw)
            self.case=self.state_zigzag["left"]
            self.mc.start_left()
            #print('left')
        elif (self.case==self.state_zigzag["left"] and self.est_y > self.boxborder_left) :
            self.regulate_yaw(0, self.est_yaw)
            self.case=self.state_zigzag["forward1"]
            #print('forward 1')
            self.start_forward=self.est_x
            self.mc.start_forward(self.x_offset)
        elif self.case == self.state_zigzag["forward1"] and self.est_x > self.start_forward+self.x_offset:
            self.regulate_yaw(0, self.est_yaw)
            self.case=self.state_zigzag["right"]
            #print('right')
            self.mc.start_right()
        elif self.case == self.state_zigzag["right"] and  self.est_y < self.boxborder_right:
            self.regulate_yaw(0, self.est_yaw)
            self.case = self.state_zigzag["forward2"]
            #print('forward 2')
            self.start_forward=self.est_x
            self.mc.start_forward()
            self.case=self.state_zigzag["back2left"]
            #print('back2left')

        #Stop Condition if box limit reached in X 
        if self.est_x > self.boxborder_front:
            print("Pas de plateforme, limite box atteinte en x")
            time.sleep(1)
            self.goal_x=self.est_x
            self.goal_y=self.est_y
            self.yaw_landing=self.est_yaw
            if self.verbose is True:
                print("yaw before landing", self.yaw_landing)
            self.case =self.state_zigzag["arrived"]
            self.goal_reached()
        
        #Temporaire condition de retour basé sur le temps de vol
        #if(time.time()-self.start_time>self.time_explore):
        #    print("stop due au timing")
        #    self.goal_reached()
            # print(" Exploration time exceeded")
            # self.yaw_landing=self.est_yaw
            # print("yaw during landing", self.yaw_landing)
            # #must record goal pos before landing because variation can occur
            # self.goal_x=self.est_x
            # self.goal_y=self.est_y
            # self.mc.land()
            # time.sleep(1)
            # self.mc.take_off(DEFAULT_HEIGHT)
            # self.clean_takeoff2()
            # self.case =self.state_zigzag["arrived"] #to get out of zigzag

    def is_arrived(self):
        if self.case == self.state_zigzag['arrived']:
            return True
        return False
    
    def is_arrived2(self):
        if self.case2 == self.state_zigzag['arrived']:
            return True
        return False


    def is_starting(self):
        if self.case == self.state_zigzag['start']:
            return True
        return False
    
    def is_starting2(self):
        if self.case2 == self.state_zigzag['start']:
            return True
        return False

    def zigzag_back(self):
        #FSM to implement the same zigzag behavior on the way back
        if self.case2==self.state_zigzag["start"]:
            self.mc.start_back()
            #print('start_zz2')
        if (self.case2==self.state_zigzag["start"] and self.est_x<self.dist_explore2) or (self.case2 == self.state_zigzag["back2left"] and self.est_x < self.start_back-self.x_offset):
            #print("start zzback")
            self.regulate_yaw(0, self.est_yaw)
            self.case2=self.state_zigzag["left"]
            self.mc.start_left()
            #print('left')
        elif (self.case2==self.state_zigzag["left"] and self.est_y > self.boxborder_left) :
            self.regulate_yaw(0, self.est_yaw)
            self.case2=self.state_zigzag["forward1"]
            #print('forward 1')
            self.start_back=self.est_x
            self.mc.start_back()
        elif (self.case2 == self.state_zigzag["forward1"] and self.est_x < self.start_back-self.x_offset):
            self.regulate_yaw(0, self.est_yaw)
            self.case2=self.state_zigzag["right"]
            #print('right')
            self.mc.start_right()
        elif (self.case2 == self.state_zigzag["right"] and  self.est_y < self.boxborder_right):
            self.regulate_yaw(0, self.est_yaw)
            self.case2 = self.state_zigzag["forward2"]
            #print('forward 2')
            self.start_back=self.est_x
            self.mc.start_back()
            self.case2=self.state_zigzag["back2left"]
            #print('back2left')

        #Stop condition based on the limit of the Arena
        if self.est_x < 0-self.thresh_back:
            #print("Pas de platforme, limite de la box atteinte au retour")
            time.sleep(1)
            self.goal_x=self.est_x
            self.goal_y=self.est_y
            self.yaw_landing=self.est_yaw
            if self.verbose is True:
                print("yaw before landing", self.yaw_landing)
            self.case =self.state_zigzag["arrived"]
            self.goal_reached2()
        
        #Temporaire condition de retour basé sur le temps de vol
        #if(time.time()-self.start_time2>self.time_explore2):
        #    print("arret au retour basé sur le timing")
        #    self.goal_reached2()

    def zigzag_box(self):
        '''
        startbox_x, y position to start exploring
        limit_x,y box to explore inside
        '''
        
        if  self.case2==self.state_zigzag["start"] or self.case2 == self.state_zigzag["back2left"]:
            self.regulate_yaw(0, self.est_yaw)
            self.case2=self.state_zigzag["left"]
            self.mc.start_left()
            print('left')
        elif self.case2==self.state_zigzag["left"] and self.est_y > (self.pos_y+self.zone_y) :
            self.regulate_yaw(0, self.est_yaw)
            self.case2=self.state_zigzag["forward1"]
            print('forward 1')
            self.mc.back(self.x_offset)
        elif self.case2 == self.state_zigzag["forward1"]:
            self.regulate_yaw(0, self.est_yaw)
            self.case2=self.state_zigzag["right"]
            print('right')
            self.mc.start_right()
        elif self.case2 == self.state_zigzag["right"] and  self.est_y < (self.pos_y-self.zone_y):
            self.regulate_yaw(0, self.est_yaw)
            self.case2 = self.state_zigzag["forward2"]
            print('forward 2')
            self.mc.back(self.x_offset)
            self.case2=self.state_zigzag["back2left"]
            print('back2left')

        #Stop condition based on the limit of the Arena
        if self.est_x < self.pos_x-self.zone_x:
            print("Pas de platforme, exploration box terminée sans succès")
            time.sleep(1)
            self.goal_x=self.est_x
            self.goal_y=self.est_y
            self.yaw_landing=self.est_yaw
            if self.verbose is True:
                print("yaw before landing", self.yaw_landing)
            self.case =self.state_zigzag["arrived"]
            self.goal_reached2()
        
        # #Temporaire condition de retour basé sur le temps de vol
        # if(time.time()-self.start_time2>self.time_explore_box):
        #     time.sleep(1)
        #     self.goal_x=self.est_x
        #     self.goal_y=self.est_y
        #     self.yaw_landing=self.est_yaw
        #     if self.verbose is True:
        #         print("yaw before landing", self.yaw_landing)
        #     self.case =self.state_zigzag["arrived"]
        #     self.goal_reached2()
            

    def back_searching(self):
        #given goal position (initial) and start (objective) define a zone around to start searching for
        if((self.est_x - self.start_x)<self.zone_x):
            self.x_reached=1
        else:
            self.x_reached=0
        if((self.est_y - self.start_y)<self.zone_y):
            self.y_reached=1
        else:
            self.y_reached=0

        if(self.x_reached==0):
            self.mc.start_back()
        elif(self.y_reached==0):
            self.mc.start_right()
        else:
            #parametric zigzag to restrict to a specific zone
            if self.pos_x==0 and self.pos_y==0:
                self.pos_x=self.est_x
                self.pos_y=self.est_y
                print("starting of the box :", self.pos_x, self.pos_y)
            self.zigzag_box()

    def go_back(self):
        #first goes to 0 in x
        dist_x=self.goal_x-self.start_x
        self.mc.back(dist_x)
        #goes to 0 in y
        dist_y=self.goal_y-self.start_x
        self.mc.right(dist_y)
        self.mc.land()

    def regulate_x(self, init_x, curr_x):
        print("in regulate_x")
        error_x=curr_x-init_x
        if error_x>EPSYLON:
            self.mc.back(error_x)
        if error_x<-EPSYLON:
            self.mc.forward(-error_x)
        else:
            print("zero_error")

    def regulate_y(self, init_y, curr_y):
        print("in regulate_y")
        error_y=curr_y-init_y
        if error_y>EPSYLON:
            self.mc.right(error_y)
        if error_y<-EPSYLON:
            self.mc.left(-error_y)
        else:
            print("zero_error")

    # regulate yaw to init angle
    def regulate_yaw(self, init_yaw, curr_yaw):
        print("in regulate yaw")
        if init_yaw - curr_yaw >= EPSYLON:
            self.mc.turn_left(init_yaw - curr_yaw)
        if init_yaw - curr_yaw < -EPSYLON:
            self.mc.turn_right(curr_yaw - init_yaw)
        else:
            print("zero error")
    
    def freq_yaw_reg(self):
        if(time.time()-self.time_yaw>self.timestep):
            print("yaw before regulate:", self.est_yaw)
            self.regulate_yaw(0, self.est_yaw)
            print("yaw after regulate:", self.est_yaw)
            self.time_yaw=time.time()
    


    def clean_takeoff2(self):
        print("in clean takeoff2")
        time.sleep(3)
        # curr_x = self.est_x
        # curr_y = self.est_y
        print("current pos (x, y, yaw):", self.est_x, self.est_y, self.est_yaw)
        print("before landing pos (x, y, yaw):", self.goal_x, self.goal_y, self.yaw_landing)
        #regulate_x(self.mc, init_coord[0], curr_x)
        #regulate_y(mc, init_coord[1], curr_y)
        self.regulate_yaw(self.yaw_landing, self.est_yaw)


    def goal_reached(self):
        self.mc.land(velocity=self.vel_landing)
        self.case = self.state_zigzag["arrived"]
        self.landed=1 #used to decide which position estimation function to use
        time.sleep(5)
        self.mc._reset_position_estimator()
        self.mc.take_off()
        self.clean_takeoff2()
        
    def goal_reached2(self):
        print("final goal reached")
        self.mc.land()
        self.case2 = self.state_zigzag["arrived"]
        self.landed=1 #used to decide which position estimation function to use
        
        

        
            