import time


# Unit: meter
DEFAULT_HEIGHT = 0.5 #1

BOX_LIMIT_X = 2 #5
BOX_LIMIT_Y = 0.7 #3

#START_POS_X = 0
#START_POS_Y = 0
GOAL_ZONE_X= 0.8
TIME_EXPLORE= 7
#variables needed for obstacle avoidance
VELOCITY = 0.2
EPSYLON=0.0001

class Drone():
    def __init__(self, mc, start_x, start_y, time_explore, x_offset=0.25, verbose=True):
        self.verbose=verbose
        self.mc = mc

        #estimation attributes
        self.est_x=0
        self.est_y=0
        self.est_z=0
        self.z_range=0
        self.est_yaw=0
        #start position attributes
        self.start_x=start_x
        self.start_y=start_y
        #goal position attributes
        self.goal_x=0
        self.goal_y=0
        #Box dimensions attributes
        # self.global_frame=False
        # self.boxborder_left=BOX_LIMIT_Y
        # self.boxborder_right=0
        # self.boxborder_front=BOX_LIMIT_X
        # self.boxborder_back=0
        # self.dist_explore=GOAL_ZONE_X

        self.boxborder_left=BOX_LIMIT_Y-start_y
        self.boxborder_right=-start_y
        self.boxborder_front=BOX_LIMIT_X-start_x
        self.boxborder_back=-start_x
        self.dist_explore=GOAL_ZONE_X-start_x
        #zigzag attributes
        self.state_zigzag={'start':-1, 'left':0, 'forward1':1, 'right':2, 'forward2':3, 'back2left':4, 'arrived':5}
        self.case=self.state_zigzag["start"]
        self.start_time=time.time()
        self.x_offset=x_offset
        #attributes freq reg yaw
        self.time_yaw=0 #to ompare with timestep
        self.timestep=1

        #attributes takeoff 2
        self.yaw_landing=0
        self.landed=0

        #attributes edge detection
        self.edge=0

        #temporary
        self.time_explore=time_explore


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
        #update estimation after being reinitialized
        if (self.goal_x==0 and self.goal_y==0):
            print("goal are not set => not able ")
        else:
            self.est_x=self.goal_x+x
            self.est_y=self.goal_y+y
            self.est_z=z
            self.z_range=z_range
            self.est_yaw=yaw


    def zigzag(self):
        #print(state_zigzag['start'])
        #self.freq_yaw_reg() #function called at every given timestep to regulate the yaw

        if self.case==self.state_zigzag["start"]:
            self.mc.start_forward()
            print('start')
        if (self.case==self.state_zigzag["start"] and self.est_x>self.dist_explore) or self.case == self.state_zigzag["back2left"]:
            self.regulate_yaw(0, self.est_yaw)
            self.case=self.state_zigzag["left"]
            self.mc.start_left()
            print('left')
        elif (self.case==self.state_zigzag["left"] and self.est_y > self.boxborder_left) :
            self.regulate_yaw(0, self.est_yaw)
            self.case=self.state_zigzag["forward1"]
            print('forward 1')
            self.mc.forward(self.x_offset)
        elif self.case == self.state_zigzag["forward1"]:
            self.regulate_yaw(0, self.est_yaw)
            self.case=self.state_zigzag["right"]
            print('right')
            self.mc.start_right()
        elif self.case == self.state_zigzag["right"] and  self.est_y < self.boxborder_right:
            self.regulate_yaw(0, self.est_yaw)
            self.case = self.state_zigzag["forward2"]
            print('forward 2')
            self.mc.forward(self.x_offset)
            self.case=self.state_zigzag["back2left"]
            print('back2left')

        #Temporaire: condition d'arret si la limite de l'arene en x 
        if self.est_x > self.boxborder_front:
            print("Seulement un edge détecté, pas le deuxième, limite arene x reached, let's land for safety")
            self.mc.land()
            time.sleep(1)
            self.case =self.state_zigzag["arrived"]
        
        #Temporaire condition de retour basé sur le temps de vol
        if(time.time()-self.start_time>TIME_EXPLORE):
            self.goal_reached()
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

        #FIXME need to be implemented outside of drone class
        if (self.edge == True and (self.case != self.state_zigzag["start"]) and (self.case != self.state_zigzag["arrived"]) ):
            print('Edge far detected!')
            #yaw_landing=position_estimate[2]
            #must record goal pos before landing because variation can occur
            #goal_x=self.est_x
            #goal_y=self.est_y
            self.edge=1
            #find_platform_center()
            self.mc.land()
            #time.sleep(1)
            #self.mc.take_off(DEFAULT_HEIGHT)
            #clean_takeoff(self.mc, [goal_x, goal_y, yaw_landing])
            #self.case =state_zigzag["arrived"] #to get out of zigzag

    def is_arrived(self):
        if self.case == self.state_zigzag['arrived']:
            return True
        return False

    def is_starting(self):
        if self.case == self.state_zigzag['start']:
            return True
        return False

    def go_back(self):
        #first goes to 0 in x
        dist_x=self.goal_x
        self.mc.back(dist_x)
        #goes to 0 in y
        dist_y=self.goal_y
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
    

    def clean_takeoff(self): 
        #au début pas de coordonnées initiales
        time.sleep(0.1)
        #nécéssaire ???
        #self.mc._reset_position_estimator()
        curr_x = self.start_x+self.est_x
        curr_y = self.start_y+self.est_y
        #init_yaw = 0    
        # print("Start pos (x, y):", curr_x, curr_y)
        # print("Start yaw:", self.est_yaw)
        # self.regulate_x(self.start_x, curr_x)
        # time.sleep(1)
        # self.regulate_y(self.start_y, curr_y)
        # time.sleep(1)
        self.regulate_yaw(0, self.est_yaw)
        time.sleep(1)
        #apres le landing on veut controler la position après le redécollage

    def clean_takeoff2(self):
        print("in clean takeoff2")
        time.sleep(1)
        # curr_x = self.est_x
        # curr_y = self.est_y
        #self.landed=1 #flag to change the estimation function
        print("current pos (x, y, yaw):", self.est_x, self.est_y, self.est_yaw)
        print("before landing pos (x, y, yaw):", self.goal_x, self.goal_y, self.yaw_landing)
        #regulate_x(self.mc, init_coord[0], curr_x)
        #regulate_y(mc, init_coord[1], curr_y)
        self.regulate_yaw(self.yaw_landing, self.est_yaw)


    def goal_reached(self):
        #when goal reached needs to: 
        #record goal position before landing
        #land
        #take off
        #update flag for using 2 estimation function
        #switch state_zigzag

        self.yaw_landing=self.est_yaw
        if self.verbose is True:print("yaw before landing", self.yaw_landing)
        self.goal_x=self.est_x
        self.goal_y=self.est_y
        self.mc.land()
        self.landed=1 #used to decide which position estimation function to use
        time.sleep(1)
        self.mc.take_off(DEFAULT_HEIGHT)
        self.clean_takeoff2()
        #clean_takeoff(self.mc, [goal_x, goal_y, yaw_landing])
        self.case =self.state_zigzag["arrived"] #to get out of zigzag
        

        
            