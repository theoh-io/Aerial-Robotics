import numpy as np
import cflib.crtp
from cflib.positioning.motion_commander import MotionCommander
from drone import Drone
import matplotlib.pyplot as plt
import time

VELOCITY_EDGE_GOAL = 0.1 #before 0.1
DELTA_X = 0.1
DELTA_Y = 0.25 
#dronito.delta_x
#dronito.delta_y

def is_edge(logs,first_edge):

    if first_edge == True:
        MIN_EDGE = 140  #in mm for Velocity=0.5
    else:
        MIN_EDGE= 50    #in mm for Velocity=0.2

    logs_copy2=logs[~np.all(logs == 0, axis=1)]

    if len(logs_copy2) > 100:        
        z_2=np.max(logs_copy2[-100:,3])
        idx_2=np.argmax(logs_copy2[-100:,3])
        z_1=np.min(logs_copy2[-100:,3])
        idx_1=np.argmin(logs_copy2[-100:,3])
        x1=logs_copy2[len(logs_copy2)-100+idx_1,0]/1000
        y1=logs_copy2[len(logs_copy2)-100+idx_1,1]/1000

        if abs(z_1-z_2) > MIN_EDGE:
            print('Abs edge: ', abs(z_1-z_2))
            return True, x1, y1
        else:
            return False, 0, 0
    else:
        print('Not enough data')
        return False, 0, 0

def find_platform_center(logs, dronito):

    x1=dronito.x_edge
    y1=dronito.y_edge
    
    if dronito.case == dronito.state_zigzag["right"]:
        dronito.mc.right(0.40, velocity=0.4)
        time.sleep(1)

    if dronito.case == dronito.state_zigzag["left"]:
        dronito.mc.left(0.40, velocity=0.4)
        time.sleep(1)
    
    #logs = np.zeros([100000,4])
    dronito.mc.back(0.45, velocity=0.3)
    time.sleep(1)

    if dronito.case == dronito.state_zigzag["right"]:
        dronito.mc.start_left(velocity=0.3)
        print(y1)
        print(dronito.est_y)

        while(dronito.est_y<(y1-DELTA_Y)):
            print('going left ',dronito.est_y,' ',y1)

    if dronito.case == dronito.state_zigzag["left"]:
        dronito.mc.start_right(velocity=0.3)
        print(y1)
        print(dronito.est_y)
        while(dronito.est_y>y1+DELTA_Y):
            print('going right ',dronito.est_y,' ',y1)

    dronito.mc.start_forward(velocity=0.2)


    print('going forward')
    dronito.edge=False

    while(dronito.edge == False):
        [dronito.edge,dronito.x_edge,dronito.y_edge]=is_edge(logs,first_edge=False)
        if (dronito.edge==True):
            print('Edge 2 detected!')

            x2=dronito.x_edge
            y2=dronito.y_edge
            
        if dronito.est_x > dronito.boxborder_front:
            print("No center found, limite arene x reached, let's land for safety")
            dronito.goal_x=dronito.est_x
            dronito.goal_y=dronito.est_y
            dronito.yaw_landing=dronito.est_yaw
            if dronito.verbose is True:
                print("yaw before landing", dronito.yaw_landing)
            dronito.edge=False
            return
    
    dronito.mc.forward(DELTA_X, velocity=VELOCITY_EDGE_GOAL)
    #time.sleep(1)

    dronito.goal_x=dronito.est_x
    dronito.goal_y=dronito.est_y
    dronito.yaw_landing=dronito.est_yaw
    if dronito.verbose is True:
        print("yaw before landing", dronito.yaw_landing)
    
    dronito.edge=False

    dX=0.15
    dY=0
    x0=x2+dX
    y0=y2+dY
    print('x1: ',x1,'y1: ',y1)
    print('x2: ',x2,'y2: ',y2)
    print('x0: ',x0,'y0: ',y0)
    print('goal_x: ',dronito.goal_x,'goal_y: ',dronito.goal_y)

    plt.figure()
    plt.axis('equal')
    plt.plot(logs[:,0]/1000,logs[:,1]/1000)
    plt.scatter([x1,x2,dronito.goal_x],[y1,y2,dronito.goal_y])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x2',(x2,y2))
    plt.annotate('x0',(x0,y0))
    plt.annotate('goal',(dronito.goal_x,dronito.goal_y))
    rectangle = plt.Rectangle((x0-0.15,y0-0.15), 0.30, 0.30,fill=None)
    plt.gca().add_patch(rectangle)
    plt.savefig('platform center')

def find_platform_center2(logs, dronito):

    x1=dronito.x_edge
    y1=dronito.y_edge

    if dronito.case2 == dronito.state_zigzag["right"]:
        dronito.mc.right(0.4, velocity=0.4)
        time.sleep(1)
        dronito.mc.forward(0.45, velocity=0.3)
        time.sleep(1)

    if dronito.case2 == dronito.state_zigzag["left"]:
        dronito.mc.left(0.4, velocity=0.4)
        time.sleep(1)
        dronito.mc.forward(0.45, velocity=0.3)
        time.sleep(1)
    
    if (dronito.case2 == dronito.state_zigzag["forward1"]) or (dronito.case2 == dronito.state_zigzag["forward2"]):
        dronito.mc.back(0.4, velocity=0.4)
        time.sleep(1)
        dronito.mc.left(0.45, velocity=0.3)
        time.sleep(1)
    
    #logs = np.zeros([100000,4])

    if dronito.case2 == dronito.state_zigzag["right"]:
        dronito.mc.start_left(velocity=0.3)
        print(y1)
        print(dronito.est_y)

        while(dronito.est_y<(y1-DELTA_Y)):
            print('going left ',dronito.est_y,' ',y1)
        
        x2_before=dronito.est_x
        y2_before=dronito.est_y

        dronito.mc.start_back(velocity=0.2)
        print('going back')

        dronito.edge=False

    if dronito.case2 == dronito.state_zigzag["left"]:
        dronito.mc.start_right(velocity=0.3)
        print(y1)
        print(dronito.est_y)
        while(dronito.est_y>y1+DELTA_Y):
            print('going right ',dronito.est_y,' ',y1)
        x2_before=dronito.est_x
        y2_before=dronito.est_y

        dronito.mc.start_back(velocity=0.2)
        print('going back')

        dronito.edge=False

    if (dronito.case2 == dronito.state_zigzag["forward1"]) or (dronito.case2 == dronito.state_zigzag["forward2"]):
        dronito.mc.start_forward(velocity=0.3)
        print(x1)
        print(dronito.est_x)
        while(dronito.est_x<x1-DELTA_Y):
            print('going forward ',dronito.est_x,' ',y1)
        x2_before=dronito.est_x
        y2_before=dronito.est_y

        dronito.mc.start_right(velocity=0.2)
        print('going right')

        dronito.edge=False

    while(dronito.edge == False):
        [dronito.edge,dronito.x_edge,dronito.y_edge]=is_edge(logs,first_edge=False)
        if (dronito.edge==True):
            x2=dronito.x_edge
            y2=dronito.y_edge
            
        if dronito.est_x > dronito.boxborder_front:
            print("No center found, limite arene x reached, let's land for safety")
            dronito.goal_x=dronito.est_x
            dronito.goal_y=dronito.est_y
            dronito.yaw_landing=dronito.est_yaw
            if dronito.verbose is True:
                print("yaw before landing", dronito.yaw_landing)
            dronito.edge=False
            return
    
    if (dronito.case2 == dronito.state_zigzag["right"]) or (dronito.case2 == dronito.state_zigzag["left"]):
        dronito.mc.back(DELTA_X, velocity=VELOCITY_EDGE_GOAL)
        #time.sleep(1)
   
    if (dronito.case2 == dronito.state_zigzag["forward1"]) or (dronito.case2 == dronito.state_zigzag["forward2"]):
        dronito.mc.right(DELTA_X, velocity=VELOCITY_EDGE_GOAL)
        #time.sleep(1)

    dronito.goal_x=dronito.est_x
    dronito.goal_y=dronito.est_y
    dronito.yaw_landing=dronito.est_yaw
    if dronito.verbose is True:
        print("yaw before landing", dronito.yaw_landing)
    
    dronito.edge=False

    dX=0.15
    dY=0
    x0=x2+dX
    y0=y2+dY
    print('x1: ',x1,'y1: ',y1)
    print('x2: ',x2,'y2: ',y2)
    print('x0: ',x0,'y0: ',y0)
    print('goal_x: ',dronito.goal_x,'goal_y: ',dronito.goal_y)

    plt.figure()
    plt.axis('equal')
    plt.plot(logs[:,0]/1000,logs[:,1]/1000)
    plt.scatter([x1,x2_before,x2,x0,dronito.goal_x],[y1,y2_before,y2,y0,dronito.goal_y])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x2_before',(x2_before,y2_before))
    plt.annotate('x2',(x2,y2))
    plt.annotate('x0',(x0,y0))
    plt.annotate('goal',(dronito.goal_x,dronito.goal_y))
    rectangle = plt.Rectangle((x0-0.15,y0-0.15), 0.30, 0.30,fill=None)
    plt.gca().add_patch(rectangle)
    plt.savefig('platform center')



