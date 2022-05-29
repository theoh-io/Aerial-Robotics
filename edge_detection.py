import numpy as np
import cflib.crtp
from cflib.positioning.motion_commander import MotionCommander
from drone import Drone
import matplotlib.pyplot as plt
import time


def is_edge(logs):

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
            print('abs edge 2: ', abs(z_1-z_2))
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

def find_platform_center(logs, dronito):

    #x1=dronito.est_x
    #y1=dronito.est_y
    x1=dronito.x_edge
    y1=dronito.y_edge
    
    """
    x1_bis=dronito.est_x
    y1_bis=dronito.est_y

    print(x1,' ',x1_bis)
    print(y1,' ',y1_bis)

    plt.figure()
    plt.axis('equal')
    plt.scatter([x1,x1_bis],[y1,y1_bis])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x1_bis',(x1_bis,y1_bis))
    plt.savefig('first edge')
    """

    if dronito.case == dronito.state_zigzag["right"]:
        dronito.mc.right(0.40)
        time.sleep(1)

    if dronito.case == dronito.state_zigzag["left"]:
        dronito.mc.left(0.40)
        time.sleep(1)

    """
    while(edge == True):
        print('still close')
        dronito.edge= is_edge_2()[0]
    """
    
    #logs = np.zeros([100000,4])
    dronito.mc.back(0.4)

    time.sleep(1)

    if dronito.case == dronito.state_zigzag["right"]:
        dronito.mc.start_left()
        print(y1)
        print(dronito.est_y)

        while(dronito.est_y<(y1-0.25)):
            #pass
            print('going left ',dronito.est_y,' ',y1)
        
        x2_before=dronito.est_x
        y2_before=dronito.est_y

    if dronito.case == dronito.state_zigzag["left"]:
        dronito.mc.start_right()
        print(y1)
        print(dronito.est_y)
        while(dronito.est_y>y1+0.25):
            #pass
            print('going right ',dronito.est_y,' ',y1)
        x2_before=dronito.est_x
        y2_before=dronito.est_y

    dronito.mc.start_forward()


    print('going forward')
    dronito.edge=False

    while(dronito.edge == False):
        [dronito.edge,dronito.x_edge,dronito.y_edge]=is_edge(logs)
        if (dronito.edge==True):
            """
            print('Edge 2 detected!')
            x2_bis=dronito.est_x
            y2_bis=dronito.est_y
            """
            x2=dronito.x_edge
            y2=dronito.y_edge

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
            
        if dronito.est_x > dronito.boxborder_front - dronito.start_x:
            print("No center found, limite arene x reached, let's land for safety")
            dronito.goal_x=dronito.est_x
            dronito.goal_y=dronito.est_y
            dronito.yaw_landing=dronito.est_yaw
            if dronito.verbose is True:
                print("yaw before landing", dronito.yaw_landing)
            dronito.edge=False
            return
    
    #before working dronito.mc.forward(0.05)
    dronito.mc.forward(0.1, velocity=0.1)

    time.sleep(1)

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

def find_platform_center2(logs, dronito):

    #x1=dronito.est_x
    #y1=dronito.est_y
    x1=dronito.x_edge
    y1=dronito.y_edge
    
    """
    x1_bis=dronito.est_x
    y1_bis=dronito.est_y

    print(x1,' ',x1_bis)
    print(y1,' ',y1_bis)

    plt.figure()
    plt.axis('equal')
    plt.scatter([x1,x1_bis],[y1,y1_bis])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x1_bis',(x1_bis,y1_bis))
    plt.savefig('first edge')
    """

    if dronito.case2 == dronito.state_zigzag["right"]:
        dronito.mc.right(0.4)
        time.sleep(1)
        dronito.mc.forward(0.4)
        time.sleep(1)

    if dronito.case2 == dronito.state_zigzag["left"]:
        dronito.mc.left(0.4)
        time.sleep(1)
        dronito.mc.forward(0.4)
        time.sleep(1)
    
    if (dronito.case2 == dronito.state_zigzag["forward 1"]) or (dronito.case2 == dronito.state_zigzag["forward 2"]):
        dronito.mc.back(0.4)
        time.sleep(1)
        dronito.mc.left(0.4)
        time.sleep(1)
    
    #logs = np.zeros([100000,4])

    if dronito.case2 == dronito.state_zigzag["right"]:
        dronito.mc.start_left()
        print(y1)
        print(dronito.est_y)

        while(dronito.est_y<(y1-0.25)):
            #pass
            print('going left ',dronito.est_y,' ',y1)
        
        x2_before=dronito.est_x
        y2_before=dronito.est_y

        dronito.mc.start_back()
        print('going back')

        dronito.edge=False

    if dronito.case2 == dronito.state_zigzag["left"]:
        dronito.mc.start_right()
        print(y1)
        print(dronito.est_y)
        while(dronito.est_y>y1+0.25):
            #pass
            print('going right ',dronito.est_y,' ',y1)
        x2_before=dronito.est_x
        y2_before=dronito.est_y

        dronito.mc.start_back()
        print('going back')

        dronito.edge=False

    if (dronito.case2 == dronito.state_zigzag["forward 1"]) or (dronito.case2 == dronito.state_zigzag["forward 2"]):
        dronito.mc.start_forward()
        print(x1)
        print(dronito.est_x)
        while(dronito.est_x<x1-0.25):
            #pass
            print('going forward ',dronito.est_x,' ',y1)
        x2_before=dronito.est_x
        y2_before=dronito.est_y

        dronito.mc.start_right()
        print('going right')

        dronito.edge=False

    while(dronito.edge == False):
        [dronito.edge,dronito.x_edge,dronito.y_edge]=is_edge(logs)
        if (dronito.edge==True):
            """
            print('Edge 2 detected!')
            x2_bis=dronito.est_x
            y2_bis=dronito.est_y
            """
            x2=dronito.x_edge
            y2=dronito.y_edge

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
            
        if dronito.est_x > dronito.boxborder_front - dronito.start_x:
            print("No center found, limite arene x reached, let's land for safety")
            dronito.goal_x=dronito.est_x
            dronito.goal_y=dronito.est_y
            dronito.yaw_landing=dronito.est_yaw
            if dronito.verbose is True:
                print("yaw before landing", dronito.yaw_landing)
            dronito.edge=False
            return
    
    #before working dronito.mc.forward(0.05)
    
    if (dronito.case2 == dronito.state_zigzag["right"]) or (dronito.case2 == dronito.state_zigzag["left"]):
        dronito.mc.back(0.1, velocity=0.1)
        time.sleep(1)
   
    if (dronito.case2 == dronito.state_zigzag["forward1"]) or (dronito.case2 == dronito.state_zigzag["forward2"]):
        dronito.mc.right(0.1, velocity=0.1)
        time.sleep(1)

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



