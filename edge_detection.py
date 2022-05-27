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

def find_platform_center(logs):

    #x1=Drone.est_x
    #y1=Drone.est_y
    x1=Drone.x_edge
    y1=Drone.y_edge
    
    """
    x1_bis=Drone.est_x
    y1_bis=Drone.est_y

    print(x1,' ',x1_bis)
    print(y1,' ',y1_bis)

    plt.figure()
    plt.axis('equal')
    plt.scatter([x1,x1_bis],[y1,y1_bis])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x1_bis',(x1_bis,y1_bis))
    plt.savefig('first edge')
    """

    if Drone.case == Drone.state_zigzag["right"]:
        Drone.mc.right(0.40)
        time.sleep(1)

    if Drone.case == Drone.state_zigzag["left"]:
        Drone.mc.left(0.40)
        time.sleep(1)

    """
    while(edge == True):
        print('still close')
        Drone.edge= is_edge_2()[0]
    """
    
    #logs = np.zeros([100000,4])
    Drone.mc.back(0.4)
    time.sleep(1)

    if Drone.case == Drone.state_zigzag["right"]:
        Drone.mc.start_left()
        print(y1)
        print(Drone.est_y)

        while(Drone.est_y<(y1-0.25)):
            #pass
            print('going left ',Drone.est_y,' ',y1)
        
        x2_before=Drone.est_x
        y2_before=Drone.est_y

    if Drone.case == Drone.state_zigzag["left"]:
        Drone.mc.start_right()
        print(y1)
        print(Drone.est_y)
        while(Drone.est_y>y1+0.25):
            #pass
            print('going right ',Drone.est_y,' ',y1)
        x2_before=Drone.est_x
        y2_before=Drone.est_y

    Drone.mc.start_forward()
    print('going forward')
    Drone.edge=False

    while(Drone.edge == False):
        [Drone.edge,Drone.x_edge,Drone.y_edge]=is_edge()
        if (Drone.edge==True):
            """
            print('Edge 2 detected!')
            x2_bis=Drone.est_x
            y2_bis=Drone.est_y
            """
            x2=Drone.x_edge
            y2=Drone.y_edge

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
            
        if Drone.est_x > Drone.boxborder_front - Drone.start_x:
            print("No center found, limite arene x reached, let's land for safety")
            Drone.mc.land()
            Drone.case = Drone.state_zigzag["arrived"]
            return
    
    #before working Drone.mc.forward(0.05)
    Drone.mc.forward(0.1, velocity=0.1)

    Drone.goal_x=Drone.est_x
    Drone.goal_y=Drone.est_y

    time.sleep(1)
    #Drone.mc.down(0.35)
    #Drone.mc.stop()
    Drone.mc.land(velocity=0.1)
    Drone.case = Drone.state_zigzag["arrived"]

    dX=0.15
    dY=0
    x0=x2+dX
    y0=y2+dY
    print('x1: ',x1,'y1: ',y1)
    print('x2: ',x2,'y2: ',y2)
    print('x0: ',x0,'y0: ',y0)
    print('goal_x: ',Drone.goal_x,'goal_y: ',Drone.goal_y)

    plt.figure()
    plt.axis('equal')
    plt.plot(logs[:,0]/1000,logs[:,1]/1000)
    plt.scatter([x1,x2_before,x2,x0,Drone.goal_x],[y1,y2_before,y2,y0,Drone.goal_y])
    plt.annotate('x1',(x1,y1))
    plt.annotate('x2_before',(x2_before,y2_before))
    plt.annotate('x2',(x2,y2))
    plt.annotate('x0',(x0,y0))
    plt.annotate('goal',(Drone.goal_x,Drone.goal_y))
    rectangle = plt.Rectangle((x0-0.15,y0-0.15), 0.30, 0.30,fill=None)
    plt.gca().add_patch(rectangle)
    plt.savefig('platform center')

