# Project Steps

## Take off
Modifying the speeds for takeoff and landing to increase stability:
Changed the constants values in MotionCommander (Fast takeoff and slow landing). ✅ <br>

## Position Estimation
Able to get estimation for x, y, z and yaw. ✅ <br>
Need to be able to calibrate yaw after takeoff and landing. <br>
How to solve the drift problem ? <br>

## Exploration

### Algorithm parcours
Simple Algorithm: ZigZag ✅ <br>
Complex Algorithm: Need to do research on efficient way to scan the landing zone. <br>

### Detection capteur zranger
Being Able to plot the change in detected z value. <br>
Mechanism to avoid going up when landing detected: Run som tests Forward and land when change detected. <br>

## Bounding box
Need To define more clearly the parameters of the bounding box: zone to start scanning...

## Obstacle Avoidance
Simple Obstacle Avoidance ✅<br>
Complete Obstacle Avoidance: Both Sides <br>
Test to see behaviour with weird obstacles shapes <br>
Record position of obstacles when detected <br>

## Global Navigation
Record Obstacle position when detected <br>
Record Safe Waypoints at a given frequency <br>
A* to sompute optimal path for the way back while avoiding obstacles <br>
### A*
Inputs: Start, Goal, heuristic (cost function in our case L2), OccupancyGrid 2D np.array
variables description: Occupancy grid: 0 free, 1 not explored, 2 obstacle
parameter: grid size => to be able to compute the distance between points of the grid
output: waypoint list (en ligne droite)
when is it called: everytime in the loop
Bonus:  Checkpoint amelioré, Visualisation of the grid in real time.


### A*
Inputs: Start, Goal, heuristic (cost function in our case L2), OccupancyGrid 2D np.array
variables description: Occupancy grid: 0 free, 1 not explored, 2 obstacle
parameter: grid size => to be able to compute the distance between points of the grid
output: waypoint list (en ligne droite)
when is it called: everytime in the loop
Bonus: Visualisation of the grid in real time.

### Waypoint following
Input: current position, next waypoint
description: send non blocking direction command 
Output: s



---

## Questions

### Algo d'exploration state of the art

### hauteur
field of view du optical flow
seuil pour detection
fly about a meter

### obstacle avoidance

multiranger example to test the threshold for detection

distance de detection des capteurs
ANN difference avec differential drive

No hard coding of the obstacle avoidance

### Exploration
turn it on flat
set estimation to 0 before taking off
Region for scanning

coming back exploration: due to drift

### Drifts 
trop de temps à explorer => landing moins précis
x et y due à l'integration de l'IMU
Kalman: z-ranger, optic flow, IMU
a améliorer ? problème probable ?

also drift from takeoff and landing (yaw ~ 20 deg)

### Arena
around 5x3m
same cylinder shape

---
## Ressources

### Exploration
https://github.com/HKUST-Aerial-Robotics/Fast-Planner



