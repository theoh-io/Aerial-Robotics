# Project Steps

## Take off

## Position Estimation
Utile pour optimiser l'exploration

(renseignement flow deck angle of view)

## Exploration

### Algorithm parcours
commence par algo simple: aller /retours ou zigzag
(Chercher algo optimal)

### Detection capteur zranger
Explore pour trouver l' objectif (box center)

### Land


## Bounding box
on connait position départ et arène dim mais pas objectif

## Obstacle Avoidance

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

### Algo de contournement

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

## Tests à faire

### drift de la box
voler longtemps et voir si la box bouge

---
## Ressources

### Exploration
https://github.com/HKUST-Aerial-Robotics/Fast-Planner

---
## Repartition des taches

