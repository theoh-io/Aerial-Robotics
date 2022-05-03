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

### land


## Bounding box
on connait position départ et arène dim mais pas objectif

## Obstacle Avoidance

### recuperer valeurs multirangers (capteurs cotés TOF)

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

### arena
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

### Am