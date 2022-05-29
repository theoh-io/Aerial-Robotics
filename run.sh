#!/bin/bash

python3 motion_flying.py

#How to use arguments
# python3 motion_flying.py \
#     --arenaX 3 --arenaY 1.5 --startY 0.75 --startX 0.4 --goal_zone 1.5 --start_zone 0.5 \
#     --vel_takeoff 0.6 --vel_landing 0.1 \
#     --vel_obst 0.5 --low_vel_obst 0.05 --thresh_y 0.9 --min_dist 0.5 --margin 2 --small_dist 0.4\
#     --delta_x 0.1 --delta_y 0.25  --vel_edge 0.2  --vel_edge_goal 0.1 \
#     --x_offset 0.25 --box_x 0.2 --box_y 0.2 