# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

The MotionCommander uses velocity setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E714')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def zigzag_blocking(width, height, offset):
    ##zigzaging
    xlimit=width
    ylimit=height
    y_offset=offset
    nb_offset=int(ylimit/(2*y_offset))
    print(nb_offset)
    for i in range(nb_offset):
        mc.left(xlimit/2)
        mc.forward(y_offset)
        mc.right(xlimit)
        mc.forward(y_offset)
        mc.left(xlimit/2)
        print(i)

def zigzag_nonblocking(width,height, offset):
    ##zigzaging
    xlimit=width
    ylimit=height
    y_offset=offset
    nb_offset=int(ylimit/(2*y_offset))
    print(nb_offset)
    for i in range(nb_offset):
        mc.start_left(xlimit/2)
        mc.start_forward(y_offset)
        mc.start_right(xlimit)
        mc.start_forward(y_offset)
        mc.start_left(xlimit/2)
        print(i)


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)

            #zigzag_blocking(0.5, 0.8, 0.1)
            zigzag_nonblocking(0.5, 0.8, 0.1)
            # There is also a set of functions that start a motion. The
            # Crazyflie will keep on going until it gets a new command.

            #mc.start_left(velocity=0.5)
            # The motion is started and we can do other stuff, printing for
            # instance
            for _ in range(5):
                print('Doing other work')
                time.sleep(0.2)

            # And we can stop
            mc.stop()

            # We land when the MotionCommander goes out of scope
