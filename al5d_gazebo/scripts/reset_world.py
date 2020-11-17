#!/usr/bin/python
from time import sleep
import numpy as np
import rospy
import sys
from arm_controller import ArmController


if __name__=='__main__':
    lynx = ArmController()
    sleep(1)
    lynx.set_state([0,0,0,0,0,0])
    sleep(3)
    home = False
    while not home:
        sleep(1)
        pos, vel = lynx.get_state()
        if not len(pos) == 0 and np.linalg.norm(pos) < .1:
            home = True

    lynx.stop()
