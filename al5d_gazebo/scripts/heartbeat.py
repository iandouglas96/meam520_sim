#!/usr/bin/python3
from time import sleep
import numpy as np
import sys
from arm_controller import ArmController


if __name__ == '__main__':

    lynx = ArmController()

    while True:
        try: # show we're alive but don't do anything
            lynx.set_pos([0,0,0,0,.4,30])
            sleep(5)
            lynx.set_pos([0,0,0,0,-.4,30])
            sleep(5)
        except KeyboardInterrupt:
            lynx.stop()
            break
