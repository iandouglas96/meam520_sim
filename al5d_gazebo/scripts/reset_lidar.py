#!/usr/bin/python3
from time import sleep
import numpy as np
import rospy
import sys
from arm_controller import ArmController

    
if __name__=='__main__':
    cont = ArmController(5,False)
    sleep(1)
    cont.set_state([0,0,0,0,0,0])
    sleep(5)
    rospy.loginfo("The current state is: ")
    print(cont.get_state())    
    cont.stop()



