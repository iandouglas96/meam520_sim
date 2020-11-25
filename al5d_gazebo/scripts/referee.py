#!/usr/bin/python
from time import sleep
import numpy as np
import rospy
import sys
from arm_controller import ArmController

import subprocess
from std_srvs.srv    import Empty, EmptyRequest

STATIC = 1
DYNAMIC = 3

def score(names, poses,verbose=False):

    # Actual Scores

    RedScore = 0
    BlueScore = 0

    # Tiebreakers

    RedMaxAltitude = 0
    BlueMaxAltitude = 0

    RedNumDynamic = 0
    BlueNumDynamic = 0

    RedSideBonusSum = 0
    BlueSideBonusSum = 0

    for index, pose in enumerate(poses):

        x = pose[0:3,3]
        R = pose[0:3,0:3]

        # Determine which type of block
        name = names[index]
        if "static" in names[index]:
            Value = STATIC
        elif "dynamic" in names[index]:
            Value = DYNAMIC

        # Determine distance from block center to goal platform
        Altitude = 10 * round((x[2] - 40) / 10,0) # to avoid decimals due to penetration in Gazebo

        # Determine if white side is up
        e3 = np.array([[0],[0],[1]])
        # compute cosine of angle between world +z and block +z
        projection = np.matmul(e3.T,  np.matmul( R , e3 ) )
        if projection > .9:
            SideBonus = 1
        else:
            SideBonus = 0

        # Compute points that would be scored by this single block
        Points = Value * (1 + Altitude / 10 + SideBonus)

        # Check if block is on a goal platform, if so, add the points to the appropriate team
        redmin = np.array([-150,-550,40]);
        redmax = np.array([-50,-450,np.Inf])
        bluemin = np.array([50,450,40]);
        bluemax = np.array([150,550,np.Inf])

        if np.all(x > redmin) and np.all(x < redmax):
            if verbose:
                print "[RED] ", ("(Dynamic)" if Value==DYNAMIC else "(Static)"), " Value=",Value," Altitude=",Altitude," SideBonus=",SideBonus," -> Points=",Points
            RedScore += Points
            RedMaxAltitude = max(RedMaxAltitude,Altitude)
            RedSideBonusSum += SideBonus
            if Value == DYNAMIC:
                RedNumDynamic+=1

        elif np.all(x > bluemin) and np.all(x < bluemax):
            if verbose:
                print "[BLUE] ", ("(Dynamic)" if Value==DYNAMIC else "(Static)"), " Value=",Value," Altitude=",Altitude," SideBonus=",SideBonus," -> Points=",Points
            BlueScore += Points
            BlueMaxAltitude = max(BlueMaxAltitude,Altitude)
            BlueSideBonusSum += SideBonus
            if Value == DYNAMIC:
                BlueNumDynamic+=1


    RED = "Red Team"
    BLUE = "Blue Team"
    if RedScore != BlueScore:
        # win by score
        Winner = RED if RedScore > BlueScore else BLUE
    elif RedMaxAltitude != BlueMaxAltitude:
        # win by tower height tiebreaker
        Winner = RED if RedMaxAltitude > BlueMaxAltitude else BLUE
    elif RedNumDynamic != BlueNumDynamic:
        # win by number of dynamic blocks tiebreaker
        Winner = RED if RedNumDynamic > BlueNumDynamic else BLUE
    elif RedSideBonusSum != BlueSideBonusSum:
        # win by sum of sidebonus tiebreaker
        Winner = RED if RedSideBonusSum > BlueSideBonusSum else BLUE
    else:
        # strict tie
        Winner = "Tie"

    if verbose:
        print("\n[[ FINAL SCORES ]]\n")
        print "[RED]: ", RedScore, " [BLUE]: ", BlueScore, " [WINNER]: ", Winner

    return RedScore, BlueScore, Winner

if __name__=='__main__':

    try:

        pause_physics_client=rospy.ServiceProxy('/gazebo/pause_physics',Empty)

        # connect to a robot for scoring purposes
        lynx = ArmController('red') # doesn't matter red or blue
        sleep(1)

        # match clock initialization
        start = rospy.Time.now()
        allotted = rospy.Duration(5)
        remaining = allotted

        while remaining.to_sec() > 0:

            elapsed = rospy.Time.now() - start
            remaining = allotted - elapsed

            subprocess.call("clear") # clear the command line
            print "Time Left:", max(remaining.to_sec(),0)
            sleep(.1)

        # game is over
        lynx.stop() # must stop before pausing physics
        pause_physics_client(EmptyRequest())

        subprocess.call("clear")
        [names, poses, twists] = lynx.get_object_state()
        score(names, poses, verbose=True) # print detailed score report

        print('')

    except KeyboardInterrupt:
        lynx.stop()
