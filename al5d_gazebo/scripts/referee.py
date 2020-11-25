#!/usr/bin/python
from time import sleep
import numpy as np
import rospy
import sys
from arm_controller import ArmController

STATIC = 1
DYNAMIC = 3

if __name__=='__main__':
    try:
        lynx = ArmController()
        sleep(1)

        [names, poses, twists] = lynx.get_object_state()

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
            Altitude = 10 * round((x[2] - 40) / 10,0) # to avoid decimals due to penetration

            # Determine if white side is up
            e3 = np.array([[0],[0],[1]])
            # compute cosine of angle between world +z and block +z
            projection = np.matmul(e3.T,  np.matmul( R , e3 ) )
            if projection > .9:
                SideBonus = 1
            else:
                SideBonus = 0

            # Compute points scored by this single block
            Points = Value * (1 + Altitude / 10 + SideBonus)

            # Check if block is on a goal platform, if so, add the points to the appropriate team
            redmin = np.array([-150,-550,40]);
            redmax = np.array([-50,-450,np.Inf])
            bluemin = np.array([50,450,40]);
            bluemax = np.array([150,550,np.Inf])

            if np.all(x > redmin) and np.all(x < redmax):
                print "[RED] ", ("(Dynamic)" if Value==DYNAMIC else "(Static)"), " Value=",Value," Altitude=",Altitude," SideBonus=",SideBonus," => Points=",Points
                RedScore += Points
                RedMaxAltitude = max(RedMaxAltitude,Altitude)
                RedSideBonusSum += SideBonus
                if Value == DYNAMIC:
                    RedNumDynamic+=1

            elif np.all(x > bluemin) and np.all(x < bluemax):
                print "[BLUE] ", ("(Dynamic)" if Value==DYNAMIC else "(Static)"), " Value=",Value," Altitude=",Altitude," SideBonus=",SideBonus," => Points=",Points
                BlueScore += Points
                BlueMaxAltitude = max(BlueMaxAltitude,Altitude)
                BlueSideBonusSum += SideBonus
                if Value == DYNAMIC:
                    BlueNumDynamic+=1


        RED = "Red Team"
        BLUE = "Blue Team"
        if RedScore != BlueScore:
            # win by score
            winner = RED if RedScore > BlueScore else BLUE
        elif RedMaxAltitude != BlueMaxAltitude:
            # win by tower height tiebreaker
            winner = RED if RedMaxAltitude > BlueMaxAltitude else BLUE
        elif RedNumDynamic != BlueNumDynamic:
            # win by number of dynamic blocks tiebreaker
            winner = RED if RedNumDynamic > BlueNumDynamic else BLUE
        elif RedSideBonusSum != BlueSideBonusSum:
            # win by sum of sidebonus tiebreaker
            winner = RED if RedSideBonusSum > BlueSideBonusSum else BLUE
        else:
            # strict tie
            winner = "Tie"

        print "Red Team: ", RedScore, " Blue Team: ", BlueScore, " Winner: ", winner

        lynx.stop()
    except KeyboardInterrupt:
        pass
