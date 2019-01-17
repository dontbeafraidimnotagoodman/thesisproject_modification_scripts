#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.5

# area=circle(0,1,targetHeight)
# radius = 0.5

goal=[0,0.5,Z]
goal_for_2=[0.5,0.5,Z]

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.crazyfliesById[4].setGroupMask(1)
    allcfs.crazyfliesById[2].setGroupMask(1)

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z, groupMask = 1)
    allcfs.crazyfliesById[2].setGroupMask(2)## take off at the same time
# crazyflie4 go to designated area, crazyflie2 goes to another area
# while maintains the orientation to each other
    timeHelper.sleep(1.5 + 0.5)# wait for crazyflie to be stable
    def cf4action(goal):
        cf4=allcfs.crazyfliesById[4]
        pos=cf4.position()# current position
        dist=np.array(pos)-np.array(goal)#to control speed calculate the distance to goal
        dist=np.linalg.norm(dist) 
        cf4.goTo(goal,0,duration=dist/0.5,groupMask=1)
        ##timeHelper.sleep(1.5 + 0.5)# wait for sys to cooperate
        return pos
    def cf2action(goal_for_2):
        cf2=allcfs.crazyfliesById[2]
        pos=cf2.position()
        dist=np.array(pos)-np.array(goal_for_2)# the same as cf4
        dist=np.linalg.norm(dist)
        cf2.goTo(goal_for_2,0,duration=dist/0.5,groupMask=2)
        return pos
#    def calculate_goal(goal):
#    print(type(allcfs.crazyfliesById[4].position()))
    cf4_pos=np.array(allcfs.crazyfliesById[4].position())# get the initial pos for cf4
    cf2_pos=np.array(allcfs.crazyfliesById[2].position())
    current_time=timeHelper.time()
    while np.linalg.norm(cf4_pos-goal)>0.1:
        pos4=cf4action(goal)
        pos2=cf2action(goal_for_2)
        update_time=timeHelper.time()
        dt=update_time-current_time
        current_time=update_time
        cf4_pos=pos4
        print(pos2)
        print(dt)

    
    timeHelper.sleep(1.5 + Z)
    allcfs.crazyfliesById[2].setGroupMask(1)# all changed to the same group so they move together
    allcfs.land(targetHeight=0.06, duration=1.0 + Z,groupMask=1)# land
    timeHelper.sleep(1.5 + Z)
