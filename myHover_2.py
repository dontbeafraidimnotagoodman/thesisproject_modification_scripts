#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.5

# area=circle(0,1,targetHeight)
# radius = 0.5

goal=[0,0.5,Z]
goal_for_2=[0.5,0.5,Z]

def allcfsaction(goal,dt,flie_num):
    cf4=allcfs.crazyfliesById[flie_num]
    pos=cf4.position()# current position
    dist=np.array(pos)-np.array(goal)#to control speed calculate the distance to goal
    n=np.linalg.norm(dist) 
    dist=dist/n # normalize the distance vector
    micro_distance=dt*(0.1)# 0.1 m/s * time=distance need to move in time dt
    current_goal=micro_distance*dist+pos # current goal in time dt
    cf4.goTo(current_goal,0,duration=micro_distance/0.05,groupMask=flie_num)
    return pos # return current position
        

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.crazyfliesById[4].setGroupMask(1)
    allcfs.crazyfliesById[2].setGroupMask(1)# here if use crazyflie 1 would get conflict 
# with group mask

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z, groupMask = 1)
    allcfs.crazyfliesById[2].setGroupMask(2)## take off at the same time
    allcfs.crazyfliesById[4].setGroupMask(4)## change the mask to make them move seperately
# crazyflie4 go to designated area, crazyflie2 goes to another area
# while maintains the orientation to each other
    timeHelper.sleep(1.5 + 0.5)# wait for crazyflie to be stable
    
    
#    def calculate_goal(goal):
#    print(type(allcfs.crazyfliesById[4].position()))
    cf4_pos=np.array(allcfs.crazyfliesById[4].position())# get the initial pos for cf4
    cf2_pos=np.array(allcfs.crazyfliesById[2].position())# get the initial pos for cf2
    dt=0.025 ## the dt for the first loop
    current_time=timeHelper.time()
    while np.linalg.norm(cf4_pos-goal)>0.1:
        pos4=allcfsaction(goal,dt,4)
        pos2=allcfsaction(goal_for_2,dt,2)
        update_time=timeHelper.time()
        dt=update_time-current_time
        current_time=update_time
        cf4_pos=pos4
        print(pos2)
        print(dt)

    
    timeHelper.sleep(1.5 + Z)
    allcfs.crazyfliesById[2].setGroupMask(1)# all changed to the same group so they move together
    allcfs.crazyfliesById[4].setGroupMask(1)
    allcfs.land(targetHeight=0.06, duration=1.0 + Z,groupMask=1)# land
    timeHelper.sleep(1.5 + Z)
