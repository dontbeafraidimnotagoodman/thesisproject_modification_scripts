#!/usr/bin/env python



import rospy
import numpy as np
from pycrazyswarm import *

Z=0.5
offset=0.1
cf_v2=np.array([-0.1,0.0,0.0])

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf2=allcfs.crazyfliesById[4]

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(2)
    dt=0
    pos_goal_checkpoint=cf2.position()
    pos_goal_checkpoint[2]=Z
    i=0
    long(i)
    error=0
    while(True):
        cf_v2=cf_v2
        cf_v2[2]=0
        t_now=rospy.Time.now()
        pos2=cf2.position()
        pos2[2]=Z
        if (i==0):
            t_last=t_now
            dt=0
        else:
            dt=(t_last-t_now).to_sec()
        if (i%1000==0 and i!=0):
            error=pos_goal_checkpoint-pos2
            error=error/np.linalg.norm(error)
            pos_goal_checkpoint=np.array(pos_goal_checkpoint)+1000*dt*cf_v2

        pos_goal=pos2
        pos_goal[2]=Z
        cf2.cmdFullState(pos=pos_goal,vel=cf_v2,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))

        t_last=t_now
        button_pressed = swarm.input.checkIfButtonIsPressed()
        print(i)
        i=i+1
        if button_pressed == True:
            break
    
    timeHelper.sleep(5)
    
    cf2.cmdStop()



