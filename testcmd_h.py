#!/usr/bin/env python
'''
 this algorithm is changed many times and is the remains of what is working. Right now it could control cf4 accurately flying to desired position with given speed , the speed control frequency is 1Hz, (be careful change this freq cuz it would make cf unstable), thus the speed could not be too fast( 1m/s is fast). What this does is tracking a virtual point which starts at (-0.5,0,0.5) moving at speed (0,0.1,0.5) and stops at (-0.5,1,0.5) I rewrote this as velocity_control.py but added my algorithm for my thesis
'''

import rospy
import numpy as np
from pycrazyswarm import *

if __name__ == "__main__":
    cf_v2=np.array([0.0,0.0,0.1])
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf8=allcfs.crazyfliesById[8]
    allcfs.takeoff(targetHeight=0.5, duration=1.0 + 0.5)
    timeHelper.sleep(2)
    t_now=rospy.Time.now()
    while(True):
      t_last=rospy.Time.now()
      pos=cf8.position()
      cf8.cmdFullState(pos=pos,vel=cf_v2,acc=np.array([0.0,0.0,9.0]),yaw=0,omega=np.array([0,0,0]))
      if((t_last-t_now).to_sec())>2:
        cf_v2=np.array([0.0-pos[0],0.0-pos[1],-0.2-pos[2]])
      else:
        cf_v2=np.array([0.0-pos[0],0.0-pos[1],1.0-pos[2]])
      
    cf8.cmdStop()
      
