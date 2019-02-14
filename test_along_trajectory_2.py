#!/usr/bin/env python



import rospy
import numpy as np
from pycrazyswarm import *
import math


## whats left
## change self.a self.b to make them stable at equilibrium point ##change normalize
## change update time to be faster ## checked
## change desire_pos-pos to pos- desire_pos ##checked


Z=0.5  ## set the target height for all cf

## this will have effect on all cfs at all time


## did not add vel ctrl 
def cal_Op_pos(DT):
    r=0.5 ## w = v/r ## begin from (-1,1)
    a=0-r*(math.sin(0.5*(0.2/0.5)*DT))## vel ctrl need to be before DT
    b=-0.5+r*(math.cos(0.5*(0.2/0.5)*DT))## circle point(0, -0.5)
    return np.array([a,b]) ## when it first begin, there should be no jump
  

def normalize(v):  ## this normalize function needs further tests
  norm=np.linalg.norm(v)
  if norm>1:
    return v/norm
  return v

'''
def normalize(v):
  norm=np.linalg.norm(v)
  if norm==0:
    return v
  return v/norm
'''  

class cfAct:
  def __init__(self,ID,target_ID,offset,master_point,a,b,h):
    self.ID=ID
    self.desire_pos=None
    self.target_ID=target_ID
    self.target_pos=None
    self.current_pos=None## this pos should be added an offset later 
    self.cf_v=None
    self.offset=offset
    self.offset_pos=None
    self.master_point=False## determine whether this is a communication node
    self.a=a
    self.b=b
    self.h=h## act as offset between neighbors because its different from offset
    ## get_pos will return the real time pos everytime you call get_posdef get_pos(self):
  def get_pos(self):
    cf=allcfs.crazyfliesById[self.ID]
    pos=cf.position()
    pos[2]=Z ## fixed height
    return pos
  def get_target_pos(self):
    cf_target=allcfs.crazyfliesById[self.target_ID]
    t_pos=cf_target.position()
    t_pos[2]=Z
    return t_pos
  
  ##only change self param in this function
  ## otherwise mess the structure up
  def update_param(self,desire_pos):
    self.current_pos=self.get_pos()
    self.offset_pos=self.current_pos+self.offset
    self.target_pos=self.get_target_pos()
    self.desire_pos=desire_pos
    self.desire_pos=self.desire_pos[2]## fixed height
  def get_cf_v(self,desire_pos):
    self.update_param(desire_pos);
    if not self.master_point:
      pos=self.offset_pos
      A_vector=normalize(self.desire_pos-pos)
      B_vector=normalize(self.target_pos-self.current_pos-self.h)
      self.cf_v=self.a*A_vector+self.b*B_vector## change cf_v here 
    else:   ## the master crazyflie do not need h offset
      pos=self.current_pos
      A_vector=normalize(self.desire_pos-pos)
      B_vector=normalize(self.target_pos-pos)
      self.cf_v=self.a*A_vector+self.b*B_vector
    self.cf_v[2]=0
    return self.cf_v
      
      
    
  
    
    
    


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(2)
    dt=0
    t_now=rospy.Time.now()## deal with time spent per loop
    T=rospy.Time.now()## deal with program running time
    cf_v3=np.array([0,0,0])
    cf_v4=np.array([0,0,0])
    cf_v5=np.array([0,0,0])
    cf_v7=np.array([0,0,0])
    Agent3=cfAct(3,3,np.array([0,0,0]),True,0.2,0.1,np.array([0,0,0]))
    Agent4=cfAct(4,3,np.array([-1,0,0]),False,0.2,0.1,np.array([-1,0,0]))
    Agent5=cfAct(5,4,np.array([-1,1,0]),False,0.2,0.1,np.array([0,1,0]))
    Agent7=cfAct(7,3,np.array([0,1,0]),False,0.2,0.1,np.array([0,1,0]))
    virtual_target_pos=np.array([0,0,0.5])## change to circle start point
    m=0
    FirstTime=True
    
    while(True):
      if dt>1:
        dt=0
        t_now=t_last
        desire_pos=virtual_target_pos
        cf_v3=Agent3.get_cf_v(desire_pos)
        cf_v4=Agent4.get_cf_v(desire_pos)
        cf_v5=Agent5.get_cf_v(desire_pos)
        cf_v7=Agent7.get_cf_v(desire_pos)

        '''if m<10:
          virtual_target_pos=virtual_target_pos+np.array([0,0.1,0])
          m=m+1'''

        if (t_last-T).to_sec()>6:
          if FirstTime:
            FirstTime=False
            StartCount=t_last
          DT=rospy.Time.now()-StartCount
          virtual_target_pos=cal_Op_pos(DT.to_sec())
        
      t_last=rospy.Time.now()
      dt=(t_last-t_now).to_sec()
      cf3=allcfs.crazyfliesById[3]
      pos3=cf3.position()
      pos3[2]=Z
      cf3.cmdFullState(pos=pos3,vel=cf_v3,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
      cf4=allcfs.crazyfliesById[4]
      pos4=cf4.position()
      pos4[2]=Z
      cf4.cmdFullState(pos=pos4,vel=cf_v4,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
      cf5=allcfs.crazyfliesById[5]
      pos5=cf5.position()
      pos5[2]=Z
      cf5.cmdFullState(pos=pos5,vel=cf_v5,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
      cf7=allcfs.crazyfliesById[7]
      pos7=cf7.position()
      pos7[2]=Z
      cf7.cmdFullState(pos=pos7,vel=cf_v7,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
      button_pressed = swarm.input.checkIfButtonIsPressed()
      if button_pressed == True:
            break


    timeHelper.sleep(5)
    cf3.cmdStop()
    cf4.cmdStop()
    cf5.cmdStop()
    cf7.cmdStop()
