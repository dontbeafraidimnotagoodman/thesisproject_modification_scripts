#!/usr/bin/env python



import rospy
import numpy as np
from pycrazyswarm import *
import math

Z=0.5
## cf_v4=np.array([-0.1,0.0,0.0])
## this is used for calculating DesirePos, just math function
    
    ## only calculate on the same plane
def pos_calculator(my_pose,designated_pos,r):
    n=my_pose[0]
    m=my_pose[1]
    p=designated_pos[0]
    q=designated_pos[1]
    A=(n-p)**2+m**2-2*m*q+q**2
    B=-2*p*((n-p)**2)-2*p*(m**2)+2*m*q*n+2*q*m*p-2*(q**2)*n-2*q*m*(n-p)+2*(q**2)*(n-p)
    C=(p**2+q**2-r)*((n-p)**2)+(p*m-q*n)**2+(2*p*m*q-2*(q**2)*n)*(n-p)
    x1=(-B+math.sqrt(B**2-4*A*C))/(2*A)
    x2=(-B-math.sqrt(B**2-4*A*C))/(2*A)
    y1=(x1*(m-p)-p*m+q*n)/(n-p)
    y2=(x2*(m-p)-p*m+q*n)/(n-p)
    if ((n-x1)**2+(m-y1)**2)<(((n-x1)**2+(m-y1)**2)):
        return np.array([x1,y1,Z])
    else:
        return np.array([x2,y2,Z])

## need to add offset to target_pos and desire_pos before passed in

## desire_pos is the center of desired circle area
## resultDesirePos is P(x)
class cfAct:
  def __init__(self,ID,desire_pos,desire_R,target_ID,offset):
    self.ID=ID
    self.desire_pos=desire_pos
    self.R=desire_R
    self.target_ID=target_ID
    self.target_pos=None
    self.current_pos=None## this pos should be added an offset later 
    self.cf_v=None
    self.offset=offset
    self.desire_pos[2]=Z ##fixed height
    self.resultDesirePos=None
    ## get_pos will return the real time pos everytime you call get_pos
  def get_pos(self):
    cf=allcfs.crazyfliesById[self.ID]
    pos=cf.position()
    pos[2]=Z ## fixed height
    self.current_pos=pos
    return pos
  ## return P'(x) which is based on biased position
  def getDesirePos(self):
    pos=self.get_pos()
    pos=pos+self.offset
    return pos_calculator(my_pose=pos,designated_pos=self.desire_pos,r=self.R)
    
  def update_parameter(self):
    pos=self.get_pos()
    pos=pos+self.offset
    self.current_pos=pos
    cf_target=allcfs.crazyfliesById[self.target_ID]
    self.target_pos=cf_target.position()
    if (np.linalg.norm(pos-self.desire_pos))>self.R:
        self.resultDesirePos=self.getDesirePos()+self.offset
    else:
        self.resultDesirePos=pos
        ## changed velocity to normalized form
  def get_cf_v(self):
    self.update_parameter()
    if np.linalg.norm(self.target_pos-self.current_pos)==0:
      cf_v=0.2*((self.target_pos-self.current_pos))+0.1*((self.resultDesirePos-self.current_pos)/np.linalg.norm(self.resultDesirePos-self.current_pos))
    elif np.linalg.norm(self.resultDesirePos-self.current_pos)==0:
      cf_v=0.2*((self.target_pos-self.current_pos)/np.linalg.norm(self.target_pos-self.current_pos))+0.1*((self.resultDesirePos-self.current_pos))
    elif np.linalg.norm(self.resultDesirePos-self.current_pos)==0 and np.linalg.norm(self.target_pos-self.current_pos)==0:
      cf_v=0.2*((self.target_pos-self.current_pos))+0.1*((self.resultDesirePos-self.current_pos))
    else:
      cf_v=0.2*((self.target_pos-self.current_pos)/np.linalg.norm(self.target_pos-self.current_pos))+0.1*((self.resultDesirePos-self.current_pos)/np.linalg.norm(self.resultDesirePos-self.current_pos))
    cf_v[2]=0
    self.cf_v=cf_v
    return cf_v
  
    
    


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(3)
    dt=0
    t_now=rospy.Time.now()
    cf_v3=np.array([0,0,0])
    cf_v4=np.array([0,0,0])
    cf_v5=np.array([0,0,0])
    cf_v7=np.array([0,0,0])
    ## set every desire_pos  and desire_R to be the same in the first experiment
    ## def __init__(self,ID,desire_pos,desire_R,target_ID,offset):
    Agent3=cfAct(3,np.array([0,-1,0]),1,7,np.array([0,-1,0]))
    Agent4=cfAct(4,np.array([0,0,0]),1,3,np.array([-1,0,0]))### need to change
    Agent5=cfAct(5,np.array([1,0,0]),1,4,np.array([0,1,0]))### need to change
    Agent7=cfAct(7,np.array([1,-1,0]),1,5,np.array([1,0,0]))### need to change
        
        
    ## bigragh forms a circle
    
    while(True):
      if dt>1:
        dt=0
        t_now=t_last
        cf_v3=Agent3.get_cf_v()
        cf_v4=Agent4.get_cf_v()
        cf_v5=Agent5.get_cf_v()
        cf_v7=Agent7.get_cf_v()
        
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

      
