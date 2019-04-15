#!/usr/bin/env python
'''
 this algorithm is changed many times and is the remains of what is working. Right now it could control cf4 accurately flying to desired position with given speed , the speed control frequency is 1Hz, (be careful change this freq cuz it would make cf unstable), thus the speed could not be too fast( 1m/s is fast). What this does is tracking a virtual point which starts at (-0.5,0,0.5) moving at speed (0,0.1,0.5) and stops at (-0.5,1,0.5) I rewrote this as velocity_control.py but added my algorithm for my thesis
'''

import rospy
import numpy as np
from pycrazyswarm import *
import tf


def normalize(v): ## this normalize function needs further tests 
    norm=np.linalg.norm(v)  
    return v/(norm+1)

class cfAct:
  def __init__(self,ID,target_ID,offset,master_point,a,b,h,phi_factor):
    self.ID=ID
    self.desire_pos=None
    self.target_ID=target_ID ## list (1*n)
    self.target_pos=None  ## list(n*2)
    self.current_pos=None## this pos should be added an offset later 
    self.cf_v=None
    self.offset=offset
    self.offset_pos=None
    self.master_point=False## determine whether this is a communication node
    self.a=a
    self.b=b
    self.h=h## act as offset between neighbors because its different from offset
    self.add_phi=None
    self.B_vector=None
    self.phi_factor=phi_factor
            ## self.h is also a list with the same order of self.target_ID
    ## get_pos will return the real time pos everytime you call get_posdef get_pos(self):
  def get_pos(self):
    cf=allcfs.crazyfliesById[self.ID]
    pos=cf.position()
    pos[2]=Z ## fixed height
    return pos
  
  def get_target_pos(self):
    target_len=len(self.target_ID)
    pos_list=[]
    for i in range(target_len):
      cf_target=allcfs.crazyfliesById[self.target_ID[i]]
      t_pos=cf_target.position()
      t_pos[2]=Z
      pos_list.append(t_pos)
    return pos_list
  
  ##only change self param in this function
  ## otherwise mess the structure up
  def update_param(self,desire_pos):
    self.current_pos=self.get_pos()
    self.offset_pos=self.current_pos+self.offset
    self.target_pos=self.get_target_pos()
    self.desire_pos=desire_pos
    self.desire_pos[2]=Z## fixed height(here is why expriment always wrong)
  def get_cf_v(self,desire_pos):
    self.update_param(desire_pos)
    if not self.master_point:
      pos=self.current_pos
      result_offset=np.array(self.target_pos)-np.array(self.h)
      B_vector=0
      for i in range(len(result_offset)):
        B_vector=B_vector+normalize(self.target_pos[i]-self.current_pos-self.h[i])
      A_vector=(self.desire_pos-pos)
      self.cf_v=self.a*A_vector+self.b*B_vector## change cf_v here 
    else:   ## the master crazyflie do not need h offset
      pos=self.current_pos
      A_vector=(self.desire_pos-pos)
      B_vector=0
      for i in range(len(result_offset)):
        B_vector=B_vector+normalize(self.target_pos[i]-self.current_pos-self.h[i])
      self.cf_v=self.a*A_vector+self.b*B_vector
    self.cf_v[2]=0
    self.B_vector=B_vector
  def phi_restriction(self):## change self.phi add restriction on phi
    target_len=len(self.target_ID)
    add_norm=0
    for i in range(target_len):
      target_pos=Op_point_group[self.target_ID[i]]
      norm_j=np.linalg.norm(target_pos)
      add_norm=add_norm+norm_j
    add_norm=add_norm+np.linalg.norm(Op_point_group[self.ID])
    self.add_phi=self.phi_factor*add_norm*np.array(self.B_vector)
  def update_cf_v(self,desire_pos):
    self.get_cf_v(desire_pos)
    self.phi_restriction()
    return self.add_phi+self.cf_v

if __name__ == "__main__":
    Z=1.0
    H=0
    cf_v8=np.array([0.0,0.0,0.0])
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf8=allcfs.crazyfliesById[8]
    allcfs.takeoff(targetHeight=Z, duration=1.0 + 0.5)

    
    tf = tf.TransformListener()
    tf.waitForTransform("/world", "/vicon/cftarget_2/cftarget_2", rospy.Time(0), rospy.Duration(0.1))
    timeHelper.sleep(2)
    t_now=rospy.Time.now()
    t_now2=rospy.Time.now()
    dt=0
    position_last,quaternion=tf.lookupTransform("/world", "/vicon/cftarget_2/cftarget_2", rospy.Time(0))
    Op_point_8=np.array([position_last[0],position_last[1],Z])
    Op_point_group={
                    8:Op_point_8}



    ##def init(self,ID,target_ID,offset,master_point,a,b,h,phi_factor)
    Agent8=cfAct(8,[8],np.array([0,0,0]),False,1,0.6,0.5*np.array([[0,0,0]]),0.05)
    while(True):
      t_last=rospy.Time.now()
      
      dt=(t_last-t_now2).to_sec()
      if dt>0.03:
        t_now2=rospy.Time.now()
        position, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_2/cftarget_2", rospy.Time(0))
        v_tar=(np.array(position)-np.array(position_last))/dt
        #######right here change v_tar[2]
        Op_point_8=np.array([position_last[0],position_last[1],Z])+np.array([v_tar[0],v_tar[1],0])
        Op_point_group[8]=Op_point_8
        cf_v_8=Agent8.update_cf_v(Op_point_8)
        ######here change height velocity
        cf_v8=np.array([cf_v_8[0],cf_v_8[1],H])
        position_last=position
        temp_cf=cf8.position()
        if(np.linalg.norm(np.array([temp_cf[0],temp_cf[1]])-np.array([position_last[0],position_last[1]]))<0.3):
          temp_h=cf8.position()
          H=position_last[2]-temp_h[2]
      print(cf_v8)
      pos=cf8.position()
      cf8.cmdFullState(pos=pos,vel=cf_v8,acc=np.array([0.0,0.0,6.0]),yaw=0,omega=np.array([0,0,0]))
        
   
      
    cf8.cmdStop()
      
