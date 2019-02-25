#!/usr/bin/env python



import rospy
import numpy as np
from pycrazyswarm import *
import math

Z=0.5  ## set the target height for all cf

## Op_point_group usage need to change 


def cal_Op_pos(DT,v_max,center,r,r_max):
    ## w = v/r
    a=center[0]-r*(math.sin((v_max/r_max)*DT))## vel ctrl need to be before DT
    b=center[1]+r*(math.cos((v_max/r_max)*DT))## circle point(100, -80)
    return np.array([a,b,Z]) ## when it first begin, there should be no jump
  
def cal_Line(DT,num):
    a=DT*num
    b=DT*num
    return np.array([a,b,Z])
  
def normalize(v): ## this normalize function needs further tests 
    norm=np.linalg.norm(v) 
    if norm>1: 
        return v/norm 
    return v


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
  swarm = Crazyswarm()
  timeHelper = swarm.timeHelper
  allcfs = swarm.allcfs
  allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
  timeHelper.sleep(2)
  dt=0
  t_now=rospy.Time.now()## deal with time spent per loop
  T=rospy.Time.now()## deal with program running time
  cf_v2=np.array([0,0,0])
  cf_v4=np.array([0,0,0])
  cf_v5=np.array([0,0,0])
  cf_v7=np.array([0,0,0])
  ##def init(self,ID,target_ID,offset,master_point,a,b,h,phi_factor)
  Agent2=cfAct(2,[7,4],np.array([0,0,0]),False,0.1,0.2,0.4*np.array([[0,-1,0],[1,0,0]]),0.05)
  Agent4=cfAct(4,[2,5],np.array([-1,0,0]),False,0.1,0.2,0.4*np.array([[-1,0,0],[0,-1,0]]),0.05)
  Agent5=cfAct(5,[4],np.array([-1,1,0]),False,0.1,0.4,0.6*np.array([[0,1,0]]),0.1)
  Agent7=cfAct(7,[2],np.array([0,1,0]),False,0.1,0.6,0.6*np.array([[0,1,0]]),0.1)
  virtual_target_pos=np.array([0,0,Z])## change to circle start point
    
  Op_point_2=np.array([0,0,Z])
  Op_point_4=np.array([0,0,Z])
  Op_point_5=np.array([0,0,Z])
  Op_point_7=np.array([0,0,Z])
  r2=0.25
  r2=0.75
  r4=0.5
  r1=1
  ## for line trajectory algorithm
  Op_point_group={2:Op_point_2,
                 4:Op_point_4,
                 5:Op_point_5,
                 7:Op_point_7}
    
  while(True):
    if dt>0.01:
      dt=0
      t_now=t_last
      cf_v2=Agent2.update_cf_v(Op_point_2)
      cf_v4=Agent4.update_cf_v(Op_point_4)
      cf_v5=Agent5.update_cf_v(Op_point_5)
      cf_v7=Agent7.update_cf_v(Op_point_7)
      DT=(rospy.Time.now()-T).to_sec()
      Op_point_2=cal_Line(0.1*DT,4.25)
      Op_point_4=cal_Line(0.1*DT,4.0)
      Op_point_5=cal_Line(0.1*DT,4.5)
      Op_point_7=cal_Line(0.1*DT,3.25)
      Op_point_group[2]=Op_point_2
      Op_point_group[4]=Op_point_4
      Op_point_group[5]=Op_point_5
      Op_point_group[7]=Op_point_7

        ##if (t_last-T)>10:
        ##  if FirstTime:
        ##    FirstTime=False
        ##    StartCount=t_last
        ##  DT=rospy.Time.now()-StartCount
        ##  virtual_target_pos=cal_Op_pos(DT)
        
    t_last=rospy.Time.now()
    dt=(t_last-t_now).to_sec()
    cf2=allcfs.crazyfliesById[2]
    pos2=cf2.position()
    pos2[2]=Z
    cf2.cmdFullState(pos=pos2,vel=cf_v2,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
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
  cf2.cmdStop()
  cf4.cmdStop()
  cf5.cmdStop()
  cf7.cmdStop()

