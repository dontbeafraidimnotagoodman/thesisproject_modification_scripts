#!/usr/bin/env python


import csv
import rospy
import numpy as np
from pycrazyswarm import *
import math

Z=0.5  ## set the target height for all cf

## change target_ID to be a list and could be returned


def cal_Op_pos(DT,v_max,center,r,r_max):
    ## w = v/r
    a=center[0]-r*(math.sin((v_max/r_max)*DT))## vel ctrl need to be before DT
    b=center[1]+r*(math.cos((v_max/r_max)*DT))## circle point(100, -80)
    return np.array([a,b,Z]) ## when it first begin, there should be no jump
  
def normalize(v): ## this normalize function needs further tests 
    norm=np.linalg.norm(v)  
    return v/(norm+1)


class cfAct:
  def __init__(self,ID,target_ID,offset,master_point,a,b,h):
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
      result_offset=np.array(self.target_pos)-np.array(self.h)
      for i in range(len(result_offset)):
        B_vector=B_vector+normalize(self.target_pos[i]-self.current_pos-self.h[i])
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
  DT=0
  DT=long(DT)
  t_now=rospy.Time.now()## deal with time spent per loop
  T=rospy.Time.now()## deal with program running time
  cf_v2=np.array([0,0,0])
  cf_v4=np.array([0,0,0])
  cf_v5=np.array([0,0,0])
  cf_v7=np.array([0,0,0])
  cf_v1=np.array([0,0,0])
  cf_v8=np.array([0,0,0])
  ##def init(self,ID,target_ID,offset,master_point,a,b,h)
  Agent2=cfAct(2,[7,4],np.array([0,0,0]),False,1,0.5,0.8*np.array([[0,-1,0],[1,0,0]]))
  Agent4=cfAct(4,[2,5],np.array([-1,0,0]),False,1,1.25,0.8*np.array([[-1,0,0],[0,-1,0]]))
  Agent5=cfAct(5,[4,8],np.array([-1,1,0]),False,1,1.0,0.8*np.array([[0,1,0],[0,-1,0]]))
  Agent7=cfAct(7,[2,1],np.array([0,1,0]),False,1,1.0,0.8*np.array([[0,1,0],[0,-1,0]]))
  Agent1=cfAct(1,[7,8],np.array([0,2,0]),False,1,1.0,0.8*np.array([[0,1,0],[1,0,0]]))
  Agent8=cfAct(8,[5,1],np.array([-1,2,0]),False,1,1.25,0.8*np.array([[0,1,0],[-1,0,0]]))
  virtual_target_pos=np.array([0,0,Z])## change to circle start point  
  Op_point_2=np.array([0,0,Z])
  Op_point_4=np.array([0,0,Z])
  Op_point_5=np.array([0,0,Z])
  Op_point_7=np.array([0,0,Z])
  Op_point_1=np.array([0,0,Z])
  Op_point_8=np.array([0,0,Z])
  r3=0.5*math.sqrt(2)
  r2=0.8*math.sqrt(2)
  r4=0.5*math.sqrt(2)
  r1=1.0*math.sqrt(2)
  r5=0.5*math.sqrt(2)
  r6=0.8*math.sqrt(2)
  count=0
  
  with open('6_consensus_circle_3.csv','w') as csvFile:
    writer = csv.writer(csvFile)
    while(True):
      if dt>0.025:
        dt=0
        t_now=t_last
        cf_v2=Agent2.get_cf_v(Op_point_2)
        cf_v4=Agent4.get_cf_v(Op_point_4)
        cf_v5=Agent5.get_cf_v(Op_point_5)
        cf_v7=Agent7.get_cf_v(Op_point_7)
        cf_v1=Agent1.get_cf_v(Op_point_1)
        cf_v8=Agent8.get_cf_v(Op_point_8)
        DT=(rospy.Time.now()-T).to_sec()
        Op_point_2=cal_Op_pos(DT,0.5,np.array([0,0])+0.8*np.array([-0.5,1.0]),r1,r1)
        Op_point_4=cal_Op_pos(DT,0.5,np.array([0,0])+0.8*np.array([0.5,1.0]),r2,r1)
        Op_point_5=cal_Op_pos(DT,0.5,np.array([0,0]+0.8*np.array([0.5,0.0])),r3,r1)
        Op_point_7=cal_Op_pos(DT,0.5,np.array([0,0])+0.8*np.array([-0.5,0.0]),r4,r1)
        Op_point_1=cal_Op_pos(DT,0.5,np.array([0,0])+0.8*np.array([-0.5,-1.0]),r5,r1)
        Op_point_8=cal_Op_pos(DT,0.5,np.array([0,0])+0.8*np.array([0.5,-1.0]),r6,r1)
        c2=Agent2.get_pos()
        c4=Agent4.get_pos()
        c5=Agent5.get_pos()
        c7=Agent7.get_pos()
        c1=Agent1.get_pos()
        c8=Agent8.get_pos()
        writer.writerow([c2,c4,c5,c7,c1,c8,DT])

        ##if (t_last-T)>10:
        ##  if FirstTime:
        ##    FirstTime=False
        ##    StartCount=t_last
        ##  DT=rospy.Time.now()-StartCount
        ##  virtual_target_pos=cal_Op_pos(DT)
        
      t_last=rospy.Time.now()
      dt=(t_last-t_now).to_sec()
      if count==0:
        cf2=allcfs.crazyfliesById[2]
        pos2=cf2.position()
        pos2[2]=Z
        cf2.cmdFullState(pos=pos2,vel=cf_v2,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
      if count==1:
        
        cf4=allcfs.crazyfliesById[4]
        pos4=cf4.position()
        pos4[2]=Z
        cf4.cmdFullState(pos=pos4,vel=cf_v4,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
      if count==2:
        cf5=allcfs.crazyfliesById[5]
        pos5=cf5.position()
        pos5[2]=Z
        cf5.cmdFullState(pos=pos5,vel=cf_v5,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
        
      if count==3:
        cf7=allcfs.crazyfliesById[7]
        pos7=cf7.position()
        pos7[2]=Z
        cf7.cmdFullState(pos=pos7,vel=cf_v7,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
        
      if count==4:
        cf1=allcfs.crazyfliesById[1]
        pos1=cf1.position()
        pos1[2]=Z
        cf1.cmdFullState(pos=pos1,vel=cf_v1,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
        
      if count==5:
        cf8=allcfs.crazyfliesById[8]
        pos8=cf8.position()
        pos8[2]=Z
        cf8.cmdFullState(pos=pos8,vel=cf_v8,acc=np.array([0.0,0.0,0.0]),yaw=0,omega=np.array([0,0,0]))
        count=0
      
      button_pressed = swarm.input.checkIfButtonIsPressed()
      if button_pressed == True:
            break


  timeHelper.sleep(5)
  cf2.cmdStop()
  cf4.cmdStop()
  cf5.cmdStop()
  cf7.cmdStop()
  cf1.cmdStop()
  cf8.cmdStop()
