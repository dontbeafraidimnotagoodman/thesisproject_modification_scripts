
#!/usr/bin/env python

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
    Z=0.8
    H=0
    cf_v2=np.array([0.0,0.0,0.0])
    cf_v4=np.array([0.0,0.0,0.0])
    cf_v5=np.array([0.0,0.0,0.0])
    cf_v7=np.array([0.0,0.0,0.0])
    cf_v1=np.array([0.0,0.0,0.0])
    cf_v8=np.array([0.0,0.0,0.0])
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=1.0 + 0.5)

    
    tf = tf.TransformListener()
    tf.waitForTransform("/world", "/vicon/cftarget_1/cftarget_1", rospy.Time(0), rospy.Duration(0.1))
    tf.waitForTransform("/world", "/vicon/cftarget_2/cftarget_2", rospy.Time(0), rospy.Duration(0.1))
    tf.waitForTransform("/world", "/vicon/cftarget_3/cftarget_3", rospy.Time(0), rospy.Duration(0.1))
    tf.waitForTransform("/world", "/vicon/cftarget_4/cftarget_4", rospy.Time(0), rospy.Duration(0.1))
    tf.waitForTransform("/world", "/vicon/cftarget_5/cftarget_5", rospy.Time(0), rospy.Duration(0.1))
    tf.waitForTransform("/world", "/vicon/cftarget_6/cftarget_6", rospy.Time(0), rospy.Duration(0.1))
    timeHelper.sleep(2)
    t_now=rospy.Time.now()
    t_now2=rospy.Time.now()
    dt=0
    position_last_1,quaternion=tf.lookupTransform("/world", "/vicon/cftarget_1/cftarget_1", rospy.Time(0))
    position_last_2,quaternion=tf.lookupTransform("/world", "/vicon/cftarget_2/cftarget_2", rospy.Time(0))
    position_last_3,quaternion=tf.lookupTransform("/world", "/vicon/cftarget_3/cftarget_3", rospy.Time(0))
    position_last_4,quaternion=tf.lookupTransform("/world", "/vicon/cftarget_4/cftarget_4", rospy.Time(0))
    position_last_5,quaternion=tf.lookupTransform("/world", "/vicon/cftarget_5/cftarget_5", rospy.Time(0))
    position_last_6,quaternion=tf.lookupTransform("/world", "/vicon/cftarget_6/cftarget_6", rospy.Time(0))
    Op_point_2=np.array([position_last_1[0],position_last_1[1],Z])
    Op_point_4=np.array([position_last_2[0],position_last_2[1],Z])
    Op_point_5=np.array([position_last_3[0],position_last_3[1],Z])
    Op_point_7=np.array([position_last_4[0],position_last_4[1],Z])
    Op_point_1=np.array([position_last_5[0],position_last_5[1],Z])
    Op_point_8=np.array([position_last_6[0],position_last_6[1],Z])
    Op_point_group={
                    2:Op_point_2,
                    4:Op_point_4,
                    5:Op_point_5,
                    7:Op_point_7,
                    1:Op_point_1,
                    8:Op_point_8}



    ##def init(self,ID,target_ID,offset,master_point,a,b,h,phi_factor)
    Agent2=cfAct(2,[7,4],np.array([0,0,0]),False,1,0.6,0.3*np.array([[0,-1,0],[1,0,0]]),0.05)
    Agent4=cfAct(4,[2,5],np.array([-1,0,0]),False,1,0.6,0.3*np.array([[-1,0,0],[0,-1,0]]),0.05)
    Agent5=cfAct(5,[4,8],np.array([-1,1,0]),False,1,0.6,0.3*np.array([[0,1,0],[0,-1,0]]),0.05)
    Agent7=cfAct(7,[2,1],np.array([0,1,0]),False,1,0.6,0.3*np.array([[0,1,0],[0,-1,0]]),0.05)
    Agent1=cfAct(1,[7,8],np.array([0,2,0]),False,1,0.6,0.3*np.array([[0,1,0],[1,0,0]]),0.05)
    Agent8=cfAct(8,[5,1],np.array([-1,2,0]),False,1,0.6,0.3*np.array([[0,1,0],[-1,0,0]]),0.05)
    count=0
    
    while(True):
      t_last=rospy.Time.now()
      
      dt=(t_last-t_now2).to_sec()
      if dt>0.03:
        t_now2=rospy.Time.now()
        position_1, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_1/cftarget_1", rospy.Time(0))
        v_tar_1=(np.array(position_1)-np.array(position_last_1))/dt
        #######right here change v_tar[2]
        Op_point_2=np.array([position_last_1[0],position_last_1[1],Z])+np.array([v_tar_1[0],v_tar_1[1],0])
        Op_point_group[2]=Op_point_2
        cf_v_2=Agent2.update_cf_v(Op_point_2)
        ######here change height velocity
        cf_v2=np.array([cf_v_2[0],cf_v_2[1],H])
        position_last_1=position_1
        ##############################################################3
        position_2, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_2/cftarget_2", rospy.Time(0))
        v_tar_2=(np.array(position_2)-np.array(position_last_2))/dt
        #######right here change v_tar[2]
        Op_point_4=np.array([position_last_2[0]-0.2,position_last_2[1],Z])+np.array([v_tar_2[0],v_tar_2[1],0])
        Op_point_group[4]=Op_point_4
        cf_v_4=Agent4.update_cf_v(Op_point_4)
        ######here change height velocity
        cf_v4=np.array([cf_v_4[0],cf_v_4[1],H])
        position_last_2=position_2
        ##################################################################
        position_3, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_3/cftarget_3", rospy.Time(0))
        v_tar_3=(np.array(position_3)-np.array(position_last_3))/dt
        #######right here change v_tar[2]
        Op_point_5=np.array([position_last_3[0],position_last_3[1],Z])+np.array([v_tar_3[0],v_tar_3[1],0])
        Op_point_group[5]=Op_point_5
        cf_v_5=Agent5.update_cf_v(Op_point_5)
        ######here change height velocity
        cf_v5=np.array([cf_v_5[0],cf_v_5[1],H])
        position_last_3=position_3
        ####################################################################
        position_4, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_4/cftarget_4", rospy.Time(0))
        v_tar_4=(np.array(position_4)-np.array(position_last_4))/dt
        #######right here change v_tar[2]
        Op_point_7=np.array([position_last_4[0],position_last_4[1],Z])+np.array([v_tar_4[0],v_tar_4[1],0])
        Op_point_group[7]=Op_point_7
        cf_v_7=Agent7.update_cf_v(Op_point_7)
        ######here change height velocity
        cf_v7=np.array([cf_v_7[0],cf_v_7[1],H])
        position_last_4=position_4
        ###################################################################3
        position_5, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_5/cftarget_5", rospy.Time(0))
        v_tar_5=(np.array(position_5)-np.array(position_last_5))/dt
        #######right here change v_tar[2]
        Op_point_1=np.array([position_last_5[0],position_last_5[1],Z])+np.array([v_tar_5[0],v_tar_5[1],0])
        Op_point_group[1]=Op_point_1
        cf_v_1=Agent1.update_cf_v(Op_point_1)
        ######here change height velocity
        cf_v1=np.array([cf_v_1[0],cf_v_1[1],H])
        position_last_5=position_5
        ###################################################################
        position_6, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_6/cftarget_6", rospy.Time(0))
        v_tar_6=(np.array(position_6)-np.array(position_last_6))/dt
        #######right here change v_tar[2]
        Op_point_8=np.array([position_last_6[0],position_last_6[1],Z])+np.array([v_tar_6[0],v_tar_6[1],0])
        Op_point_group[8]=Op_point_8
        cf_v_8=Agent8.update_cf_v(Op_point_8)
        ######here change height velocity
        cf_v8=np.array([cf_v_8[0],cf_v_8[1],H])
        position_last_6=position_6
        ####################
        ####################
        temp_cf=cf8.position()
        if(np.linalg.norm(np.array([temp_cf[0],temp_cf[1]])-np.array([position_last_6[0],position_last_6[1]]))<0.3):
          temp_h=cf8.position()
          H=position_last_6[2]-temp_h[2]
          
      if count==0:
        cf2=allcfs.crazyfliesById[2]
        pos2=cf2.position()
        cf2.cmdFullState(pos=pos2,vel=cf_v2,acc=np.array([0.0,0.0,6.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
        
      if count==1:
        cf4=allcfs.crazyfliesById[4]
        pos4=cf4.position()
        cf4.cmdFullState(pos=pos4,vel=cf_v4,acc=np.array([0.0,0.0,6.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
        
      if count==2:
        cf5=allcfs.crazyfliesById[5]
        pos5=cf5.position()
        cf5.cmdFullState(pos=pos5,vel=cf_v5,acc=np.array([0.0,0.0,7.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
  
      if count==3:
        cf7=allcfs.crazyfliesById[7]
        pos7=cf7.position()
        cf7.cmdFullState(pos=pos7,vel=cf_v7,acc=np.array([0.0,0.0,6.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
        
      if count==4:
        cf1=allcfs.crazyfliesById[1]
        pos1=cf1.position()
        cf1.cmdFullState(pos=pos1,vel=cf_v1,acc=np.array([0.0,0.0,6.0]),yaw=0,omega=np.array([0,0,0]))
        count=count+1
        
      if count==5:
        cf8=allcfs.crazyfliesById[8]
        pos8=cf8.position()
        cf8.cmdFullState(pos=pos8,vel=cf_v8,acc=np.array([0.0,0.0,6.0]),yaw=0,omega=np.array([0,0,0]))
        count=0
        
   
      

      
