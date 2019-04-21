import csv 
import numpy as np
from mpl_toolkits import mplot3d
import math



import matplotlib.pyplot as plt



fig = plt.figure()
ax = plt.axes(projection='3d')

def cal_Op_pos(DT,v_max,center,r,r_max):
    ## w = v/r
    a=center[0]-r*(math.sin((v_max/r_max)*DT))## vel ctrl need to be before DT
    b=center[1]+r*(math.cos((v_max/r_max)*DT))## circle point(100, -80)
    return np.array([a,b]) ## when it first begin, there should be no jump

f=open('6_consensus_circle_best.csv')
csv_f = csv.reader(f)

agent2=[]
agent4=[]
agent5=[]
agent7=[]
agent1=[]
agent8=[]
dt=[]
i=0
csv_f=csv.reader(f)

for row in csv_f:
  a=(row[0])
  a=a[1:-1]
  a.strip()
  a=np.fromstring(a, dtype=float, sep=' ')
  agent2.append(a)
  a=(row[1])
  a=a[1:-1]
  a.strip()
  a=np.fromstring(a, dtype=float, sep=' ')
  agent4.append(a)
  a=(row[2])
  a=a[1:-1]
  a.strip()
  a=np.fromstring(a, dtype=float, sep=' ')
  agent5.append(a)
  a=(row[3])
  a=a[1:-1]
  a.strip()
  a=np.fromstring(a, dtype=float, sep=' ')
  agent7.append(a)
  a=(row[4])
  a=a[1:-1]
  a.strip()
  a=np.fromstring(a, dtype=float, sep=' ')
  agent1.append(a)
  a=(row[5])
  a=a[1:-1]
  a.strip()
  a=np.fromstring(a, dtype=float, sep=' ')
  agent8.append(a)
  
  dt.append(float(row[6]))
  i=i+1
  print(i)
  if i>(3400):
    break

x_coor=[None]*len(dt)
y_coor=[None]*len(dt)
for i in range(len(dt)):
  m=cal_Op_pos(dt[i]-(3.1415926)/4,0.5,np.array([0,0]),0.63*math.sqrt(2),1*math.sqrt(2))
  x_coor[i]=m[0]
  y_coor[i]=m[1]
dt=np.array(dt)
agent5=np.array(agent5)
agent2=np.array(agent2)
agent4=np.array(agent4)
agent7=np.array(agent7)
agent1=np.array(agent1)
agent8=np.array(agent8)
##ax.plot3D(0.225*dt,0.225*dt,dt,'green')
ax.plot3D(x_coor,y_coor,dt,'green')
ax.plot3D(agent2[:,0],agent2[:,1],dt,'gray')
ax.plot3D(agent4[:,0],agent4[:,1],dt,'gray')
ax.plot3D(agent5[:,0],agent5[:,1],dt,'gray')
ax.plot3D(agent7[:,0],agent7[:,1],dt,'gray')
ax.plot3D(agent1[:,0],agent1[:,1],dt,'gray')
ax.plot3D(agent8[:,0],agent8[:,1],dt,'gray')
ax.plot3D((agent2[:,0]+agent4[:,0]+agent5[:,0]+agent7[:,0]+agent1[:,0]+agent8[:,0])/6,(agent2[:,1]+agent4[:,1]+agent5[:,1]+agent7[:,1]+agent1[:,1]+agent8[:,1])/6,dt,'red')
ax.plot([agent2[0,0],agent4[0,0]],[agent2[0,1],agent4[0,1]],[dt[0],dt[0]],'blue')
ax.plot([agent4[0,0],agent5[0,0]],[agent4[0,1],agent5[0,1]],[dt[0],dt[0]],'blue')
ax.plot([agent5[0,0],agent8[0,0]],[agent5[0,1],agent8[0,1]],[dt[0],dt[0]],'blue')
ax.plot([agent8[0,0],agent1[0,0]],[agent8[0,1],agent1[0,1]],[dt[0],dt[0]],'blue')
ax.plot([agent1[0,0],agent7[0,0]],[agent1[0,1],agent7[0,1]],[dt[0],dt[0]],'blue')
ax.plot([agent7[0,0],agent2[0,0]],[agent7[0,1],agent2[0,1]],[dt[0],dt[0]],'blue')

ax.plot([agent2[1000,0],agent4[1000,0]],[agent2[1000,1],agent4[1000,1]],[dt[1000],dt[1000]],'blue')
ax.plot([agent4[1000,0],agent5[1000,0]],[agent4[1000,1],agent5[1000,1]],[dt[1000],dt[1000]],'blue')
ax.plot([agent5[1000,0],agent8[1000,0]],[agent5[1000,1],agent8[1000,1]],[dt[1000],dt[1000]],'blue')
ax.plot([agent8[1000,0],agent1[1000,0]],[agent8[1000,1],agent1[1000,1]],[dt[1000],dt[1000]],'blue')
ax.plot([agent1[1000,0],agent7[1000,0]],[agent1[1000,1],agent7[1000,1]],[dt[1000],dt[1000]],'blue')
ax.plot([agent7[1000,0],agent2[1000,0]],[agent7[1000,1],agent2[1000,1]],[dt[1000],dt[1000]],'blue')
plt.show()
