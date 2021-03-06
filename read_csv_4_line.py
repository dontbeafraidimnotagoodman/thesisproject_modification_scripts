import csv 
import numpy as np
from mpl_toolkits import mplot3d



import matplotlib.pyplot as plt



fig = plt.figure()
ax = plt.axes(projection='3d')



f=open('consensus_line_2.csv')
csv_f = csv.reader(f)

agent2=[]
agent4=[]
agent5=[]
agent7=[]

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
  
  
  dt.append(float(row[4]))
  i=i+1
  print(i)
  if i>(1770-2):
    break


dt=np.array(dt)
agent5=np.array(agent5)
agent2=np.array(agent2)
agent4=np.array(agent4)
agent7=np.array(agent7)

ax.plot3D((0.2)*dt,(0.2)*dt,dt,'green')
ax.plot3D(agent2[:,0],agent2[:,1],dt,'gray')
ax.plot3D(agent4[:,0],agent4[:,1],dt,'gray')
ax.plot3D(agent5[:,0],agent5[:,1],dt,'gray')
ax.plot3D(agent7[:,0],agent7[:,1],dt,'gray')
ax.plot([agent2[0,0],agent4[0,0]],[agent2[0,1],agent4[0,1]],[dt[0],dt[0]],'blue')
ax.plot([agent4[0,0],agent5[0,0]],[agent4[0,1],agent5[0,1]],[dt[0],dt[0]],'blue')
ax.plot([agent2[0,0],agent7[0,0]],[agent2[0,1],agent7[0,1]],[dt[0],dt[0]],'blue')

ax.plot([agent2[500,0],agent4[500,0]],[agent2[500,1],agent4[500,1]],[dt[500],dt[500]],'blue')
ax.plot([agent4[500,0],agent5[500,0]],[agent4[500,1],agent5[500,1]],[dt[500],dt[500]],'blue')
ax.plot([agent2[500,0],agent7[500,0]],[agent2[500,1],agent7[500,1]],[dt[500],dt[500]],'blue')

ax.plot3D((agent2[:,0]+agent4[:,0]+agent5[:,0]+agent7[:,0])/4,(agent2[:,1]+agent4[:,1]+agent5[:,1]+agent7[:,1])/4,dt,'red')
plt.show()
