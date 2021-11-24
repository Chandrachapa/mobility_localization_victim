import xpress as xp
import math
import random
#from pyibex import *
#from codac import *
import math
import random
import time
import numpy as np
import pandas as pd
from scipy.spatial import distance
from sklearn.metrics import mean_squared_error
from numpy.linalg import inv
#TO INSTALL XPRESS: RUN 'pip install xpress' 

#initialization
nodeids_loc,nodeids_radius,benchids_node,benchids_dst,d,benchids_aoa,a={},{},{},{},{},{},{}
mx, my, gpsradius = {},{},{}
itercount=[]

#weights
c = np.array([2,2,3,3,1,3,2,5])#weights

#collect data
nodeids = ['55','10','45']#ids

#initial positions at time t1
nodeids_loc['55'] =(5,5)
nodeids_loc['10'] =(10,1)
nodeids_loc['45'] =(1,3)

#gps accuracy in radii time t1
nodeids_radius['55'] =2
nodeids_radius['10'] =1
nodeids_radius['45'] =1

#benchmark nodes of each id initially time t1
benchids_node['55'] = ('10','45')
benchids_node['10'] = ('45','55')
benchids_node['45'] = ('55','10')

#benchmark distance initially time t1
benchids_dst['55'] = (5,4)
benchids_dst['10'] = (8,5)
benchids_dst['45'] = (4,8)

#adding benchmark nodes info to an array
i = 0
for iter in nodeids:  #i is in the order of ids in nodeids
  mx[i] = nodeids_loc[iter][0]
  my[i] = nodeids_loc[iter][1]
  gpsradius[i] = nodeids_radius[iter]
  itercount.append(iter)
  i+=1

#generating the measurements received
N = 3#total nodes 
S = range(N)

#angle measurements
#benchmark aoa
for i in S:#each node
  astar = []
  for kk in range(len(benchids_node[itercount[i]])):
     deltay=nodeids_loc[itercount[i]][1] - nodeids_loc[benchids_node[itercount[i]][kk]][1]#kkth bench node y
     deltax=nodeids_loc[itercount[i]][0] - nodeids_loc[benchids_node[itercount[i]][kk]][0]#kkth bench node y

     astar=math.atan2(deltay,deltax)  
     if (benchids_aoa.get(itercount[i])==None):
       benchids_aoa[itercount[i]] = astar
     else:
       lst0 =benchids_aoa[itercount[i]]
       lst1 = astar
       benchids_aoa[itercount[i]] = (lst0,lst1)

#localization function 
def localization(mx,my,benchids_node,benchids_dst,nodeids_loc):
    #location of unknown device #variables x and y 
    x = [xp.var() for i in S]
    y = [xp.var() for i in S]

    #CSP formulation
    p = xp.problem()
    for i in S:
      p.addVariable(x[i])#x and y coordinates
      p.addVariable(y[i])

    # polynomial constraint
    #gps constraints
    for i in S:#each node
      p.addConstraint((x[i]-mx[i])**2+ (y[i]-my[i])**2<=gpsradius[i]) #own position gps

      #distance measurements using rssi values
      #for each node, for the length of benchmarks=nb distance constraints
      for kk in range(len(benchids_node[itercount[i]])):
        d[kk] = benchids_dst[itercount[i]][kk]#distance value set in benchid order #true distance
        bxy = nodeids_loc[benchids_node[itercount[i]][kk]]
        alpharssi = 0.5*d[kk]*2#rssi error
        d[kk] = d[kk]+random.choice([-1, 1])*random.uniform(0,alpharssi)#estimated distance

        p.addConstraint(((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)**0.5<=d[kk]*(1+alpharssi)) #distance constraint rssi
        p.addConstraint(((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)**0.5>=d[kk]*(1-alpharssi)) #distance constraint rssi
        #add toa constraint#if incoverage
        
        #angle measurment constraints
        op=0.1#aoa error
        a[kk] = benchids_aoa[itercount[i]][kk]#in benchid order #true angle
        a[kk]=a[kk] + random.choice([-1, 1])*random.uniform(0,op)

        amin=a[kk]-op
        amax=a[kk]+op
        if amin>-3.14159 and amax<=3.14159 :
          yyy=0
          p.addConstraint((y[i]-bxy[1])/(x[i]-bxy[0])<=math.tan(amax))
          p.addConstraint((y[i]-bxy[1])/(x[i]-bxy[0])>=math.tan(amin))
        elif amin<=-3.14159 :
          p.addConstraint((y[i]-bxy[1])/((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)<=math.sin(6.28318-amin))
          p.addConstraint((y[i]-bxy[1])/((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)>=math.sin(amax))
        elif amax>3.14159 :
          p.addConstraint((y[i]-bxy[1])/((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)<=math.sin(amax-6.28318))
          p.addConstraint((y[i]-bxy[1])/((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)>=math.sin(amin))

    #solving using the set of measurements the location of the unknown device
    p.setObjective(1)
    p.solve()
    return x,y


#if (solution) is ((0,0),(0,0),(0,0)) 
bfar = np.zeros((3, 4))
bmiddle = np.zeros((3, 4))
bbest = np.zeros((3, 4))
#known: weight of each constraint
#weights = np.zeros((1,3))
weights = np.random.randn(4)
print('weight',weights)
#use a boolen vector to select the constraints

C_v = weights 
n = 4
print('weights:',weights)

with Model() as M:
  x = M.variable([n,1], Domain.binary())

  M.constraint(Expr.sum(x,0), Domain.greaterThan(0.1))#minimum constraints selected : greater than includes equality# hence put 0.1 to ensure selecting at least one constraint
  M.objective(ObjectiveSense.Maximize, Expr.dot(C_v ,x))
  M.solve()
  print('\nsolution:', x.level())