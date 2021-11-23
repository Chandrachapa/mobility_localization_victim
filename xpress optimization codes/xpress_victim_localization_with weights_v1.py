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
mx, my,gpsradius = {},{},{}
itercount=[]

#weights
c = np.array([2,2,3,3,1,3,2,5])#weights

#collect data
nodeids = ['55','10','45']#ids

#initial positions
nodeids_loc['55'] =(5,5)
nodeids_loc['10'] =(10,1)
nodeids_loc['45'] =(1,3)

#gps accuracy in radii
nodeids_radius['55'] =2
nodeids_radius['10'] =1
nodeids_radius['45'] =1

#benchmark nodes of each id initially
benchids_node['55'] = ('10','45')
benchids_node['10'] = ('45','55')
benchids_node['45'] = ('55','10')

#benchmark distance initially
benchids_dst['55'] = (5,4)
benchids_dst['10'] = (8,5)
benchids_dst['45'] = (4,8)

i = 0
for iter in nodeids:  #i is in the order of ids in nodeids
  mx[i] = nodeids_loc[iter][0]
  my[i] = nodeids_loc[iter][1]
  gpsradius[i] = nodeids_radius[iter]
  itercount.append(iter)
  i+=1

N = 3#total nodes 
S = range(N)

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

  #rssi constraints
  #for each node, for the length of benchmarks=nb distance constraints
  for kk in range(len(benchids_node[itercount[i]])):
     d[kk] = benchids_dst[itercount[i]][kk]#distance value set in benchid order #true distance
     bxy = nodeids_loc[benchids_node[itercount[i]][kk]]
     alpharssi = 0.5*d[kk]*2#rssi error
     d[kk] = d[kk]+random.choice([-1, 1])*random.uniform(0,alpharssi)#estimated distance

     p.addConstraint(((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)**0.5<=d[kk]*(1+alpharssi)) #distance constraint rssi
     p.addConstraint(((x[i]-bxy[0])**2+(y[i]-bxy[1])**2)**0.5>=d[kk]*(1-alpharssi)) #distance constraint rssi
     #add toa constraint#if incoverage
     
     #aoa constraints
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

#solving
p.setObjective(1)
p.solve()

print('solution: ', p.getSolution())

#if (solution) is ((0,0),(0,0),(0,0)) 
bfar = np.zeros((3, 4))
bmiddle = np.zeros((3, 4))
bbest = np.zeros((3, 4))
#known: weight of each constraint
#weights = np.zeros((1,3))
weights = np.random.randn(3)
print('weight',weights)
#use a boolen vector to select the constraints

CN = 9
#objective function 
#sum(sum(np.matmul(weights,boolenarr.transpose())))
#changes the selected combination of constraints and try to get the solution 

#repeat until a solution is obtained

#maximizing the minimum constraints satisfied formulation

#known: constraints set
#optimization problem formulation
#variable optimized: boolean vector/array
import cvxpy as cp
# Construct the problem.
boolenarr =cp.Variable((3))

print("booleanarr",boolenarr)
#x = cp.Variable(n)
y = weights@boolenarr
objective = cp.Maximize(sum(y))
constraints = [sum(boolenarr)>=0,sum(boolenarr)<=10]
prob = cp.Problem(objective, constraints)

# The optimal objective value is returned by `prob.solve()`.
prob.solve()
# The optimal value for x is stored in `x.value`.
print(boolenarr.value)
# The optimal Lagrange multiplier for a constraint is stored in
# `constraint.dual_value`.
#print(constraints[0].dual_value)