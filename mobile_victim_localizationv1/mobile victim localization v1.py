#obtaining data from excel 
from pyibex import *
from codac import *
import math
import random
import time
import numpy as np
import pandas as pd
from scipy.spatial import distance
from sklearn.metrics import mean_squared_error

#importing all data
# =================== 0. Definitions ==================
def localization(N,nodeids,nodeids_loc,benchids,bench_locs):
#def localization(mx,my,benchids_node,benchids_dst,nodeids_loc):
    #generating the measurements received
    #N = 3#total nodes localized
    S = range(N)
    notsatisfied = []
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
           lst0 = benchids_aoa[itercount[i]]
           lst1 = astar
           benchids_aoa[itercount[i]] = (lst0,lst1)
        
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
    for i in S:#each node
        if (x[i]==nan):
            notsatisfied.append(1) 
        else:
            notsatisfied.append(0)
    return x,y,notsatisfied

# =================== 1. Import data =========================================================================================================================================================
df = pd. read_excel (r'csp.xlsx', sheet_name='Input')
rows = df.shape[0]
columns = df.shape[1]
random.seed(100)

#assume only gps constraints are considered for each node

#initial update time is set to 120 = 2 mins
update_time  = 120

#store the initial solution of locations of nodes in a hashmap
node_loc = {}

nodes_considered  = []
callid_nbnodes = {}
callnodeid_benchids = {}
callnodeid_benchlocs = {}
callid_value = {}

#received calls
for i in range(1,rows):#taking each data received
    timestamp = df.loc[i][9] # time stamp
      
    if (timestamp<update_time):#considered time interval
        
        callidnodeid = df.loc[i][2]
        nodeid = df.loc[i][3]
        
        nodelocx =  df.loc[i][4]
        nodelocy = df.loc[i][5]
        
        gps = df.loc[i][6]
        
        benchids = df.loc[i][7]
        benchlocs =  df.loc[i][8]
        
        #create an array to store the nodes considered in this update time interval
        nodes_considered.append(callidnodeid) 
        
        #store gps in a hashmap
        callnodeid_benchlocs[callidnodeid] = gps
        
        #create hashmap to store the benchmark locs for each callnodeid
        callnodeid_benchlocs[callidnodeid],benchlocs
        callnodeid_benchids[callidnodeid]= benchids        
        
        #compute the number of nodes that take part in this call transfer
        if (callid_nbnodes.get(callidnodeid) == None):
            callid_nbnodes[callidnodeid] = 1
        else:
            callid_nbnodes[callidnodeid] = callid_nodes[callidnodeid]+1
        #compute the weight of data in this call transfer
        callid_value[callidnodeid] = 1/(update_time-timestamp)
        N = len(nodes_considered)
        
        nodeids_loc = (nodelocx,nodelocy)
        
    x,y,notsatisfied =localization(N,nodes_considered,nodeids_loc,callnodeid_benchids,callnodeid_benchlocs)
    nodes_considered=[]
    
    if sum(notsatisfied) == nodeids.length():
        print("finished")
    else:
        while(solution_satis_prob < threshold):
            x,y,notsatisfied =localization(nodeids,benchids_of_node)
        else: 
            print("finished 2")
      