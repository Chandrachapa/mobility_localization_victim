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
import xpress as xp
xp.controls.outputlog = 0

#importing all data
# =================== 0. Definitions ==================
def localization(N,nodeids,nodeids_loc,gps,benchids,bench_locs):
#def localization(mx,my,benchids_node,benchids_dst,nodeids_loc):
    #generating the measurements received
    S = range(N)
    notsatisfied = []#to store the count of not satisfied constraints
    itercount = []#to store the number of nodes
    mx, my, gpsradius = {},{},{}
    
    #adding benchmark nodes info to an array
    i = 0
    print('nodeids',nodeids)
    for iter in nodeids:  #i is in the order of ids in nodeids
      mx[i] = nodeids_loc[0]
      my[i] = nodeids_loc[1]
      gpsradius[i] = gps
      itercount.append(iter)
      i+=1
       
    for iter in bench_locs:
        if (type(bench_locs[iter])!= str and math.isnan(bench_locs[iter])): 
            print('yes',bench_locs[iter])
        else:
            string = bench_locs[iter].split(";")
            print('no',iter,' ',bench_locs[iter],'',string[0])
            
            
    #angle measurements
    #benchmark aoa
    #CSP formulation
    
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
      print(mx)
      p.addConstraint((x[i]-mx[i]) <= 500)
      p.addConstraint((y[i]-my[i]) <= 500)
        
    #solving using the set of measurements the location of the unknown device
    p.setObjective(1)
    p.solve()
    #solving
    print('solution: ', p.getSolution())
    
    for i in S:#each node
        print(p.getSolution(x[i]))
        if (math.isnan(p.getSolution(x[i]))):
            notsatisfied.append(1) 
        else:
            notsatisfied.append(0)
    print("sol: ",p.getSolution(x))
    return p.getSolution(x[0]),p.getSolution(y[0]),notsatisfied

# =================== 1. Import data =========================================================================================================================================================
df = pd. read_excel (r'csp.xlsx', sheet_name='Input')
rows = 100
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
callnodeid_gps={}
weight=[]
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
        callnodeid_gps[callidnodeid] = gps 
        
        #create hashmap to store the benchmark locs for each callnodeid
        callnodeid_benchlocs[callidnodeid]=benchlocs
        callnodeid_benchids[callidnodeid]= benchids        
        
        #compute the number of nodes that take part in this call transfer
        if (callid_nbnodes.get(callidnodeid) == None):
            callid_nbnodes[callidnodeid] = 1
        else:
            callid_nbnodes[callidnodeid] = callid_nodes[callidnodeid]+1
        #compute the weight of data in this call transfer
        callid_value[callidnodeid] = 1/(update_time-timestamp)
        freshness =1/(update_time-timestamp)
        weight.append(freshness)
        N = len(nodes_considered)
        
        nodeids_loc = (nodelocx,nodelocy)
        
    #simulated annealing
    #inputs
    Fpast = 1.0
    lamda = 0.5
    exponential = 0.25
    threshold = 0.5
   # weight=1
    weighta = np.array(weight)
    #keep finding a better solution until the threshold is satisfied
    print("!!!!!!!!!!!0",exponential<threshold or  exp !=0)
    print('nbnodes',len(nodes_considered)!=0)
    while (exponential<threshold or  exponential[0] !=0):
        print("1",nodeids_loc)
        if len(nodes_considered)==0 :#if the nbnodes considered during this period is null then terminate the computing
            break
        x,y,notsatisfied =localization(N,nodes_considered,nodeids_loc,callnodeid_gps,callnodeid_benchids,callnodeid_benchlocs)
        print("1.5",x)
        #weight is an array of all the callnodeids freshness of data
        print("2",sum(notsatisfied),weighta)
        Fcurrent = weighta.T*sum(notsatisfied)
        print("exponential",exponential,Fcurrent)
        exponential = (Fcurrent-Fpast)/lamda
        
        #update the est solution -> nodeids_loc
        
        nodeids_loc = (x,y)
        Fpast = Fcurrent
        print("exp",exponential)
        exp = exponential[0]
        if(exp==0):#break condition
            break
        
    nodes_considered=[]
    weight =[]
    print("fin")