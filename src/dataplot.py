#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 22 14:04:02 2018

@author: lhuang28
"""
#%%
from scipy import spatial
import numpy as np
import matplotlib.pyplot as plt
import pdb
color_dict = {0:(0,0, 1), 1:(0,1,1), 2:(1, 0,0), 3:(0.580, 0, 0.827), 4: (0, 0.5, 1),
                5:(0,1,0), 6:(0.510, 0, 0.294), 7:(1,1,1), 8:(0,0,0)}
        
file = open('actor_trajectory.txt','r')
data = file.readlines()

trajectory = []
for idx, line in enumerate(data):
    trajectory.append([])
    elements = line.split()
    data_list = [float(i) for i in elements]
    for item in data_list:
        trajectory[idx].append(item)

file.close()

plt.figure(figsize = (30,30))

i = 0        
actor1, = plt.plot(np.asarray(range(len(trajectory[i]))).astype(np.float)/29.0,  \
                   np.asarray(trajectory[i]).astype(np.float), color = 'r',linewidth = 5.0 )
i+=1
actor2, = plt.plot(np.asarray(range(len(trajectory[i]))).astype(np.float)/29.0,  \
                   np.asarray(trajectory[i]).astype(np.float), color = 'b',linewidth = 5.0 )
i+=1
actor3, = plt.plot(np.asarray(range(len(trajectory[i]))).astype(np.float)/29.0,  \
                   np.asarray(trajectory[i]).astype(np.float), color = 'k',linewidth = 5.0 )
i+=1
actor4, = plt.plot(np.asarray(range(len(trajectory[i]))).astype(np.float)/29.0, \
                   np.asarray(trajectory[i]).astype(np.float), color = 'g',linewidth = 5.0 )


file = open('cam_trajectory.txt','r')
data = file.readlines()

trajectory = []
for idx, line in enumerate(data):
    trajectory.append([])
    elements = line.split()
    data_list = [float(i) for i in elements]
    for item in data_list:
        trajectory[idx].append(item)

file.close()
#i = 0
#cam1, = plt.plot(np.asarray(range(len(trajectory[i]))).astype(np.float)/29.0,  \
#                 np.asarray(trajectory[i]).astype(np.float), linestyle='--', \
#                 marker='o',markersize=18,markevery =200, color = color_dict[1],linewidth = 3.0 )
#i+=1
#cam2, = plt.plot(np.asarray(range(len(trajectory[i]))).astype(np.float)/29.0,  \
#                 np.asarray(trajectory[i]).astype(np.float), linestyle='--', \
#                 marker='o',markersize=18, markevery =200, color =color_dict[3],linewidth = 3.0 )
#i+=1
#cam3, = plt.plot(np.asarray(range(len(trajectory[i]))).astype(np.float)/29.0,  \
#                 np.asarray(trajectory[i]).astype(np.float), linestyle='--', \
#                 marker='o', markersize=18, markevery =200,  color = color_dict[5],linewidth = 3.0 )
#plt.legend([actor1, actor2, actor3, actor4, cam1, cam2, cam3], \
#           ['Alice', 'Bob', 'Candice', 'Dave', 'Cam: AC0', 'Cam: AC1', 'Cam: Start'], prop={'size': 45})
opacity = 0.35
for lap in range(13):
    i = 0
    newtraj= []
    newtraj.append([])
    k= 0
    
    traj = lap/13.+ 1./13.*(13.*np.asarray(trajectory[i]).astype(np.float)\
                     -np.floor(13.*np.asarray(trajectory[i]).astype(np.float)))
    newtraj[0].extend([traj[0]])
    for j in range(1,np.shape(traj)[0]):
        if abs(traj[j]-traj[j-1])<0.5/13.:
            newtraj[k].extend([traj[j]])
        else:
            k+=1
            newtraj.append([])
            newtraj[k].extend([traj[j]])
#%% 
#    
    cam1, = plt.plot(np.asarray(range(0, len(newtraj[0]))).astype(np.float)/29.0,  \
                     np.asarray(newtraj[0]), linestyle='--', \
                          color = color_dict[1],linewidth = 8.0 , alpha=opacity)
                    
    for j in range(1,k+1):    
        rg0 = 0 

        for n in range(j): 
            rg0 += len(newtraj[n])
        rg1 = rg0 + len(newtraj[j])
  
        cam1, = plt.plot(np.asarray(range(rg0, rg1)).astype(np.float)/29.0,  \
                         np.asarray(newtraj[j]), linestyle='--', \
                          color = color_dict[1],linewidth = 8.0 , alpha=opacity)
    i+=1
    newtraj= []
    newtraj.append([])
    k= 0
    
    traj = lap/13.+ 1./13.*(13.*np.asarray(trajectory[i]).astype(np.float)\
                     -np.floor(13.*np.asarray(trajectory[i]).astype(np.float)))
    newtraj[0].extend([traj[0]])
    for j in range(1,np.shape(traj)[0]):
        if abs(traj[j]-traj[j-1])<0.5/13.:
            newtraj[k].extend([traj[j]])
        else:
            k+=1
            newtraj.append([])
            newtraj[k].extend([traj[j]])
    cam2, = plt.plot(np.asarray(range(0, len(newtraj[0]))).astype(np.float)/29.0,  \
                 np.asarray(newtraj[0]), linestyle='--', \
                      color = color_dict[3],linewidth = 8.0 , alpha=opacity)
    for j in range(1,k+1):
        rg0 = 0 

        for n in range(j): 
            rg0 += len(newtraj[n])
        rg1 = rg0 + len(newtraj[j])
        cam2, = plt.plot(np.asarray(range(rg0, rg1)).astype(np.float)/29.0,  \
                         np.asarray(newtraj[j]), linestyle='--', \
                          color = color_dict[3],linewidth = 8.0 , alpha=opacity)
    i+=1
    newtraj= []
    newtraj.append([])
    k= 0
    
    traj = lap/13.+ 1./13.*(13.*np.asarray(trajectory[i]).astype(np.float)\
                     -np.floor(13.*np.asarray(trajectory[i]).astype(np.float)))
    newtraj[0].extend([traj[0]])
    for j in range(1,np.shape(traj)[0]):
        if abs(traj[j]-traj[j-1])<0.5/13.:
            newtraj[k].extend([traj[j]])
        else:
            k+=1
            newtraj.append([])
            newtraj[k].extend([traj[j]])
    cam3, = plt.plot(np.asarray(range(0, len(newtraj[0]))).astype(np.float)/29.0,  \
                     np.asarray(newtraj[0]), linestyle='--', \
                          color = color_dict[5],linewidth = 8.0 , alpha=opacity)
       
    for j in range(1,k+1):
        rg0 = 0 

        for n in range(j): 
            rg0 += len(newtraj[n])
        rg1 = rg0 + len(newtraj[j])
        cam3, = plt.plot(np.asarray(range(rg0, rg1)).astype(np.float)/29.0,  \
                         np.asarray(newtraj[j]), linestyle='--', \
                          color = color_dict[5],linewidth = 8.0 , alpha=opacity)

plt.legend([actor1, actor2, actor3, actor4, cam1, cam2, cam3], \
           ['Alice', 'Bob', 'Candice', 'Dave', 'Cam: AC0', 'Cam: AC1', 'Cam: Start'], prop={'size': 45})

for i in range(13):
    plt.plot(np.linspace(0,len(trajectory[0])/29.,10), (i+1)/13.*np.ones(10),color = 'k',linewidth = .5 )
ymin = -0.05
ymax = 1.0
plt.ylim(ymin,ymax)
plt.xlabel("Time (sec)", fontsize=50)
#plt.xlabel("Velocity magnitude ($\mu m/s$)", fontsize=50)
plt.ylabel('Progress', fontsize=50)
plt.xticks(fontsize=50)
plt.yticks(fontsize=50)
plt.savefig('Race.png')


#plt.show()
        
