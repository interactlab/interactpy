# TODO(allanzhou): All the stuff in here should eventually be
# moved into the appropriate files.

import openravepy
import numpy as np
from scipy.interpolate import *
import math
import SimpleUtils
from numpy import linalg as LA
import copy

def create_waypoint(joint_pos, first, timestep):
    if(first):
        val=0
    else:
        val=timestep*2
    waypoint= np.array([val, joint_pos[0], joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5],joint_pos[6],joint_pos[7]])
    return waypoint

def make_const_vel(bin_traj, env):
    g1=openravepy.ConfigurationSpecification.Group()
    g1.name="joint_values ADA 0 1 2 3 4 5 6 7"
    g1.interpolation="linear"
    g1.dof=8
    g1.offset=0
    config_spec=openravepy.ConfigurationSpecification()
    config_spec.AddDeltaTimeGroup()
    config_spec.AddGroup(g1)
    new_bin_traj=openravepy.RaveCreateTrajectory(env, "")
    new_bin_traj.Init(config_spec)
    #print new_bin_traj.GetConfigurationSpecification()

    t=0
    timestep=0.005
    dist_allowed_per_timestep=0.05
    data= bin_traj.Sample(t)
    prev_joint=np.array([data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]])
    t+=timestep
    final_waypoints=[]
    curr_index=0
    first=True
    
    while t<bin_traj.GetDuration():
        data= bin_traj.Sample(t)
        joint=np.array([data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]])    

        diff_vector=joint-prev_joint
        dist= LA.norm(diff_vector)

        if(dist<dist_allowed_per_timestep): #difference is so small, get rid of this waypoint
            donothing=0

        elif(dist==dist_allowed_per_timestep): #good, keep this waypoint
            final_waypoints.append(joint)
            new_bin_traj.Insert(curr_index,create_waypoint(joint, first, timestep),True)
            curr_index+=1
            prev_joint=joint

        else: #difference is too big, add points in between
            lng=np.arange(0,dist,dist_allowed_per_timestep)
            offset=np.outer(diff_vector/dist,lng)
            intermed=np.outer(prev_joint,np.ones(offset.shape[1]))+offset #add this difference to prev_j
            for col in range(intermed.shape[1]):
                final_waypoints.append(intermed[:,col])
                prev_joint= intermed[:,col]

                new_bin_traj.Insert(curr_index,create_waypoint(intermed[:,col], first, timestep),True)
                if(first): 
                    first=False

                curr_index+=1

        t+=timestep
    return new_bin_traj