#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 20:26:40 2021

This basically calculates the minimum time it takes to perform a 
movement of so and so many steps under the condition that a certain 
speed must not be exceeded.

@author: josef
"""
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# movement spec
delta_s = 100
w_s = 0
w_e = 0

# machine params
w_max = 20
acc = 600
alpha = 2*np.pi/(400*4)

delta_theta = delta_s * alpha
delta_t1 = (w_max - w_s)/acc
delta_t3 = (w_max - w_e)/acc

delta_theta1 = acc * np.square(delta_t1) / 2
delta_theta3 = acc * np.square(delta_t3) / 2

if delta_theta < (delta_theta1 + delta_theta3): 
    # We cannot even go to full speed because we are not allowed to do that many steps
    w_m = np.sqrt(acc*delta_theta + np.square(w_s)/2 + np.square(w_e/2))
    t_min = (2*w_m - w_s - w_e)/acc
    print("Cannot reach full speed")
else:
    # we can fully accelerate and decelerate. 
    t_min = delta_t1 + delta_t3 + (delta_theta - delta_theta1 - delta_theta3)/w_max
    print("Reach full speed")
    
print("t_min = " + str(t_min*1000))