#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 22 13:03:08 2020

This file contains helper functions to transfor coordinates 
given in (R, phi) to (alpha_x, alpha_y) and back. 

Angles are all in radian, dimensions are supposed in mm. 

@author: lks and josef
"""

import numpy as np

#everyting in mm
base = {"posx_x": -60, "posx_y": 35, "posx_r": 25, "posy_x": 60, "posy_y": 45, "posy_r": 35, "roller_r":5}


"""
PolarToangle(): 
    Transforms a pair of machine coordinates for POSX and POSY axis to 
    a polar representation with radius and angle. 
    
    Rphi = [R, phi] Radius in mm, phi in radian
    base: dictionary with origins and radii of both positioning levers
    
    returns: [alpha_x, alpha_y]: angles in radian
"""
def PolarToAngle (Rphi, base):
    x_left = base["posx_x"]
    y_left = base["posx_y"]
    r_left = base["posx_r"]
    x_right = base["posy_x"]
    y_right = base["posy_y"]
    r_right = base["posy_r"]
    roller_r = base["roller_r"]
    
    R = Rphi[0] - roller_r
    G = Rphi[1]+np.pi/2
    x_input = R * np.cos(G)
    y_input = R * np.sin(G)

    # Coefficient vector for quadratic equation to get parameter for left instance
    coeff_left = [pow(x_input, 2) + pow(y_input, 2),
                  -2 * y_input * (x_input - x_left) + 2 * x_input * (y_input - y_left),
                  pow((x_input - x_left), 2) + pow((y_input - y_left), 2) - pow(r_left, 2)]


    # Coefficient vector for quadratic equation to get parameter for right instance
    coeff_right = [pow(x_input, 2) + pow(y_input, 2),
                   -2 * y_input * (x_input - x_right) + 2 * x_input * (y_input - y_right),
                   pow((x_input - x_right), 2) + pow((y_input - y_right), 2) - pow(r_right, 2)]

    solution_left = np.roots(coeff_left)
    solution_right = np.roots(coeff_right)

    if sum(np.iscomplex(solution_right)) or sum(np.iscomplex(solution_left)):
        print("##### ERROR: There is no real solution for the given input #####")
        return False

    intersect_left = [[-y_input * solution_left[0] - x_left + x_input, 
                       x_input * solution_left[0] - y_left + y_input],
                      [-y_input * solution_left[1] - x_left + x_input, 
                       x_input * solution_left[1] - y_left + y_input]]

    intersect_right = [[-y_input * solution_right[0] - x_right + x_input,
                        x_input * solution_right[0] - y_right + y_input],
                       [-y_input * solution_right[1] - x_right + x_input,
                        x_input * solution_right[1] - y_right + y_input]]

    angles_left = [np.arctan2(h[1], h[0]) for h in intersect_left]
    angles_right = [np.arctan2(h[1], -h[0]) for h in intersect_right]
    angle_left = min(angles_left, key=abs)
    angle_right = min(angles_right, key=abs)

    if np.abs(angle_left) > np.pi/2 and np.abs(angle_right) > np.pi/2:
        print("##### ERROR: Angle out of range #####")
        return False
    
    return [angle_left, angle_right]

"""
MachineToPolar(): 
    Transforms a pair of machine coordinates for POSX and POSY axis to 
    a polar representation with radius and angle. 
    
    alpha = [alpha_x, alpha_y]: alpha_x and alpha_y are lever angles in radian
    base: dictionary with origins and radii of both positioning levers
    
    returns: [R, phi]: R in mm, phi in radian
"""
def AngleToPolar(angles, base):
    
    alpha_l = angles[0] # angle for posx axis, in degrees
    alpha_r = angles[1] # angle for posy axis, in degrees
    U_l = np.array([base["posx_x"], base["posx_y"]]) # pivot point of posx axis
    U_r = np.array([base["posy_x"], base["posy_y"]]) # pivot point of posy axis
    r_l = base["posx_r"] # radius of posx lever
    r_r = base["posy_r"] # radius of posy lever
    roller_r = base["roller_r"]
    
    # intersection points
    S_l = U_l + np.array([r_l*np.cos(alpha_l), r_l*np.sin(alpha_l)])
    S_r = U_r + np.array([-r_r*np.cos(alpha_r), r_r*np.sin(alpha_r)])
    
    # line 
    k_t = S_l - S_r
    k_r = np.array([-k_t[1], k_t[0]])
    
    A = np.matrix([list(k_r), list(k_t)])
    t = np.linalg.solve(A, S_l)
    
    P = k_r * t[0]
    
    R = np.sqrt(np.power(P[0], 2) + np.power(P[1], 2))
    phi = np.arctan2(P[1], P[0]) - np.pi/2
    R += roller_r
    
    return [R, phi]

R_in = 40
phi_in = 24 /180*np.pi

print("Input parameters: R = %5.2f mm phi = %5.2f degrees" %(R_in, phi_in*180/np.pi))

res1 = PolarToAngle([R_in,phi_in], base)

if res1 != False:
    print("Polar to angle: alpha_x = %5.2f, alpha_y = %5.2f" % (res1[0]/np.pi*180, res1[1]/np.pi*180))
    res2 = AngleToPolar(res1, base)
    print("Angle to Polar: R = %5.2f mm, phi = %5.2f degrees" %(res2[0], res2[1]/np.pi*180))
