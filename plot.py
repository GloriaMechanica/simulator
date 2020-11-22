#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 22 14:44:32 2020
Script to plot the trajectory of the input-point-motion based on changing values
of the angles of POS-X and POS-Y
@author: lks and josef
"""

import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from coordinates import AngleToPolar
from coordinates import PolarToAngle

"""
AnglesToActualVector(alpha, base): 
    Transforms a pair of machine coordinates for POSX and POSY axis to 
    the actual position of the intersection between lever and bow 

    alpha = [alpha_x, alpha_y]: alpha_x and alpha_y are lever angles in radian
    base: dictionary with origins and radii of both positioning levers

    returns: [x_vector, y_vector] in mm
"""


def AnglesToActualVector(alpha, base):
    x_vector = [
        base["posx_x"] + base["posx_r"] * np.cos(alpha[0]),
        base["posx_y"] + base["posx_r"] * np.sin(alpha[0]),
    ]

    y_vector = [
        base["posy_x"] + base["posy_r"] * np.cos(np.pi - alpha[1]),
        base["posy_y"] + base["posy_r"] * np.sin(np.pi - alpha[1]),
    ]

    return [x_vector, y_vector]


"""
PlotBowSim(Rphi_sim, base, alphas): 
    Plots the Bow Movement depending on a values for XPOS and YPOS

    base: dictionary with origins and radii of both positioning levers
    alphas = [alpha_x_t, alpha_y_t]: alpha_x_t and alpha_y_t are lever angles depending on t
"""


def PlotBowSim(alphas, base):
    # Generate the two levers, these are static objets
    lever_x = plt.Circle((base["posx_x"], base["posx_y"]), base["posx_r"], fill=False, color="black")
    lever_y = plt.Circle((base["posy_x"], base["posy_y"]), base["posy_r"], fill=False, color="black")

    lineswidth = 0.5
    # Set some parameters for the plot
    fig, ax = plt.subplots()
    plt.xlim(-100, 100)
    plt.ylim(-10, 100)
    plt.grid(linestyle='--')
    ax.set_aspect(1)
    # Draw the two levers
    ax.add_artist(lever_x)
    ax.add_artist(lever_y)

    # Draw all positions based on every value of alpha
    for alpha in alphas:
        # get the absolute lever-vectors
        positions = AnglesToActualVector(alpha, base)
        Rphi = AngleToPolar(alpha, base)
        
        # offset for bow hair by roller radius
        ros_x = base["roller_r"]*np.cos(Rphi[1]+np.pi/2)
        ros_y = base["roller_r"]*np.sin(Rphi[1]+np.pi/2)
        
        roller_left = plt.Circle((positions[0][0], positions[0][1]), base["roller_r"], fill=False, color="black")
        roller_right = plt.Circle((positions[1][0], positions[1][1]), base["roller_r"], fill=False, color="black")
        ax.add_artist(roller_left)
        ax.add_artist(roller_right)
    
        # Draw the bow hair
        plt.plot((positions[0][0]+ros_x, positions[1][0]+ros_x), (positions[0][1]+ros_y, positions[1][1]+ros_y), color="blue", zorder=1, linewidth=lineswidth)

        # Draw left lever
        plt.plot((base["posx_x"], positions[0][0]), ([base["posx_y"], positions[0][1]]), color="black", linewidth=lineswidth)
        # Draw right lever
        plt.plot((base["posy_x"], positions[1][0]), ([base["posy_y"], positions[1][1]]), color="black", linewidth=lineswidth)

        # Draw resulting position
        plt.plot((0, Rphi[0] * np.cos(Rphi[1] + np.pi / 2)), (0, Rphi[0] * np.sin(Rphi[1] + np.pi / 2)), color="green",
                 zorder=-1, linewidth=lineswidth)

    # Generate the sections of the strings, these are static objets
    # Make string radius adjustable for easier testing
    string_diameter = 1
    string_radius = 33
    e_angle = (-27+90)/180*np.pi
    a_angle = (-9+90)/180*np.pi
    d_angle = (9+90)/180*np.pi
    g_angle = (27+90)/180*np.pi
    string_e = plt.Circle((string_radius*np.cos(e_angle), string_radius*np.sin(e_angle)), string_diameter / 2, fill=True, color="red", zorder=1)
    string_a = plt.Circle((string_radius*np.cos(a_angle), string_radius*np.sin(a_angle)), string_diameter / 2, fill=True, color="red", zorder=1)
    string_d = plt.Circle((string_radius*np.cos(d_angle), string_radius*np.sin(d_angle)), string_diameter / 2, fill=True, color="red", zorder=1)
    string_g = plt.Circle((string_radius*np.cos(g_angle), string_radius*np.sin(g_angle)), string_diameter / 2, fill=True, color="red", zorder=1)

    # Draw the strings
    ax.add_artist(string_e)
    ax.add_artist(string_a)
    ax.add_artist(string_d)
    ax.add_artist(string_g)
    fig.savefig("movement.pdf", bbox_inches="tight")

    plt.title('Simulation of Bow Position', fontsize=12)
    plt.show()


"""
alphaMotionProto(Rphi_sim): 
    Function prototype to simulate alpha movement XPOS and YPOS

   Rphi_sim: Dict containing start and end position for R and phi of the bow
"""


def alphaMotionProto(Rphi_sim, base):
    samples = 10

    alpha_start = PolarToAngle((Rphi_sim["R_start"], Rphi_sim["phi_start"]), base)
    alpha_end = PolarToAngle((Rphi_sim["R_end"], Rphi_sim["phi_end"]), base)

    alphas_left = np.linspace(alpha_start[0], alpha_end[0], samples)
    alphas_right = np.linspace(alpha_start[1], alpha_end[1], samples)

    alphas = []
    for i in range(samples):
        alphas.append([alphas_left[i], alphas_right[i]])

    return alphas


# TESTING STUFF

G = -27
D = -9
A = 9
E = 27

start_string = E
end_string = A
string_radius = 38 - 5 # needs to be 5mm lower becaus of bow rollers

base = {"posx_x": -60, "posx_y": 25, "posx_r": 25, "posy_x": 60, "posy_y": 50, "posy_r": 35, "roller_r":5}
Rphi_sim = {"R_start": string_radius, "phi_start": start_string * np.pi / 180, "R_end": string_radius, "phi_end": end_string * np.pi / 180}

alphas = alphaMotionProto(Rphi_sim, base)

PlotBowSim(alphas, base)
