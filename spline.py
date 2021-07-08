#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  8 18:15:42 2021

@author: josef
"""
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import solve
from scipy.interpolate import interp1d

input_times = np.array([0, 300, 800, 200, 1300, 1500, 100])
input_positions = np.array([0, 300, 700, 1300, 600, 2000, 1700])


#x = input_times[0:4]
#y = input_positions[0:4]
x = np.array([1, 2, 3])
y = np.array([1, 2, 1])
alpha = 0
omega = 0

# f2 = interp1d(x, y, kind='cubic')

xnew = np.linspace(min(x), max(x), num=41, endpoint=True)

def custom_fit(x, y, omega, alpha):
    """x : three times
       y : three thetas
       omega : initial velocity
       alpha : inital acceleration"""
    # definition of the coefficient matrix
    # helper polynomials
    p = np.array([1, 1, 1, 1])
    dp = np.array([0, 3, 2, 1])
    ddp = np.array([0, 0, 6, 2])
    zer = np.zeros((1, 4))
    # powers of x
    x_p = x[:, np.newaxis]**np.arange(3, -1, -1)
    # evaluation of polynomials
    p_x = p * x_p
    dp_x = dp * x_p[:2, :]
    ddp_x = ddp * x_p[:2, :]
    # pad the helper polynomials
    dp_x = np.roll(dp_x, -1, axis=1)
    ddp_x = np.roll(ddp_x, -2, axis=1)
    # set up the coefficient matrix
    A = np.block([
        [p_x[0, :],     zer],
        [p_x[1, :],     zer],
        [zer,           p_x[1, :]],
        [zer,           p_x[2, :]],
        [dp_x[0, :],    zer],
        [ddp_x[0, :],   zer],
        [dp_x[1, :], -dp_x[1, :]],
        [ddp_x[1, :], -ddp_x[1, :]]])
    # set up the inhomogenity vector
    b = np.hstack((y[:2], y[1:], omega , alpha, 0, 0))
    # solve the equation system
    return solve(A, b)


def eval_spline(coef, x_bound, x):
    """coef : coefficient vector
        x_bound : stuetzstellen
        x   : points of evaluation"""
    # decompose the coefficient vector
    p_1 = coef[:4]
    p_2 = coef[4:]
    # determine section
    in_interval_1 = (x >= x_bound[0]) & (x < x_bound[1])
    in_interval_2 = (x >= x_bound[1]) & (x <= x_bound[2])
    # evaluate the spline
    return in_interval_1 * np.polyval(p_1, x) + in_interval_2 * np.polyval(p_2, x)
    
coef = custom_fit(x, y, omega, alpha)
ynew = eval_spline(coef, x, xnew)

plt.plot(x, y, 'o', xnew, ynew, '-')
plt.legend(['data', 'cubic'], loc='best')
plt.grid()
plt.show()

plt.plot(xnew[1:], np.diff(ynew))#
plt.grid()
plt.show()

plt.plot(xnew[1:-1], np.diff(np.diff(ynew)))
plt.grid()
plt.show()
