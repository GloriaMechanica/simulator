#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 17 14:53:02 2020

This is basically a simulator of the SPV motor controller. 
It can be used to predict wheter a certain movement can 
be executed or not, for debugging purposes. 

@author: josef
"""

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Gemeral settings
debug_compare = 0
start_index_debug = 0
catchup = 0
plot_pics = 1

# general parameters
N_APPROX = 4 # number of w_m calculations to approximate optimal passover speed
R_ERR = 1e-6
W_ERR = 100
F_TIMER = 8000e3 # timer frequency
FACTOR = 1000
S_CORR = 2
C_TABLE_SIZE = 1200

# motor parameters (might be different per axis)
steps_per_rev = 400 #degrees
step_mode = 4 # 1=full step, 2=half step, 4=quater step ...
acc = 600 # maximal acceleration in rad/s
speed_max = 50 # maximal speed in rad/s
alpha = 2 * np.pi / steps_per_rev / step_mode


if debug_compare==1:
    file_name = "dump.log"
    raw_data = np.fromfile(file_name, dtype='<u2')
    
    cycle_dumps = []
    cycle_data = []
    
    i=0
    while (i < (len(raw_data)-6)):
        if raw_data[i]==0 and raw_data[i+1]==0 and raw_data[i+2]==0 and raw_data[i+3]==0:
            #this is a start of a cycle
            j = i+4
            delta_s = raw_data[j]
            j+=1
            delta_t = raw_data[j]
            j+=1
            cycle_data = []
            while(not(raw_data[j]==0 and raw_data[j+1]==0 and raw_data[j+2]==0 and raw_data[j+3]==0) and j < (len(raw_data)-4)):
                cycle_data.append(raw_data[j])
                j+=1
            i = j-1
            cycle_dumps.append([delta_s, delta_t, cycle_data])
        i+=1

motor1 = [acc, alpha]
interesting_diff = []

test_pos =  [0, 	1000, 	2000, 	3100, 	500, 	10000, 	0, 		0, 		0]
test_time =  [0, 	300, 	700, 	1200, 	1800, 	2800, 	3800, 	4000, 	4500]

#test_pos =  [0, 	800, 	1100, 	2000, 	500, 	1000, 	0, 		0, 		0]
#test_time =  [0, 	300, 	700, 	1200, 	1800, 	2800, 	3000, 	4000, 	4500]

"""
Input to function should be: 
    cyclespec: 
        delta_s0
        delta_s1
        delta_t0
        delta_t1
        w_s
    motor: 
        acc (maximum acceleration)
        alpha
"""
def motor_calculations(cyclespec, motor, cycle_index):
    # movements for two subsequent blocks
    movement_s0 = cyclespec[0] #steps
    movement_s1 = cyclespec[1]  #steps
    time_0 = cyclespec[2]  #ms
    time_1 = cyclespec[3]  #ms
    w_s = cyclespec[4]; # start speed 
    runs = N_APPROX
    
    # calculation of static

    delta_s0 = movement_s0
    delta_s1 = movement_s1
    delta_t0 = time_0 / 1e3
    delta_t1 = time_1 / 1e3
    
    
    w_m = [0] * N_APPROX
    w_t0 = [0] * N_APPROX
    w_t1 = [0] * N_APPROX
    d_s = [0] * N_APPROX
    d_m = [0] * N_APPROX
    d_e = [0] * N_APPROX
    w_diff = [0] * N_APPROX
    
    
    # Are inputs valid?
    if (delta_t0 <= 0 or delta_t1 <= 0):
        print("Error: negative times not possible")
        return [1]
        
    # Is this a zero-cycle
    if (delta_s0 == 0):
        print("Detected zero cycle. No need for further calculations")
        return [1]
    
    print("############### start cycle "+str(cycle_index)+" calculations ##################")

    
    #does the niext cycle go in the other direction as this cycle?
    if (delta_s0 * delta_s1 < 0):
    	# If it does so, passover speed needs to be 0.
    	# This is achived by setting delta_s1 to 0. The algorithm itself chooses w_m_f to be 0, which is correct.
    	# The fact that we wrote 0 to delta_s1 is not relevant, because when it comes to this cycle, it will be reloaded
    	# as delta_s0 and executed correctly
    	delta_s1 = 0;  
        
    # Is the direction of this cycle backwards?
    if (delta_s0 < 0):
    	# If so, the sign bits both of delta_s0 and delta_s1 need to be flipped, but the hardware direction is also flipped
        # If the signs are different, delta_s1 has already been set to 0, so this does not matter.
    	delta_s0 = -delta_s0
    	delta_s1 = -delta_s1
    	dir_abs = -1
    else:
    	dir_abs = 1
        
    print("delta_s0: "+str(delta_s0))
    print("delta_t0: "+str(delta_t0*1e3) + " ms")        
    print("delta_s1: "+str(delta_s1))
    print("delta_t1: "+str(delta_t1*1e3) + " ms")
    print("w_s: " + str(w_s))
     
    # All calculations of speeds etc. are done in radiant. So we need to convert steps to radiant
    delta_theta0 = delta_s0 * alpha
    delta_theta1 = delta_s1 * alpha
    
    # Mean speeds of cycles are base for some decisions
    w_mean0 =  delta_theta0 / delta_t0
    w_mean1 =  delta_theta1 / delta_t1
    
    # End speed of future cycle is set to its mean speed, because it can always do that
    # Other speeds might not even be possible if only very few steps are available
    w_e = w_mean1 
    w_mean2 = w_e # only used internally for simulation 
        
    # Calculate range and stepsize for w_m iteration so that it fills N_approx steps between w_mean0 and w_mean1
    w_base = min(w_mean0, w_mean1)
    w_stepsize = (max(w_mean0, w_mean1) - w_base) / (N_APPROX - 1)
    w_m[0] = w_base;
    
    # equivalent acceleration indices are needed to decide whether this cycle is "slow" or not.
    # an equivalent acceleration index is the number of steps into one cycle until which
    # or from which on one needs to accelerate/decelerate
    neq_mean0 = int(np.floor(w_mean0 * w_mean0 / (2 * alpha * acc)))
    neq_mean1 = int(np.floor(w_mean1 * w_mean1 / (2 * alpha * acc)))
    
    # Decide if cycles are "slow" and therefore do not need any acceleration ramps
    # It is important that first neq_mean1 is checked and then possibly overwritten by neq_mean0
    slow0 = 0
    slow1 = 0
    if (neq_mean1 == 0): # the upcoming cycle 
        w_m[0] = w_mean1 # w_e is already w_mean1, which it is always. (because we cannot look in the future forever, so its a good compromise)
        slow1 = 1
        runs = 1 # calculate only one run 
    
    
    # this looks crude, what if the motor is turning faster and will be forced slow here?
    # But this never happens because w_m was already chosen right the cycle before,
    # so this statement is not even really necessary here, but it can't hurt to make sure.
    if (neq_mean0 == 0):
        w_s = w_mean0 	
        w_m[0] = w_mean0
        slow0 = 1 		# Mark flag
        runs = 1
    
    # calculate the whole cycle including
    #   o  up/down decisions
    #   o  all speeds(targetspeed, ...)
    # for each of the proposed passover speeds
    for i in range(runs):
        if (runs > 1):
            # neither of the cycles is slow and more than one iteration shall be done
            w_m[i] = w_stepsize * i + w_base
    
        # chose right direction for start acceleration
        if (w_s > w_mean0):
            dw_s = -acc
            d_s[i] = -1
            #print("Start down")
        else:
            dw_s = acc
            d_s[i] = 1
            #print("Start up");
    
    	# chose right direction for mid acceleration (passover)
        if (w_mean0 > w_mean1):
            dw_m = -acc
            d_m[i] = -1
           # print("Middle down")
        else:
            dw_m = acc
            d_m[i] = 1
            #print("Middle up");
            
        if (w_mean1 > w_mean2):
            dw_e = -acc
            d_e[i] = -1
            #print("End down"); 
        else:
            dw_e = acc 
            d_e[i] = 1 
            #print("End up") 
    
    	# Polynomial coefficients for target speed calculation
        a0 = 1/(2*dw_m) - 1/(2*dw_s)
        b0 = delta_t0 + w_s/dw_s - w_m[i]/dw_m
        c0 = w_m[i]*w_m[i]/(2*dw_m) - w_s*w_s/(2*dw_s) - delta_theta0
    
        a1 = 1/(2*dw_e) - 1/(2*dw_m)
        b1 = delta_t1 + w_m[i]/dw_m - w_e/dw_e
        c1 = w_e*w_e/(2*dw_e) - w_m[i]*w_m[i]/(2*dw_m) - delta_theta1
    
    	# Calculate target speeds for this w_m[i]
    	# Target speed 0
        if (-R_ERR < a0 and a0 < R_ERR ): # Means a0 == 0
            if (-R_ERR < b0 and b0 < R_ERR): # Means b0 == 0
    			# Speed cannot be calculated
                print("ERROR: Linear 0 (loop "+ str(i) + ")")
                return [1]
            else:
                #print("Linear in cycle 0 ok")
                w_t0[i] = -c0/b0
        else:
            disk0 = b0*b0-4*a0*c0
                
            if (disk0 > 0):
                #print("Root in cycle 0 ok")
                w_t0[i] = (-b0 + np.sqrt(disk0))/(2*a0)
            else:
                print("ERROR: Root 0 (loop " + str(i) + ")")
                return [1]
    
        # Target speed 1
        if (-R_ERR < a1 and a1 < R_ERR ): # Means a0 == 0
            if (-R_ERR < b1 and b1 < R_ERR): # Means b0 == 0
    			# Speed cannot be calculated
                print("ERROR: Linear 1 (loop "+ str(i) +")")
                return [1]
            else:
               #print("Linear 1 ok")
                w_t1[i] = -c1/b1
        else:
            disk1 = b1*b1-4*a1*c1
            if (disk1 > 0):
                #print("Root 1 ok")
                w_t1[i] = (-b1 + np.sqrt(disk1))/(2*a1)
            else:
                print("ERROR: Root 1 (loop " + str(i) + ")")
                return [1]
    
        # Difference between target speed is the thing to be minimized
        w_diff[i] = abs(w_t0[i] - w_t1[i]);
        
    # And now choose the best option
    w_min = W_ERR
    w_m_f = W_ERR
    w_t0_f = W_ERR
    w_t1_f = W_ERR
    d_s_f = 0 # Initialize, just so the warning goes away
    
    for i in range(runs):
        if (((w_t0[i] >= 0) and (w_t0[i] < speed_max) and (w_t1[i] >= 0) and (w_t1[i] < speed_max) and (w_diff[i] < w_min)) or slow0):
            # Its better than the previous, so we take it
            w_min = w_diff[i]
            w_m_f = w_m[i]
            w_t0_f = w_t0[i]
            w_t1_f = w_t1[i]
            d_s_f = d_s[i]
            d_m_f = d_m[i]
            d_e_f = d_e[i] # we acutually do not need the end direction, because no one cares about it
    
    # Correction of acceleration, if it changed during calculation. This can happen sometimes, and dw_m needs to be updated.
    # It should not be neccessary to update d_m_f here, but by rewriting it again we assure that they always match in direction
    if (w_t0_f > w_t1_f):
    	dw_m = -acc
    	d_m_f = -1
    else:
    	dw_m = acc
    	d_m_f = 1
    
    # In some cases, laying the target speed under w_t0 and w_t1 and not between them actually gives
    # a faster acceleration (don't ask me why, but the equations show it), but we no not want that, so
    # we use the smallest possible in the range between w_t0 and w_t1, which is w_t0  (w_t1 is always
    # "further away", because we accelerate towards it).
    if ((w_t0_f - w_m_f)*d_m_f > 0):
    	w_m_f = w_t0_f
    
    if (w_t0_f < speed_max):
    	print("Found ideal w_m = "+str(w_m_f)+", w_t0 = "+str(w_t0_f)+", w_t1 = "+str(w_t1_f));
    else:
    	print("ERROR: Found nothing possible! (w_m = "+str(w_m_f)+")");
    	return [1]
        
    # Post processing all values for ISR
    s_on = int((w_t0_f*w_t0_f - w_s*w_s)/(2*alpha*dw_s)) + S_CORR
    s_off = int(delta_s0 - (w_m_f*w_m_f - w_t0_f*w_t0_f)/(2*alpha*dw_m))
    neq_on = int(w_s*w_s/(2*alpha*dw_s))
    neq_off = int(w_t0_f*w_t0_f/(2*alpha*dw_m)) 
    c_t = int(alpha/(w_t0_f / F_TIMER)*FACTOR)
    
    if (slow0 == 1):
        neq_on = 0; 
        neq_off = 0;
        s_on = 0; 
        s_off = delta_s0; 
        
    s_on1 = int((w_t1_f*w_t1_f - w_m_f*w_m_f)/(2*alpha*dw_m)) + S_CORR
    s_off1 = int(delta_s1 - (w_e*w_e - w_t1_f*w_t1_f)/(2*alpha*dw_e)) 
    neq_on1 = int(w_m_f*w_m_f/(2*alpha*dw_m))
    neq_off1 = int(w_t1_f*w_t1_f/(2*alpha*dw_e)) 
    
    if (w_t1_f == 0):
        c_t1 = 1.1*delta_t1*F_TIMER
    else:     
        c_t1 = int(alpha/(w_t1_f / F_TIMER)*FACTOR)
        
    if (slow0 == 1):
    	neq_on = 0
    	neq_off = 0
    	s_on = 0
    	s_off = delta_s0
        
    if (slow1 == 1):
        neq_on1 = 0 
        neq_off1 = 0
        s_on1 = delta_s0
        s_off1 = delta_s0 + delta_s1       
    
    # Fill the whole table with taylor approximation preload values
    c_table = np.zeros(C_TABLE_SIZE) 
    for i in range(C_TABLE_SIZE):
        c_table[i] =  int(F_TIMER*np.sqrt(2*alpha/acc)*FACTOR * (np.sqrt(i+1) - np.sqrt(i)))        
    
    print("s_on: "+str(s_on)+" s_off: "+str(s_off))
    print("neq_on: "+str(neq_on)+" neq_off: "+str(neq_off))
    print("c_0: "+str(c_table[0]) + " c_t: "+str(c_t)+ " c_t without factor: "+str(int(c_t/1e3)))
    print("d_on: "+str(d_s_f)+" d_off: "+str(d_m_f))
    print("dir_abs: "+str(dir_abs)+" slow: "+str(slow0))


    
    print("----------- simulate calculated cycle -----------------")
     

    # just for debug
    w_typ = alpha/(c_table/F_TIMER/FACTOR);
    
    # length of the simulation, in steps
    length = delta_s0 + delta_s1; 
    
    # This C would be here from the last cycle
    #c = alpha/w_s*F_TIMER*FACTOR;
    
    # Simulating stepper action test variables
    c_total = 0;
    c_total0 = 0;
    overshoot_0 = 0; 
    overshoot_1 = 0;
    w_m_real = 0;
    w_real = [0] * length; 
    tvec = [0] * length;
    c_hw_real = [0] * length;
    
    c_debug_acc = 0
    c_debug_dec = 0
    c_debug_target = 0
    
    c_temp = 0; 
    s = 0
    
    while s < length:
        # ACCEL 0  
        if s < delta_s0:
            if (s < s_on):
                if (s == 0):
                    n = neq_on                
    
                if (slow0 == 1):
                    # slow target speed (without accel)
                    c_temp = c_t 
                else:
                    c_temp = c_table[int(abs(n))]                   
                
                n = n + 1
                
                # Overshoot protection
                if (int((c_temp - c_t)*d_s_f) >= 0):
                    c = c_temp
                else:
                    c = c_t # go along with c_t instead of overshooting
                    overshoot_0 = overshoot_0 + 1                
                                           
            # DECEL 0  
            elif (s >= s_off):
               # end accel period
                if (s == s_off):
                    n = neq_off 
    
                if (slow0 == 1):
                    # slow target speed (without accel)
                    c_temp = c_t                  
                else:
                    # standard start acceleration
                    c_temp = c_table[int(abs(n))]
                
                n = n + 1
                
                # overshoot protection
                if (int((c_t - c_temp)*d_m_f) >= 0):
                    c = c_temp
                else:
                    c = c_t
                    overshoot_0 = overshoot_0 + 1         
            else:
               c = c_t
        else:
            # ---------- 1 ---------------
            # ACCEL 1        
            if (s < s_on1 + delta_s0):
                if (s == delta_s0):
                    n = neq_on1              
                
                if (slow1 == 1):
                    # slow target speed (without accel)
                    c_temp = c_t1 
                else:
                    c_temp = c_table[int(abs(n))];                     
    
                
                n = n + 1 
                
                # Overshoot protection
                if (int((c_temp - c_t1)*d_m_f) >= 0):
                    c = c_temp; 
                else:
                    c = c_t1 
                    overshoot_1 = overshoot_1 + 1; 
                                           
            # DECEL 1  
            elif (s > s_off1 + delta_s0):
                # end accel period
                if (s == s_off1 + delta_s0):
                    n = neq_off
    
                if (slow1 == 1):
                    # slow target speed (without accel)
                    c_temp = c_t1                   
                else:
                    # standard start acceleration
                    c_temp = c_table[int(abs(n))]
                
                n = n + 1 
                
                # overshoot protection
                if (int((c_t1 - c_temp)*d_e_f) >= 0):
                    c = c_temp
                else:
                    c = c_t1
                    overshoot_1 = overshoot_1 + 1 
            
            else: 
               c = c_t1

            if (c < 0):
                print("negative c!")
            
        # now we "waited" until the step is done    
       
        # calculate information about this step cycle for plotting etc.
        c_hw = np.floor(c/FACTOR)
        c_hw_real[s] = c_hw
        c_total = c_total + c_hw    
        
        w_real[s] = alpha/(c_hw/F_TIMER)
      
        if (s < delta_s0):
            c_total0 = c_total0 + c_hw
            w_m_real = alpha/(c_hw/F_TIMER)
            if (s<s_on): 
                c_debug_acc += c_hw
            elif (s>s_off): 
                c_debug_dec += c_hw
            else: 
                c_debug_target += c_hw
    
        tvec[s] = c_total/F_TIMER;  
        s = s + 1;     
    
    t_theo = delta_t0 * 1e3; 
    t_real = c_total0 /F_TIMER * 1e3; 
    t_theo2 = delta_t1 * 1e3; 
    t_real2 = (c_total - c_total0)/F_TIMER  * 1e3
    dw_real = np.diff(w_real)/np.diff(tvec) 
    diff_ticks = c_total0 - delta_t0*F_TIMER
    
    # Calculate for debug purposes how long the acceleration and deceleration phases actually took
    t_acc_theo = abs(w_s - w_t0_f)/acc * 1e3
    t_dec_theo = abs(w_t0_f - w_m_f)/acc * 1e3
    t_target_theo = delta_t0 * 1e3 - t_acc_theo - t_dec_theo
    t_acc_real = c_debug_acc/F_TIMER * 1e3
    t_dec_real = c_debug_dec/F_TIMER * 1e3
    t_target_real = c_debug_target/F_TIMER *1e3
    diff_acc = (t_acc_real-t_acc_theo)
    diff_target = (t_target_real-t_target_theo)
    diff_dec = (t_dec_real-t_dec_theo)
    
    print("First cycle: s_on: " + str(s_on) + " s_off: " + str(s_off)) 
    #print("Second cycle: s_on2: "+str(s_on1)+" s_off2: "+str(s_off1))
    print("Pass-over speed: "+str(w_m_f)+" (theo) "+str(w_m_real)+" (real)") 
    print("Overshoot: cycle 0: "+str(overshoot_0)+", cycle 1: " +str(overshoot_1))
    print("Timing ideal: acc: "+str(t_acc_theo) + " target: "+str(t_target_theo)+" dec: "+str(t_dec_theo))
    print("Timing real : acc: "+str(t_acc_real) + " target: "+str(t_target_real)+" dec: "+str(t_dec_real))
    print("Differences: acc: " + str(diff_acc) + "ms target: " + str(diff_target) + "ms dec: "+str(diff_dec)+"ms")
    print("Timing total 0: " + str(t_theo) + " (ideal) " + str(t_real) + " (real) " + str((t_real-t_theo)) + " ms (diff) " + str((t_real - t_theo)/t_theo * 100 ) + " percent") 
    #print("Timing total 1: "+str(t_theo2)+" (ideal) "+str( t_real2)+" (real) "+str((t_real2-t_theo2))+" ms (diff) "+str((t_real2 - t_theo2)/t_theo2 * 100)+" percent") 
    print("Difference in timer ticks: " + str(diff_ticks) )
    print("Total ticks: " + str(np.sum(np.array(c_hw_real)[0:1000])))
    
    
    
    
    """
    Contents of status vector
    0:  delta_s0
    1:  delta_t0
    2:  w_s0
    3:  w_t0
    4:  w_m0
    5:  t_real0
    6:  c_total0
    7:  t_acc_theo
    8:  t_acc_real
    9:  t_target_theo
    10: t_target_real
    11: t_dec_theo
    12: t_dec_real
    13: diff_acc: difference between theoretical and real acceleration period
    14: diff_target
    15: diff_dec
    16: diff_total: difference between theoretical and real total time
    17: diff_total_percent: difference between theoretical and real total time, in percent
    """
    status_vector = np.array([delta_s0, delta_t0*1e3, w_s, w_t0_f, w_m_f, t_real, c_total0, t_acc_theo, t_acc_real, t_target_theo, t_target_real, t_dec_theo, t_dec_real, diff_acc, diff_target, diff_dec, t_real - t_theo, (t_real - t_theo)/t_theo * 100  ])
       
    print("-------- Finished motor control calculations -------------");
    
    
    if debug_compare==1:
        c_debug = np.array(cycle_dumps[start_index_debug + cycle_index][2])
        t_debug = c_debug/F_TIMER
        w_debug = alpha/t_debug    
        t_debug_vec = np.cumsum(t_debug)
        interesting_diff.append(c_debug-np.array(c_hw_real)[0:delta_s0])

    
    if (plot_pics):
        width = 160
        height = width 
        fig, (ax1, ax2) = plt.subplots(2)
        ax1.plot(tvec, w_real, label="speed sim")
        if debug_compare==1:
            ax1.plot(t_debug_vec, w_debug, label="speed hardware")
        ax2.plot(tvec[:-1], dw_real, label="acceleration")
        #ax1.set_title(r'Frequency sweep')
        ax1.set_xlabel(r' time in s')
        ax1.set_ylabel(r'speed in rad/s')
        ax2.set_ylabel(r'acceleration in rad/s$^2$')
        ax2.set_ylim([-2*acc,2*acc])
        ax2.hlines(acc, min(tvec), max(tvec), color="red", linestyle='--')
        ax2.hlines(-acc, min(tvec), max(tvec), color="red", linestyle='--')
        #ax1.legend()
        ax1.grid(True, linestyle='--')
        ax2.grid(True, linestyle='--')
        ax1.legend()
        fig.set_size_inches(width/25.4, height/25.4)
        fig.savefig("cycle_"+str(cycle_index)+".pdf", format="pdf", bbox_inches="tight")

    return [0, status_vector, c_hw_real]


t_catchup = 0
w_md = 0
diff_time_total = 0
diff_ticks_total = 0
diff_time_vect = []
status_summary = []
for i in range(len(test_pos)-2):
    delta_s0 = test_pos[i+1] - test_pos[i]
    delta_s1 = test_pos[i+2] - test_pos[i+1]
    delta_t0 = test_time[i+1] - test_time[i]
    delta_t1 = test_time[i+2] - test_time[i+1]
    cyclespec = [delta_s0, delta_s1, delta_t0-t_catchup*catchup,  delta_t1, w_md]
    ret = motor_calculations(cyclespec, motor1, i)
    if (ret[0] == 0):
        w_md = ret[1][4]
        diff_time  = ret[1][5] - delta_t0
        diff_time_total += diff_time
        diff_time_vect.append(diff_time)
        diff_ticks_total += ret[1][6] - delta_t0*F_TIMER/1e3
        t_catchup = diff_time_total
        status_summary.append(ret[1])
        print("Cumulated time error: " + str(diff_time_total) + " ms")
        print("Cumulated ticks error: " + str(diff_ticks_total))
    else: 
        break
        print("Impossible movement. Aborted.")
i = 0       

status_summary = np.array(status_summary)
print( "################# Movement summary ######################")
print("Total time error: " + str(diff_time_total) + " ms")
print("Total ticks error: " + str(diff_ticks_total))
for i in range(len(diff_time_vect)):
    print("Cycle "+str(i)+" has "+str(status_summary[i,5]-status_summary[i,1]) +" ms difference " + str((status_summary[i,5]-status_summary[i,1])/status_summary[i,1]*1e2)+"%" )

i = 0
print("\n")
for v in interesting_diff: 
    print("Cycle " + str(i) + " has " + str(np.sum(v)) + " timer ticks difference.")
    i+=1
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    