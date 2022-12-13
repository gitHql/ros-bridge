import matplotlib
matplotlib.use('module://mplopengl.backend_qtgl')

import matplotlib.pyplot as plt


import numpy as np

def clean_length(imu_hz):
    return 200 * imu_hz


def plot_pid_imureal(pedal_history, pid_val, real_val, output_throttles, brake_upper_borders):
    plt.ion()  

    a , = plt.plot(pid_val, label='pid want acc')

    b, = plt.plot(real_val,  linewidth=2, label='real_acc')

    low = [val - (0.1 if val > 0 else 0.2 )for val in pid_val]
    a , = plt.plot(low, label='pid_low')

    high =  [val + (0.1 if val > 0 else 0.2) for val in pid_val]
    a , = plt.plot(high, label='pid_up')

    c, = plt.plot(pedal_history, label = 'target pedal')
    d, = plt.plot(output_throttles, label = 'output_throttles')

    d, = plt.plot(brake_upper_borders, label = 'brake_upper_borders')
    

    zero =  [0 for val in pid_val]
    a , = plt.plot(zero,  linestyle='dotted', marker='', label='zero')

    leg = plt.legend(loc='upper right', prop={'size': 6})
    # leg = plt.legend(loc='lower right', prop={'size': 6})

    plt.show()
    plt.pause(0.01)
    plt.clf()  #清除图像