import matplotlib
matplotlib.use('module://mplopengl.backend_qtgl')

import matplotlib.pyplot as plt


import numpy as np

def clean_length(imu_hz):
    return 200 * imu_hz


def plot_pid_imureal(pedal_history, pid_val, real_val, output_throttles):
    plt.ion()  

    high =  [val + (0.1 if val > 0 else 0.2) for val in pid_val]
    a , = plt.plot(high, label='pid_up')

    a , = plt.plot(pid_val, label='pid want acc')

    low = [val - (0.1 if val > 0 else 0.2 )for val in pid_val]
    a , = plt.plot(low, label='pid_low')

    b, = plt.plot(real_val, label='real_acc')
    c, = plt.plot(pedal_history, label = 'target pedal')
    d, = plt.plot(output_throttles, label = 'output_throttles')

    leg = plt.legend(loc='upper right', prop={'size': 6})
    # leg = plt.legend(loc='lower right', prop={'size': 6})


    # length = len(pid_val)
    # my_x_ticks = np.arange(0, length, 1 if length < 100 else 5)
    # my_y_ticks = np.arange(-2, 5, 0.1)
    # plt.xticks(my_x_ticks)
    # plt.yticks(my_y_ticks)

    plt.show()
    plt.pause(0.01)
    plt.clf()  #清除图像

count = 0