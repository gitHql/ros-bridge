import matplotlib.pyplot as plt


def plot_pid_imureal(pedal_history, pid_val, real_val, throttle_lower_borders):
    plt.ion()  

    high =  [val + 0.1 for val in pid_val]
    a , = plt.plot(high, label='pid_up')

    a , = plt.plot(pid_val, label='pid')

    low = [val - 0.1 for val in pid_val]
    a , = plt.plot(low, label='pid_low')

    b, = plt.plot(real_val, label='real')
    # c, = plt.plot(pedal_history, label = 'target pedal')
    # d, = plt.plot(throttle_lower_borders, label = 'throttle')

    leg = plt.legend(loc='lower right', prop={'size': 6})

    plt.show()
    plt.pause(0.02)
    plt.clf()  #清除图像

count = 0