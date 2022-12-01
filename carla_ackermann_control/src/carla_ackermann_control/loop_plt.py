import matplotlib.pyplot as plt


def plot_pid_imureal(pedal_history, pid_val, real_val, throttle_lower_borders):
    plt.ion()  
    a , = plt.plot(pid_val, label='pid')
    b, = plt.plot(real_val, label='real')
    # c, = plt.plot(pedal_history, label = 'target pedal')
    # d, = plt.plot(throttle_lower_borders, label = 'throttle')

    leg = plt.legend(loc='upper left')

    plt.show()
    plt.pause(0.02)
    plt.clf()  #清除图像

count = 0