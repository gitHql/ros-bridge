from carla_ackermann_msgs.msg import EgoVehicleControlInfo  

import matplotlib
# matplotlib.use('module://mplopengl.backend_qtgl')
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (16,9)
plt.rcParams['figure.dpi'] = 100


def plot_pid_imureal(pedal_history, pid_val, real_val,  \
    output_throttles, brake_upper_borders):
    plt.ion()  

    a , = plt.plot(pid_val, label='pid want acc')

    b, = plt.plot(real_val,  linewidth=2, label='real_acc')

    low = [val - (0.1 if val > 0 else 0.2 )for val in pid_val]
    a , = plt.plot(low,  linestyle='-.', label='pid_low')

    high =  [val + (0.1 if val > 0 else 0.2) for val in pid_val]
    a , = plt.plot(high,  linestyle='-.',label='pid_up')

    c, = plt.plot(pedal_history, linestyle='--',  label = 'target pedal')
    d, = plt.plot(output_throttles, linestyle='--',  label = 'output_throttles')
    d, = plt.plot(brake_upper_borders, label = 'brake_upper_borders')
    

    zero =  [0 for val in pid_val]
    a , = plt.plot(zero,  linestyle='dotted', marker='', label='zero')

    leg = plt.legend(loc='upper right', prop={'size': 6})
    # leg = plt.legend(loc='lower right', prop={'size': 6})

    plt.show()
    plt.pause(0.01)
    plt.clf()  #清除图像

class LogicalStatus:
    # from this velocity on it is allowed to switch to reverse gear
    standing_still_epsilon = 0.1
    # from this velocity on hand brake is turned on
    full_stop_speed_epsilon = 3/3.6
    change_target_delay_count_needed = 100

    def __init__(self):
        self.clean_plot()
        self.inited_when_restart = False

        #target_keep_the_same
        self.thesame_time = 0
        self.target_changed_counting = self.change_target_delay_count_needed
        self.cold_counter = 0
        self.in_cold_starting = True
    
        self.max_delta_pedal = 0.0
        self.min_target_pedal = 0.0

    def init_info(self, info, sec):
        # target values
        info.target.steering_angle = 0.
        info.target.speed = 0.
        info.target.speed_abs = 0.
        info.target.accel = 0.
        info.target.jerk = 0.

        # current values
        info.current.time_sec = sec
        info.current.speed = 0.
        info.current.speed_abs = 0.
        info.current.accel = 0.

        # control values
        info.status.status = 'n/a'
        info.status.speed_control_activation_count = 0
        info.status.speed_control_accel_delta = 0.
        info.status.speed_control_accel_target = 0.
        info.status.accel_control_pedal_delta = 0.
        info.status.accel_control_pedal_target = 0.
        info.status.brake_upper_border = 0.
        info.status.throttle_lower_border = 0.

        # control output
        info.output.throttle = 0.
        info.output.brake = 1.0
        info.output.steer = 0.
        info.output.reverse = False
        info.output.hand_brake = True

    def clean_plot(self):
        self.all_pid_accel = []
        self.all_imu_accer = []
        self.pedal_history = []
        self.brake_upper_borders = []
        self.throttle_lower_borders = []

    def append_info(self, info):
        self.pedal_history.append(info.status.accel_control_pedal_target)
        self.brake_upper_borders .append(info.status.brake_upper_border )
        self.throttle_lower_borders.append(info.output.throttle )

    def make_plt(self, info:EgoVehicleControlInfo,  control_loop_rate:int, accel_controller):
        _, = plt.plot([], label='accel_control_pedal_target.{}'.format(
            round(info.status.accel_control_pedal_target, 3)))

        self.min_target_pedal = min(self.min_target_pedal, 
        info.status.accel_control_pedal_target)
        _, = plt.plot([], label='logical_status.min_target_pedal.{}'.format(
            round(self.min_target_pedal, 3)))

        _,  plt.plot([], label='accel_control_pedal_target.{}'.format(
            round(info.status.accel_control_pedal_target, 3)))
        _ , = plt.plot([], label= 'throttle_lower_border={}'.format(
            round(info.status.throttle_lower_border, 3)))
        _ , = plt.plot([], label='brake_upper_border={}'.format(
            round(info.status.brake_upper_border, 3)))

        _ , = plt.plot([], label='setpoint={}'.format(round(accel_controller.setpoint , 3)))
        _ , = plt.plot([], label=' current.accel={}'.format(round(info.current.accel, 3)))
        _ , = plt.plot([], label='accel_control_pedal_delta={}'.format(
            round(info.status.accel_control_pedal_delta, 5) ))
        
        self.max_delta_pedal = max(self.max_delta_pedal, 
        round(info.status.accel_control_pedal_delta, 3))
        _ , = plt.plot([], label='max accel_control_pedal_delta={}'.format(self.max_delta_pedal) )

        _ , = plt.plot([], label='max_pedel={}'.format(round(info.restrictions.max_pedal, 3)))
        _, = plt.plot([], label='status={}'.format(info.status.status))
        _, = plt.plot([], label='throttle={}'.format(info.output.throttle))

        plot_pid_imureal(self.pedal_history, self.all_pid_accel,
         self.all_imu_accer, self.throttle_lower_borders, self.brake_upper_borders)
        
        imu_hz = 1/ control_loop_rate
        if len(self.all_pid_accel) >= self.clean_length(imu_hz): # 1/self.control_loop_rate :
            self.clean_plot()

    def clean_length(self, imu_hz):
        return 20 * imu_hz