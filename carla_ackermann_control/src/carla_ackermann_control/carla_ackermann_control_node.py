#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Control Carla ego vehicle by using AckermannDrive messages
"""

import sys, numpy,  rospy
from simple_pid import PID  # pylint: disable=import-error,wrong-import-order

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_ackermann_control import carla_control_physics as phys

from ackermann_msgs.msg import AckermannDrive  # pylint: disable=import-error,wrong-import-order
from demo_msgs.msg import CL_VehicleCommand
from std_msgs.msg import Float32
from std_msgs.msg import Header # pylint: disable=wrong-import-order
from sensor_msgs.msg import Imu

from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=no-name-in-module,import-error
from carla_ackermann_msgs.msg import EgoVehicleControlInfo  # pylint: disable=no-name-in-module,import-error
from logical import LogicalStatus

ROS_VERSION = roscomp.get_ros_version()

if ROS_VERSION == 1:
    # pylint: disable=no-name-in-module,import-error,ungrouped-imports
    from carla_ackermann_control.cfg import EgoVehicleControlParameterConfig 
    from dynamic_reconfigure.server import Server # pylint: disable=no-name-in-module,import-error
if ROS_VERSION == 2:
    from rcl_interfaces.msg import SetParametersResult

class CarlaAckermannControl(CompatibleNode):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        super(CarlaAckermannControl, self).__init__("carla_ackermann_control")

        # PID controller
        # the controller has to run with the simulation time, not with real-time
        #
        # To prevent "float division by zero" within PID controller initialize it with
        # a previous point in time (the error happens because the time doesn't
        # change between initialization and first call, therefore dt is 0)
        sys.modules['simple_pid.PID']._current_time = (       # pylint: disable=protected-access
            lambda: self.get_time() - 0.1)
        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)

        # we might want to use a PID controller to reach the final target speed
        self.speed_controller = PID(Kp=self.get_param("speed_Kp", alternative_value=0.05),
                                    Ki=self.get_param("speed_Ki", alternative_value=0.),
                                    Kd=self.get_param("speed_Kd", alternative_value=0.5),
                                    sample_time=0.05,
                                    output_limits=(-1., 1.))
        self.init_accel_pid()

        # use the correct time for further calculations
        sys.modules['simple_pid.PID']._current_time = (       # pylint: disable=protected-access
            lambda: self.get_time())

        if ROS_VERSION == 1:
            self.reconfigure_server = Server(
                EgoVehicleControlParameterConfig,
                namespace="",
                callback=self.reconfigure_pid_parameters,
            )
        if ROS_VERSION == 2:
            self. add_on_set_parameters_callback(self.reconfigure_pid_parameters)
        
        self.last_ackermann_msg_received_sec =  self.get_time()
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        # control info
        self.info = EgoVehicleControlInfo()

        print('init status.accel_control_pedal_target=', self.info.status.accel_control_pedal_target)

        # set initial maximum values, must have info updated
        self.vehicle_info_update_restrictions(self.vehicle_info)

        self.logical_status = LogicalStatus()
        self.logical_status.init_info(self.info, self.get_time())

        # ackermann drive commands
        self.control_subscriber = self.new_subscription(
            # AckermannDrive,
            CL_VehicleCommand,
            "/carla/" + self.role_name + "/ackermann_cmd",
            self.ackermann_command_updated,
           qos_profile=10
        )

        # current status of the vehicle
        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus,
            "/carla/" + self.role_name + "/vehicle_status",
            self.vehicle_status_updated,
           qos_profile=10
        )

        # vehicle info
        self.vehicle_info_subscriber = self.new_subscription(
            CarlaEgoVehicleInfo,
            "/carla/" + self.role_name + "/vehicle_info",
            self.vehicle_info_update_restrictions,
           qos_profile=10
        )

        # updateKp
        self.vehicle_info_subscriber = self.new_subscription(
            Float32,
            "/control/Kp",
            self.updateKp,
           qos_profile=10
        )

        
        self.vehicle_info_subscriber = self.new_subscription(
            Float32,
            "/control/clean_render",
            self.clean_render_datas,
           qos_profile=10
        )

        # vehicle Imu
        self.vehicle_info_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/imu",
            self.vehicle_imu_updated,
           qos_profile=10
        )

        # to send command to carla
        self.carla_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            qos_profile=10)

        # report controller info
        self.control_info_publisher = self.new_publisher(
            EgoVehicleControlInfo,
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            qos_profile=10)

    if ROS_VERSION == 1:

        def reconfigure_pid_parameters(self, ego_vehicle_control_parameter, _level):
            """
            Callback for dynamic reconfigure call to set the PID parameters

            :param ego_vehicle_control_parameter:
            :type ego_vehicle_control_parameter: \
                carla_ackermann_control.cfg.EgoVehicleControlParameterConfig

            """
            self.loginfo("Reconfigure Request:  "
                         "speed ({speed_Kp}, {speed_Ki}, {speed_Kd}), "
                         "accel ({accel_Kp}, {accel_Ki}, {accel_Kd})"
                         "".format(**ego_vehicle_control_parameter))
            self.speed_controller.tunings = (
                ego_vehicle_control_parameter['speed_Kp'],
                ego_vehicle_control_parameter['speed_Ki'],
                ego_vehicle_control_parameter['speed_Kd'],
            )
            self.accel_controller.tunings = (
                ego_vehicle_control_parameter['accel_Kp'],
                ego_vehicle_control_parameter['accel_Ki'],
                ego_vehicle_control_parameter['accel_Kd'],
            )
            return ego_vehicle_control_parameter

    if ROS_VERSION == 2:

        def reconfigure_pid_parameters(self, params):  # pylint: disable=function-redefined
            """Check and update the node's parameters."""
            param_values = {p.name: p.value for p in params}

            pid_param_names = {
                "speed_Kp",
                "speed_Ki",
                "speed_Kd",
                "accel_Kp",
                "accel_Ki",
                "accel_Kd",
            }
            common_names = pid_param_names.intersection(param_values.keys())
            if not common_names:
                # No work do to
                return SetParametersResult(successful=True)

            if any(p.value is None for p in params):
                return SetParametersResult(
                    successful=False, reason="Parameter must have a value assigned"
                )

            self.speed_controller.tunings = (
                param_values.get("speed_Kp", self.speed_controller.Kp),
                param_values.get("speed_Ki", self.speed_controller.Ki),
                param_values.get("speed_Kd", self.speed_controller.Kd),
            )
            self.accel_controller.tunings = (
                param_values.get("accel_Kp", self.accel_controller.Kp),
                param_values.get("accel_Ki", self.accel_controller.Ki),
                param_values.get("accel_Kd", self.accel_controller.Kd),
            )

            self.loginfo(
                "Reconfigure Request:  speed ({}, {}, {}), accel ({}, {}, {})".format(
                    self.speed_controller.tunings[0],
                    self.speed_controller.tunings[1],
                    self.speed_controller.tunings[2],
                    self.accel_controller.tunings[0],
                    self.accel_controller.tunings[1],
                    self.accel_controller.tunings[2]
                )
            )

            return SetParametersResult(successful=True)
    
    def vehicle_imu_updated(self, msg):
        if not isinstance(msg, Imu):
            return

        import math
        from transforms3d.euler import quat2euler
        quat = numpy.empty((4, ))
        quat[0] = msg.orientation.w 
        quat[1] = msg.orientation.x
        quat[2] = msg.orientation.y
        quat[3] =  msg.orientation.z


        ax, ay, az = quat2euler(quat)
        cord = msg.linear_acceleration

        theta_x = msg.orientation.x #abs(msg.orientation.x)
        delta_x = cord.x - 0
        delta_y = cord.y - 0
        delta_z = cord.z - 9.81
        world_x = delta_x  * math.cos(ax)  + ( delta_y * math.sin(ax) + delta_z * math.sin(ax))

        #imu direct value
        #从静止到启动的瞬时加速度巨大, 直接剪切裁剪到目标加速度，使得PID回归

        accel = numpy.clip(cord.x, -14, 6.3)
        accel = (self.info.current.accel * 9 + accel) / 10
        # self.set_current_accel(accel)

        # self.info.current.accel = accel
    def get_msg_header(self):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        header.frame_id = "map"
        header.stamp = roscomp.ros_timestamp(sec=self.get_time(), from_sec=True)
        return header

    def vehicle_status_updated(self, vehicle_status:CarlaEgoVehicleStatus):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        self.vehicle_status = vehicle_status

        # print(self.vehicle_status)
        # current_time_sec = self.get_time()
        # print('>>>>', current_time_sec)
        # print()

        self.update_current_values()
        
        self.logical_status.all_imu_accer.append(self.info.current.accel)
        self.logical_status.all_pid_accel.append(self.info.target.accel)

        self.vehicle_control_cycle()
        self.send_ego_vehicle_control_info_msg()

        current_accel = vehicle_status.acceleration.linear.y
        # self.set_current_accel(-current_accel)



    def vehicle_info_update_restrictions(self, vehicle_info):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        # set target values
        self.vehicle_info = vehicle_info

        # calculate restrictions
        self.info.restrictions.max_steering_angle = phys.get_vehicle_max_steering_angle(
            self.vehicle_info)
        self.info.restrictions.max_speed = phys.get_vehicle_max_speed(
            self.vehicle_info)
        self.info.restrictions.max_accel = phys.get_vehicle_max_acceleration(
            self.vehicle_info)
        self.info.restrictions.max_decel = phys.get_vehicle_max_deceleration(
            self.vehicle_info)
        self.info.restrictions.min_accel = self.get_param('min_accel', 0.1)
        # clipping the pedal in both directions to the same range using the usual lower
        # border: the max_accel to ensure the the pedal target is in symmetry to zero
        self.info.restrictions.max_pedal = min(
            self.info.restrictions.max_accel, self.info.restrictions.max_decel)

    def ackermann_command_updated(self, ros_ackermann_drive):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """

        #start:补充非0起步的时候target设置的情况
        if self.logical_status.inited_when_restart and abs(self.info.status.accel_control_pedal_target) <= 0.0001 \
            and self.info.current.speed_abs > self.logical_status.full_stop_speed_epsilon:

            self.info.status.accel_control_pedal_target = 0.5
            print('happens when restart, 0.5 target')
            self.logical_status.inited_when_restart = True
        #end

        self.last_ackermann_msg_received_sec = self.get_time()
        self.set_target_steering_angle(ros_ackermann_drive.CL_phiTargetStrAngle)
        # self.set_target_speed(ros_ackermann_drive.speed)
        
        neg_pos_speed = self.info.current.speed + (ros_ackermann_drive.CL_aTargetLongAcc*3.6)
        self.set_target_speed(numpy.clip(neg_pos_speed, 5/3.6, self.info.restrictions.max_speed))
        self.set_target_accel(ros_ackermann_drive.CL_aTargetLongAcc)
        self.set_target_jerk(0.0)

    def update_current_values(self):
        """
        Function to update vehicle control current values.

        we calculate the acceleration on ourselves, because we are interested only in
        the acceleration in respect to the driving direction
        In addition a small average filter is applied

        :return:
        """
        current_time_sec = self.get_time()
        current_speed = round(self.vehicle_status.velocity, 4)

        delta_speed = round(current_speed - self.info.current.speed, 4)
        current_accel = round(delta_speed / 0.005, 4)
        # current_accel = (self.info.current.accel * 4 + current_accel)/5
        self.set_current_accel(current_accel)


        x_acc = self.vehicle_status.acceleration.linear.x*3.6 
        if self.info.current.accel - x_acc >= 0.01:
            print(self.info.current.accel , x_acc, self.info.current.accel - x_acc )

        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

        if self.info.current.speed_abs < self.logical_status.standing_still_epsilon \
            and self.info.target.accel < 0:
            
            self.set_target_accel(0)
            self.info.status.accel_control_pedal_target = 0
            # self.info.output.throttle = 0
            # self.info.output.brake = 0

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        self.info.target.steering_angle = -target_steering_angle
        if abs(self.info.target.steering_angle) > self.info.restrictions.max_steering_angle:
            self.logerr("Max steering angle reached, clipping value from {} clip.range {}"\
                .format(self.info.target.steering_angle, self.info.restrictions.max_steering_angle))
            self.info.target.steering_angle = numpy.clip(
                self.info.target.steering_angle,
                -self.info.restrictions.max_steering_angle,
                self.info.restrictions.max_steering_angle)

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.restrictions.max_speed:
            self.logerr("Max speed reached, clipping value")
            self.info.target.speed = numpy.clip(
                target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
        else:
            self.info.target.speed = target_speed
        self.info.target.speed_abs = abs(self.info.target.speed)

    
    def set_target_accel(self, target_accel):
        target_accel = round(target_accel, 4)
        """
        set target accel
        """
        # if speed is set to zero, then use max decel value
        if self.info.target.speed_abs < self.logical_status.standing_still_epsilon:
            self.info.target.accel = -self.info.restrictions.max_decel

            # setpoint of the acceleration controller is the output of the speed controller
            self.accel_controller.setpoint =  self.info.target.accel 
           
            # self.info.status.accel_control_pedal_target  = self.info.target.accel
        else:
            old = self.info.target.accel
            self.info.target.accel = numpy.clip(target_accel, 
            - self.info.restrictions.max_decel, self.info.restrictions.max_decel)

            if abs(old - self.info.target.accel) >= 0.1:
                print('accel changed from {} to {}'.format(old, self.info.target.accel))
                self.logical_status.reqached_new_target = False
                self.logical_status.get_to_normal__times = 0

                # setpoint of the acceleration controller is the output of the speed controller
                # self.accel_controller.auto_mode = False
                # self.info.status.accel_control_pedal_target = numpy.clip(self.info.target.accel, -1, 1)
                self.accel_controller.setpoint =  self.info.target.accel 
                self.accel_controller.auto_mode = True
                # self.info.status.accel_control_pedal_target  = self.info.target.accel

    def set_target_jerk(self, target_jerk):
        """
        set target accel
        """
        self.info.target.jerk = target_jerk

    
    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        # perform actual control
        self.control_steering()
        self.control_stop_and_reverse()
        
        # self.run_speed_control_loop()
        
       
        self.run_accel_control_loop()
        self.update_drive_vehicle_control_command()
        if not self.info.output.hand_brake:
            # only send out the Carla Control Command if AckermannDrive messages are
            # received in the last second (e.g. to allows manually controlling the vehicle)
            if True or (self.last_ackermann_msg_received_sec + 1.0) >  self.get_time():
                self.info.output.header = self.get_msg_header()
                import time 
                # time.sleep(0.2)
                self.carla_control_publisher.publish(self.info.output)

    def control_steering(self):
        """
        Basic steering control
        """
        self.info.output.steer = self.info.target.steering_angle / \
            self.info.restrictions.max_steering_angle

    def control_stop_and_reverse(self):
        """
        Handle stop and switching to reverse gear
        """

        # auto-control of hand-brake and reverse gear
        self.info.output.hand_brake = False
        if self.info.current.speed_abs < self.logical_status.standing_still_epsilon:
            # standing still, change of driving direction allowed
            self.info.status.status = "standing"
            if self.info.target.speed < 0:
                if not self.info.output.reverse:
                    self.loginfo(
                        "VehicleControl: Change of driving direction to reverse")
                    self.info.output.reverse = True
            elif self.info.target.speed > 0:
                if self.info.output.reverse:
                    self.loginfo(
                        "VehicleControl: Change of driving direction to forward")
                    self.info.output.reverse = False
            if self.info.target.speed_abs < self.logical_status.full_stop_speed_epsilon:
                self.info.status.status = "full stop"
                self.info.status.speed_control_accel_target = 0.
                self.info.status.accel_control_pedal_target = 0.
                self.set_target_speed(0.)
                self.info.current.speed = 0.
                self.info.current.speed_abs = 0.
                self.info.current.accel = 0.
                self.info.output.hand_brake = True
                self.info.output.brake = 1.0
                self.info.output.throttle = 0.0

        elif numpy.sign(self.info.current.speed) * numpy.sign(self.info.target.speed) == -1:
            # requrest for change of driving direction
            # first we have to come to full stop before changing driving
            # direction
            self.loginfo("VehicleControl: Request change of driving direction."
                         " v_current={} v_desired={}"
                         " Set desired speed to 0".format(self.info.current.speed,
                                                          self.info.target.speed))
            self.set_target_speed(0.)

    def run_speed_control_loop(self):
        """
        Run the PID control loop for the speed

        The speed control is only activated if desired acceleration is moderate
        otherwhise we try to follow the desired acceleration values

        Reasoning behind:

        An autonomous vehicle calculates a trajectory including position and velocities.
        The ackermann drive is derived directly from that trajectory.
        The acceleration and jerk values provided by the ackermann drive command
        reflect already the speed profile of the trajectory.
        It makes no sense to try to mimick this a-priori knowledge by the speed PID
        controller.
        =>
        The speed controller is mainly responsible to keep the speed.
        On expected speed changes, the speed control loop is disabled
        """
        epsilon = 0.00001
        target_accel_abs = abs(self.info.target.accel)
        if target_accel_abs < self.info.restrictions.min_accel:
            if self.info.status.speed_control_activation_count < 5:
                self.info.status.speed_control_activation_count += 1
        else:
            if self.info.status.speed_control_activation_count > 0:
                self.info.status.speed_control_activation_count -= 1
        # set the auto_mode of the controller accordingly
        self.speed_controller.auto_mode = self.info.status.speed_control_activation_count >= 5
        
        if self.speed_controller.auto_mode:
            self.speed_controller.setpoint = self.info.target.speed_abs
            self.info.status.speed_control_accel_delta = float(self.speed_controller(
                self.info.current.speed_abs))

            # clipping borders
            clipping_lower_border = -target_accel_abs
            clipping_upper_border = target_accel_abs
            # per definition of ackermann drive: if zero, then use max value
            if target_accel_abs < epsilon:
                clipping_lower_border = -self.info.restrictions.max_decel
                clipping_upper_border = self.info.restrictions.max_accel
            self.info.status.speed_control_accel_target = numpy.clip(
                self.info.status.speed_control_accel_target +
                self.info.status.speed_control_accel_delta,
                clipping_lower_border, clipping_upper_border)
        else:
            self.info.status.speed_control_accel_delta = 0.
            self.info.status.speed_control_accel_target = self.info.target.accel

    def run_accel_control_loop(self):
        """
        Run the PID control loop for the acceleration
        """
        if self.info.current.speed_abs < self.logical_status.standing_still_epsilon:
            self.accel_controller.output_limits = (0, 0.1)
        else:
            self.accel_controller.output_limits = (-14, 7.3)
        self.info.status.accel_control_pedal_delta = float(self.accel_controller(
            self.info.current.accel))

        if self.info.output.throttle >= 1.0 and self.info.status.accel_control_pedal_delta > 0:
            pass
        elif self.info.output.brake >= 1.0 and  self.info.status.accel_control_pedal_delta < 0:
            
            pass
        else:
            self.info.status.accel_control_pedal_target = self.info.status.accel_control_pedal_target \
                + self.info.status.accel_control_pedal_delta

            self.info.status.accel_control_pedal_delta = \
                numpy.clip(self.info.status.accel_control_pedal_delta, -1, 1)
    
    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """
        # the driving impedance moves the 'zero' acceleration border
        # Interpretation: To reach a zero acceleration the throttle has to pushed
        # down for a certain amount
        #mark: to keep speed current,must at leat use this throttle to get the accel
        self.info.status.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(
        self.vehicle_info, self.vehicle_status, self.info.output.reverse) 

        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        self.info.status.brake_upper_border = self.info.status.throttle_lower_border + \
            phys.get_vehicle_lay_off_engine_acceleration(self.vehicle_info)
            
        old_throttle = self.info.output.throttle

        self.logical_status.in_cold_starting = False  #change this as next if False in the same time
        if False  and self.info.current.speed_abs <= self.logical_status.full_stop_speed_epsilon \
            and self.info.target.accel >= 0.1: #ignored backward
            self.info.output.brake = 0.0
            print('force start', self.logical_status.cold_counter)
            self.info.output.throttle = self.info.status.accel_control_pedal_target  
            return

            self.logical_status.in_cold_starting = True
            self.logical_status.cold_counter += 1

            if self.logical_status.cold_counter <= 20:
                self.info.status.status = "accelerating"

                self.info.output.throttle = 1

                self.info.output.brake = 0.0
                print('force start', self.logical_status.cold_counter)
                self.info.status.accel_control_pedal_target  =   self.info.output.throttle 
                self.accel_controller.auto_mode = False

                # self.reinit_accel_pid(0)
        else:
            if self.logical_status.in_cold_starting == True:
                self.logical_status.in_cold_starting = False
                
                ranges = [3, 2.5, 2, 1.5, 1, 0.5]
                throttle_alpha = 1
                for i in range(len(ranges)):
                    if self.info.target.accel >= ranges[i]:
                        throttle_alpha = i+1
                        break

                self.info.status.status = "accelerating"
                self.info.output.throttle = 1/throttle_alpha
                print('started success>>>>>>>>>>>>>>>>>>>>>>>>>', self.info.output.throttle )

                self.info.status.accel_control_pedal_target = self.info.output.throttle 
                self.info.output.brake = 0.0
                self.logical_status.cold_counter = 0
                self.accel_controller.auto_mode = True
            else:
                # if self.info.target.accel - 0.1 >=  self.info.current.accel:
                if self.info.status.accel_control_pedal_target >= 0:
                    # if self.info.current.accel < self.info.target.accel:
                    self.info.status.status = "accelerating"
                    self.info.output.brake = 0.0
                    # the value has to be normed to max_pedal
                    # be aware: is not required to take throttle_lower_border into the scaling factor,
                    # because that border is in reality a shift of the coordinate system
                    # the global maximum acceleration can practically not be reached anymore because of
                    # driving impedance            
                    new_value =  self.info.status.accel_control_pedal_target 

                    self.info.output.throttle = new_value
                else:  #self.info.current.accel >= self.info.target.accel + 0.1:
                    # print('braking')
                    self.info.status.status = "braking"
                    # braking required
                    self.info.output.throttle =  0
                    self.info.output.brake = 0 - self.info.status.accel_control_pedal_target 

        # finally clip the final control output (should actually never happen)
        self.info.output.brake = numpy.clip(
            self.info.output.brake, 0., 1.)
        self.info.output.throttle = numpy.clip(
            round(self.info.output.throttle,4) , 0., 1.)

        throttle_delta =  self.info.output.throttle - old_throttle
        self.logical_status.append_throttle_delta(throttle_delta, self.info.status.accel_control_pedal_delta)
        self.logical_status.append_info(self.info)

    def init_accel_pid(self, delta_to_old = 0):
        self.accel_controller = PID(Kp=self.get_param("accel_Kp", alternative_value=0.05),
                                    Ki=self.get_param("accel_Ki", alternative_value=0.),
                                    Kd=self.get_param("accel_Kd", alternative_value=0.05),
                                    sample_time=self.control_loop_rate,
                                    output_limits=(-14, 6.3))
        self.accel_controller.time_fn = rospy.get_rostime

    def updateKp(self, value):
        print('Kp changes from to', round(self.accel_controller.Kp, 3), round(value.data, 3))
        self.accel_controller.Kp = value.data

    def clean_render_datas(self, any):
        self.logical_status.clean_plot()

    def reinit_accel_pid(self, delta_to_old=0):
        return
        if self.logical_status.reqached_new_target:
            return
        delta_to_target = round(self.info.target.accel - self.info.current.accel, 3)
        if abs(self.info.target.accel - self.info.current.accel) >  2*(0.4 if self.info.target.accel < 0 else 0.2):
            # Kp = min(1.0, abs(delta_to_target)/6.7)
            Kp = 0.5
            # if self.info.target.accel < 0:
            #     Kp *= 2
            if Kp == self.accel_controller.Kp:
                print('======================================far, index=', len(self.logical_status.all_imu_accer), 'delta to target', round(self.info.target.accel  - self.info.current.accel, 3) )
                return
            
            print('++++++++++++++++======================far, Kp=', Kp,
                '\t imu position=', len(self.logical_status.all_imu_accer),
                '\t target_position', len(self.logical_status.all_pid_accel)  )
        elif abs(self.info.target.accel - self.info.current.accel) >  (0.4 if self.info.target.accel < 0 else 0.2):
            # Kp = min(1.0, abs(delta_to_target)/6.7)
            Kp = 0.1
            # if self.info.target.accel < 0:
            #     Kp *= 2
            if Kp == self.accel_controller.Kp:
                print('====================far, index=', len(self.logical_status.all_imu_accer), 'delta to target', round(self.info.target.accel  - self.info.current.accel, 3) )
                return
            
            print('++++++++++++++++far, Kp=', Kp,
                '\t imu position=', len(self.logical_status.all_imu_accer),
                '\t target_position', len(self.logical_status.all_pid_accel)  )
            
        elif  (0.4 if self.info.target.accel < 0 else 0.2)/2 < abs(self.info.target.accel - self.info.current.accel) <= (0.4 if self.info.target.accel < 0 else 0.2):
            Kp = 0.01
            # if self.info.target.accel < 0:
            #     Kp *= 2
            if Kp == self.accel_controller.Kp:
                print('________near', len(self.logical_status.all_imu_accer), 'delta to target', round(self.info.target.accel  - self.info.current.accel, 3))
                return

            print('-------------------------------------near , Kp=', Kp,  'delta to target', round(self.info.target.accel  - self.info.current.accel, 3),
                '\t imu position=', len(self.logical_status.all_imu_accer),
                '\t target_position', len(self.logical_status.all_pid_accel)  )

        else:
            Kp=self.get_param("accel_Kp", alternative_value=0.05)
            if self.info.target.accel < 0:
                Kp *= 2
            
            print('////////////////////////////////recover pid to normal, Kp=',Kp,   
             'delta to target', round(self.info.target.accel  - self.info.current.accel, 3),
                '\t imu position=', len(self.logical_status.all_imu_accer),
                '\t target_position', len(self.logical_status.all_pid_accel)  )
            
            self.logical_status.get_to_normal__times += 1
            if self.logical_status.get_to_normal__times >= 2: 
                self.logical_status.reqached_new_target = True

            if Kp == self.accel_controller.Kp:
                return
        
        # if self.info.target.accel * self.info.status.accel_control_pedal_target < 0:
        #     if self.info.target.accel > 0:
        #         self.info.status.accel_control_pedal_target = 
        #     else:
        #         self.info.status.accel_control_pedal_target = 0

        self.accel_controller.Kp = Kp
        self.accel_controller.time_fn = rospy.get_rostime

        self.keep_last_change_count = 0

    # from ego vehicle
    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.header = self.get_msg_header()
        import threading
        threading.Thread(target = lambda : self.control_info_publisher.publish(self.info)).run()
        
    
    
    
    def set_current_accel(self, current_accel):
        current_accel = numpy.clip(current_accel, -14, 6.3)

        self.info.current.accel = current_accel 
        if not self.logical_status.in_cold_starting:
            self.reinit_accel_pid(0)
            
    def run(self):
        """
        Control loop
        :return:
        """
        import sys, signal
        def signal_handler(signal, frame):
            print("\nprogram exiting gracefully")
            self.logical_status.stop_plt()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        while(True):
            self.logical_status.make_plt(self.info, self.control_loop_rate, self.accel_controller)
            import rospy
            # rospy.sleep(0.05)
            import time
            time.sleep(0.05)

def main(args=None):
    roscomp.init("carla_ackermann_control", args=args)

    try:
        controller = CarlaAckermannControl()
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
