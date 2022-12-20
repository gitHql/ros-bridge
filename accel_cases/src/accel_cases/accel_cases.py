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
from simple_pid import PID  # pylint: disable=import-error,wrong-import-order

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from demo_msgs.msg import CL_VehicleCommand
from std_msgs.msg import Header # pylint: disable=wrong-import-order
from sensor_msgs.msg import Imu

from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
import random, numpy as np
from random import uniform, choice


from demo_msgs.msg import CL_VehicleCommand

class TestBaseOnSpeed(CompatibleNode):
    def __init__(self):
        super(TestBaseOnSpeed, self).__init__("accel_cases")

        self.vehicle_status = CarlaEgoVehicleStatus()
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        
        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus,
            "/carla/" + self.role_name + "/vehicle_status",
            self.vehicle_status_updated,
           qos_profile=10
        )


        self.control_publisher = self.new_publisher(CL_VehicleCommand,  
            "/carla/ego_vehicle/ackermann_cmd", 
            qos_profile=10)

    def vehicle_status_updated(self, vehicle_status):
        self.vehicle_status = vehicle_status
        
        if self.switch_to_positive:
            self.choce_target_by_speed_reach_milestones(vehicle_status)
        else:
            self.choce_target_by_slowdown_speed_reach_milestones(vehicle_status)
        print(self.target)
        self.angle = 0.0
        self.publish_cl_control()

    switch_to_positive = True

    def choce_target_by_speed_reach_milestones(self, vehicle_status:CarlaEgoVehicleStatus):
        def delta(next_speed_kph):
            return  (  (vehicle_status.velocity * 3.6 ) - next_speed_kph) / 20 
        if vehicle_status.velocity * 3.6 >= 0:
            self.target = delta(0)
        if vehicle_status.velocity * 3.6 >= 10:
            self.target = 2  + delta(10)
        if vehicle_status.velocity * 3.6 >= 30:
            self.target = 3 + delta(30)
        if vehicle_status.velocity * 3.6 >= 50:
            self.target = 2 + delta(50)
        if vehicle_status.velocity * 3.6 >= 70:
            self.target = 1 + delta(70)
            # self.switch_to_positive = False

        if vehicle_status.velocity * 3.6 >= 90:
            self.target = 0
            self.switch_to_positive = False
       
    def choce_target_by_slowdown_speed_reach_milestones(self, vehicle_status:CarlaEgoVehicleStatus):
        if vehicle_status.velocity * 3.6 < 5:
            self.target = 1
            self.switch_to_positive = True
            return

        if vehicle_status.velocity * 3.6 >= 10:
        #     self.target = -6 + vehicle_status.velocity * 3.6 / 30 
        # if vehicle_status.velocity * 3.6 >= 30:
        #     self.target = -5 + vehicle_status.velocity * 3.6 / 50 
        # if vehicle_status.velocity * 3.6 >= 50:
        #     self.target = -4 + vehicle_status.velocity * 3.6 / 70 
        # if vehicle_status.velocity * 3.6 >= 70:
        #     self.target = -3 + vehicle_status.velocity * 3.6 / 90
        # if vehicle_status.velocity * 3.6 >= 90:
        #     self.target = -2 + vehicle_status.velocity * 3.6 / 110 
        # if vehicle_status.velocity *3.6 >= 110:
        #     self.target = -1
        # if vehicle_status.velocity *3.6 >= 130:
            self.target = -5
        
    def choice_target(self, vehicle_status:CarlaEgoVehicleStatus):
        if vehicle_status.velocity * 3.6 > 20:
            self.big_reached_counting += 1
            self.small_reach_counting = 0
            
        else:
            if vehicle_status.velocity *3.6 < 10:
                self.small_reach_counting += 1
                self.big_reached_counting = 0
            pass
            # self.big_reached_counting -= 2

        big_keep_times, small_keep_times =10, 10
        if self.target > 0 and (self.big_reached_counting > big_keep_times or  vehicle_status.velocity *3.6 > 90 ):
            #fall down
            self.big_reached_counting = 0
            self.small_reach_counting = 0
            start, end = -2*self.PID_MAX_TARGET, -0.2

            # self.target = round(uniform(start, end), 1)
            self.target = choice([i for i in  np.arange(start, end, 0.4)])
            print('======================target changed to {}'.format(self.target))
        else:
            if  self.target < 0 and (  s55555elf.small_reach_counting > small_keep_times  or  vehicle_status.velocity *3.6 <= 3):
                #rise up
                self.big_reached_counting = 0
                self.small_reach_counting = 0
                start, end = 0.2, self.PID_MAX_TARGET

                # self.target = round(uniform(start, end), 1)
                self.target = choice([i for i in np.arange(start, end, 0.4)])
                print('======================target changed to {}'.format(self.target))
            pass

    PID_MAX_TARGET = 3.7
    target = 1
    angle = 0.0
    big_reached_counting = 0
    small_reach_counting  = 0

    def publish_cl_control(self):
        import numpy
        self.target = numpy.clip(self.target, -2*self.PID_MAX_TARGET, self.PID_MAX_TARGET)
        msg = CL_VehicleCommand()
        msg.header.seq = 1 
        # msg.stamp = 0
        # msg.frame_id= 'ssss'
        msg.CL_stSysSts= 1
        msg.CL_flgAccelEnable= True
        msg.CL_flgStrWhlEnable= False
        msg.CL_nStrWhlSpeed= 0
        msg.CL_flgGearEnable= False
        msg.CL_flgLeftTurnLight= 1
        msg.CL_flgRightTurnLight= 1
        msg.CL_phiTargetStrAngle= self.angle

        msg.CL_gearTargetGear= 0
        msg.CL_aTargetLongAcc= self.target

        self.control_publisher.publish(msg)
        
        # print(round( imu_msg.linear_acceleration.x, 3 ), round( self.target, 3))



    #     reached_target = -0.1 < imu_msg.linear_acceleration.x  -  self.target < 0.1
    #     near_zero = (-0.1 < imu_msg.linear_acceleration.x < 0.1)
    #     from random import uniform

    #     if reached_target:
    #         self.reached_counting += 1
            
    #         if self.reached_counting > 500:
    #             self.reached_counting = 0
    #             if self.target > 0:
    #                 start, end = -self.PID_MAX_TARGET, 0
    #             else:
    #                 start, end = 0, self.PID_MAX_TARGET

    #             # start, end = 0, self.PID_MAX_TARGET  #屏蔽正负区间的翻转
    #             self.target = round(uniform(start, end), 1)
    #             print('======================target changed to {}'.format(self.target))
    #     else:
    #         self.reached_counting -= 2
            
    #         if self.target > 0 and   imu_msg.linear_acceleration.x < self.target:
    #             self.max_accel_count += 1
    #         else:
    #             self.max_accel_count = 0
            
    #         if self.max_accel_count > 100:
    #             self.max_accel_count = 0
    #             print("max_speed reached but accel won't kee")
    #             self.target =round(uniform(-self.PID_MAX_TARGET, 0), 1)

    #         if self.target < 0 and  imu_msg.linear_acceleration.x > self.target:
    #             self.won_reach_negative_count += 1
    #         else:
    #             self.won_reach_negative_count = 0
    #         if self.won_reach_negative_count > 100:
    #             self.won_reach_negative_count = 0
    #             print("already min accel but accel won't get there", self.target)
    #             self.target =round(uniform(0,self.PID_MAX_TARGET), 1)

    #         else:
    #             pass
    #             # print('invalid logic, target=', self.target, 'actual=', imu_msg.linear_acceleration.x)
    #     self.publish_cl_control()

    # PID_MAX_TARGET = 1
    # target = 1
    # reached_counting = 0
    # max_accel_count = 0
    # won_reach_negative_count = 0

    # def publish_cl_control(self):
    #     import numpy
    #     self.target = numpy.clip(self.target, -self.PID_MAX_TARGET, self.PID_MAX_TARGET)
    #     msg = CL_VehicleCommand()
    #     msg.header.seq = 1 
    #     # msg.stamp = 0
    #     # msg.frame_id= 'ssss'
    #     msg.CL_stSysSts= 1
    #     msg.CL_flgAccelEnable= True
    #     msg.CL_flgStrWhlEnable= False
    #     msg.CL_nStrWhlSpeed= 0
    #     msg.CL_flgGearEnable= False
    #     msg.CL_flgLeftTurnLight= 1
    #     msg.CL_flgRightTurnLight= 1
    #     msg.CL_phiTargetStrAngle= 0.0

    #     msg.CL_gearTargetGear= 1
    #     msg.CL_aTargetLongAcc= self.target

    #     self.control_publisher.publish(msg)
    def run(self):
        """
        Control loop
        :return:
        """
        import sys, signal
        def signal_handler(signal, frame):
            print("\nprogram exiting gracefully")
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        while(True):
            # print('+')
            import rospy
            rospy.sleep(0.05)

def main(args=None):
    roscomp.init("carla_ackermann_contro_casesl", args=args)

    try:
        controller = TestBaseOnSpeed()
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
