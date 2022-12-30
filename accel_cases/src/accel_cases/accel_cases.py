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

from std_msgs.msg import Float32


import pygame

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    
    from pygame.locals import K_a,K_b, K_c,K_d,K_e, K_f,K_g,K_h,K_i
    from pygame.locals import K_j, K_k,K_l,K_m,K_n,K_o,K_p,K_q,K_r
    from pygame.locals import K_s, K_t, K_u,K_v,K_w,K_x,K_y, K_z

    from pygame.locals import K_F1, K_F2, K_F3,K_F4,K_F5,K_F6,K_F7,K_F8,K_F9,K_F10,K_F11, K_F12
    from pygame.locals import K_0, K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

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

        


        self.control_publisher = self.new_publisher(
            CL_VehicleCommand,  
            "/carla/ego_vehicle/ackermann_cmd", 
            qos_profile=10)
        
        self.Kp_publisher = self.new_publisher(
            Float32,
            "/control/Kp",
           qos_profile=10
        )

        self.render_fixed_delay_publisher = self.new_publisher(
            Float32,
            "/carla/sensors/delay_time",
           qos_profile=10
        )

        self.clean_render  = self.new_publisher(
            Float32,
            "/control/clean_render",
           qos_profile=10
        )
       

    def vehicle_status_updated(self, vehicle_status):
        self.vehicle_status = vehicle_status
        self.angle = 0.0

        # if self.switch_to_positive:
        #     self.choce_target_by_speed_reach_milestones(vehicle_status)
        # else:
        #     self.choce_target_by_slowdown_speed_reach_milestones(vehicle_status)
        
        
        # self.publish_cl_control()

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
            if  self.target < 0 and (  self.small_reach_counting > small_keep_times  or  vehicle_status.velocity *3.6 <= 3):
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

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (\
            key == K_q and pygame.key.get_mods() & KMOD_CTRL)

    def parse_event(self):
        has_event = False
        for event in pygame.event.get():
            
           
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                has_event = True

                if self._is_quit_shortcut(event.key):
                    return True

                key = event.key
                print(key)
                f_keys =  [K_F1, K_F2, K_F3, K_F4, K_F5, K_F6, K_F7, 
                K_F8, K_F9, K_F10, K_F11, K_F12]
                udlr_keys = [K_DOWN, K_UP, K_LEFT, K_RIGHT]
                if key in f_keys + udlr_keys:

                    if key in f_keys:
                        kv = {K_F1:-6, K_F2:-5, K_F3:-4, K_F4:-3, K_F5:-2, 
                        K_F6:-1, K_F7:0, K_F8:1 ,K_F9:2, K_F10:3, K_F11:3.7, 
                        K_F12:4}

                        self.target = kv[key]
                        
                    elif key == K_DOWN:
                        self.target -= 0.5
                        self.angle = 0
                    elif key == K_UP:
                        self.target += 0.5
                        self.angle = 0
                    elif key == K_LEFT:
                        # self.target -= 0.1
                        self.angle -= 10
                    elif key == K_RIGHT:
                        # self.target += 0.1
                        self.angle += 10

                    self.target = np.clip(self.target, -7.2, 3.7)
                    self.publish_cl_control()

                    while(abs(self.angle) >= 1):
                        self.angle -=1
                        self.publish_cl_control()
                    
                elif key in [K_1, K_2, K_3, K_4, K_5,]:
                    kv = { K_1:1,K_2:0.2, K_3:0.04, K_4:0.01, K_5:0.0}
                    self.render_fixed_delay_publisher.publish(Float32(kv[key]))
                elif key in [ K_0, K_6, K_7, K_8, K_9]:

                    kv = {K_6:0.015, K_7:0.014,
                     K_8:0.013, K_9:0.012,K_0:0.011}
                    self.Kp_publisher.publish(Float32(kv[key]))

                    #记录,0.14：30kmkp以内0到3，可以在40个step也就是200ms内达到目标，且几乎没有超出
                elif key == K_c:
                    self.clean_render.publish(Float32())

               

    def publish_cl_control(self):
        print('will send accel {} angle {} '.format(self.target, self.angle))
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
        
    def run(self):
        """
        Control loop
        :return:
        """


        return self.parse_event()


def main(args=None):
    roscomp.init("carla_ackermann_contro_casesl", args=args)
    
    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("accel changer")
    clock = pygame.time.Clock()

    import sys, signal
    def signal_handler(signal, frame):
        print("\nprogram exiting gracefully")
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        controller = TestBaseOnSpeed()
        from pygame import HWSURFACE, DOUBLEBUF
        display = pygame.display.set_mode((400,300),
            HWSURFACE | DOUBLEBUF)

        while roscomp.ok():
            clock.tick_busy_loop(200)
            if controller.run():
                return
            pygame.display.flip()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
