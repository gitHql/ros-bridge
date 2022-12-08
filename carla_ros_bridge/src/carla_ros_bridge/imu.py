#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla imu sensor
"""

from transforms3d.euler import euler2quat
import carla_common.transforms as trans
from carla_ros_bridge.sensor import Sensor
from sensor_msgs.msg import Imu

from demo_msgs.msg import CL_VehicleCommand


class ImuSensor(Sensor):

    """
    Actor implementation details for imu sensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor : carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """

        # print('carla_actor.attr', carla_actor.attributes)
        super(ImuSensor, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)
        # print('==============================================================sensor_tick_time', self.sensor_tick_time)
        self.imu_publisher = node.new_publisher(Imu, self.get_topic_prefix(), qos_profile=10)
        self.control_publisher = node.new_publisher(CL_VehicleCommand,  
        "/carla/ego_vehicle/ackermann_cmd", 
        qos_profile=100)
        self.listen()

    def destroy(self):
        super(ImuSensor, self).destroy()
        self.node.destroy_publisher(self.imu_publisher)

    # pylint: disable=arguments-differrospy.spin()
    count = 0
    def sensor_data_updated(self, carla_imu_measurement):
        
        """
        Function to transform a received imu measurement into a ROS Imu message

        :param carla_imu_measurement: carla imu measurement object
        :type carla_imu_measurement: carla.IMUMeasurement
        """
        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(timestamp=carla_imu_measurement.timestamp)
        self.count +=1 
        # print('imu_msg.header' , imu_msg.header, self.count)
        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        roll, pitch, yaw = trans.carla_rotation_to_RPY(carla_imu_measurement.transform.rotation)
        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        self.imu_publisher.publish(imu_msg)
        # print(round( imu_msg.linear_acceleration.x, 3 ), round( self.target, 3))



        reached_target = -0.1 < imu_msg.linear_acceleration.x  -  self.target < 0.1
        near_zero = (-0.1 < imu_msg.linear_acceleration.x < 0.1)
        from random import uniform

        if reached_target:
            self.reached_counting += 1
            
            if self.reached_counting > 100:
                self.reached_counting = 0
                if self.target > 0:
                    start, end = -self.PID_max, 0
                else:
                    start, end = 0, self.PID_max

                # start, end = 0, self.PID_max  #屏蔽正负区间的翻转
                self.target = round(uniform(start, end), 3)
                print('======================target changed to {}'.format(self.target))
        else:
            self.reached_counting = 0
            
            if self.target > 0 and   imu_msg.linear_acceleration.x < self.target:
                self.max_accel_count += 1
            else:
                self.max_accel_count = 0
            
            if self.max_accel_count > 100:
                self.max_accel_count = 0
                print("max_speed reached but accel won't kee")
                self.target =round(uniform(-self.PID_max, 0), 3)

            if self.target < 0 and  imu_msg.linear_acceleration.x > self.target:
                self.won_reach_negative_count += 1
            else:
                self.won_reach_negative_count = 0
            if self.won_reach_negative_count > 100:
                self.won_reach_negative_count = 0
                print("already min accel but accel won't get there", self.target)
                self.target =round(uniform(0,self.PID_max), 3)

            else:
                print('invalid logic', self.target, imu_msg.linear_acceleration.x)
        self.publish_cl_control()

    PID_max = 1
    target = 1
    reached_counting = 0
    max_accel_count = 0
    won_reach_negative_count = 0

    def publish_cl_control(self):
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
        msg.CL_phiTargetStrAngle= 0.0

        msg.CL_gearTargetGear= 1
        msg.CL_aTargetLongAcc= self.target

       

        self.control_publisher.publish(msg)