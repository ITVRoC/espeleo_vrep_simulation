#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, tan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
import numpy as np

import sys



"""
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
"""


# Callback for the accelerometer data
def callback_acc(data):
    global accel

    alpha = 0.8
    accel[0] = alpha*accel[0] + (1.0-alpha)*data.x
    accel[1] = alpha*accel[1] + (1.0-alpha)*data.y
    accel[2] = alpha*accel[2] + (1.0-alpha)*data.z

    return
# ----------  ----------  ----------  ----------  ----------


# Callback for the gyro data
def callback_gyro(data):
    global gyro

    alpha = 0.8
    gyro[0] = alpha*gyro[0] + (1.0-alpha)*data.x
    gyro[1] = alpha*gyro[1] + (1.0-alpha)*data.y
    gyro[2] = alpha*gyro[2] + (1.0-alpha)*data.z

    return
# ----------  ----------  ----------  ----------  ----------


# Callback to get the pose of the robot
def callback_pose_tf(data):

    global gps


    for T in data.transforms:
        # Chose the transform of the EspeleoRobo
        if (T.child_frame_id == "EspeleoRobo"):

            # Get the position
            pos_xyz = [0.0, 0.0, 0.0]
            pos_xyz[0] = T.transform.translation.x
            pos_xyz[1] = T.transform.translation.y
            pos_xyz[2] = T.transform.translation.z

            gps = get_gps(pos_xyz)

    return
# ----------  ----------  ----------  ----------  ----------







# Function that compute gps states given position states
def get_gps(pos):

    global LON_0, LAT_0, ALT_0, EARTH_RADIUS, RAD_TO_DEGREE

    gps_ = [0.0, 0.0, 0.0]

    gps_[0] = LON_0 + RAD_TO_DEGREE*pos[0]/EARTH_RADIUS
    gps_[1] = LAT_0 + RAD_TO_DEGREE*pos[1]/EARTH_RADIUS
    gps_[2] = ALT_0 + pos[2]

    return gps_
# ----------  ----------  ----------  ----------  ----------






# Rotina primaria
def imunode():
    global freq
    global accel, gyro
    global R, q, rpyi

    R = np.eye(3)
    q = np.array([0,0,0,1])
    rpy = np.array([[0],[0],[0]])

    imu_msg = Imu()
    imu_msg.header.frame_id = "/imu"
    #imu_msg.header.frame_id = "world"

    odom_msg = Odometry()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = "EspeleoRobo"

    gps_msg = NavSatFix()
    gps_msg.header.frame_id = "/imu"


    pub_imu = rospy.Publisher("/imu/raw", Imu, queue_size=1)
    pub_gps = rospy.Publisher("/fix", NavSatFix, queue_size=1)
    rospy.init_node("xsens_emulator")
    rospy.Subscriber("/sensors/acc", Point, callback_acc)
    rospy.Subscriber("/sensors/gyro", Point, callback_gyro)
    rospy.Subscriber("/tf", TFMessage, callback_pose_tf) # ground thruth - from tf transform

    rate = rospy.Rate(freq)

    sleep(0.5)
    print "\33[92mXsens emulator initialized!\33[0m"


    i = 0;
    while not rospy.is_shutdown():

        i = i + 1
        time = i / float(freq)

        #Publish imu data
        imu_msg.header.stamp = rospy.Time.now();

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.angular_velocity_covariance = [0.2, 0.1, 0.1, 0.1, 0.2, 0.1, 0.1, 0.1, 0.2]

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance = [0.2, 0.1, 0.1, 0.1, 0.2, 0.1, 0.1, 0.1, 0.2]

        pub_imu.publish(imu_msg)

        # ----------  ----------  ----------  ----------  ----------

        #Publish gps data
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.latitude = gps[1]
        gps_msg.longitude = gps[0]
        gps_msg.altitude = gps[2]

        pub_gps.publish(gps_msg)

        # ----------  ----------  ----------  ----------  ----------

        """
        # Create odom message
        odom_msg.header.stamp = rospy.Time.now();
        #odom_msg.pose.pose.position.x = p[0]
        #odom_msg.pose.pose.position.y = p[1]
        #odom_msg.pose.pose.position.z = p[2]
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        #Publish odom message
        pub_odom.publish(odom_msg)
        """

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------








# Funcao inicial
if __name__ == '__main__':

    global LON_0, LAT_0, ALT_0
    try:
        print "\33[93mLoad initial geolocation: successful\33[0m"
        LON_0 = float(sys.argv[2]) #degrees
        LAT_0 = float(sys.argv[1]) #degrees
        ALT_0 = float(sys.argv[3]) #meters
    except:
        LON_0 = -43.0 #degrees
        LAT_0 = -19.0 #degrees
        ALT_0 = 800.0 #meters
        print "\33[91mFailed to load parameters for initial geolocation !\33[0m"
        print "\33[93mUsing default:\nLAT_0 = -19.0\nLON_0 = -43.0\nALT_0 = 800.0\n\33[0m"


    global EARTH_RADIUS, RAD_TO_DEGREE
    EARTH_RADIUS = 6367444.5
    RAD_TO_DEGREE = 180.0/3.1415926535

    global g
    g = 9.81

    global freq
    freq = 50.0

    global accel, gyro, gps
    accel = [0, 0, -9.81]
    gyro = [0, 0, 0]
    gps = get_gps([0.0,0.0,0.0])




    try:
        imunode()
    except rospy.ROSInterruptException:
        pass
