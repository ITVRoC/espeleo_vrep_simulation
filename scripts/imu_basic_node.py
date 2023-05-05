#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, tan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
# from visualization_msgs.msg import Marker, MarkerArray
# import tf
# from tf2_msgs.msg import TFMessage
import numpy as np


"""
Universidade Federal de Minas Gerais (UFMG) - 2019
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



# Compute the jacobian of representation
def Jacobian_of_reprsentation(rpy):

    Jr = np.array([[1, sin(rpy[0])*tan(rpy[1]), cos(rpy[0])*tan(rpy[1])],
                    [0, cos(rpy[0]), -sin(rpy[0])],
                    [0, sin(rpy[0])/cos(rpy[1]), cos(rpy[0])/cos(rpy[1])]])

    return Jr
# ----------  ----------  ----------  ----------  ----------


# Filter for orientation from imu
def filter(R, q, rpy, accel, gyro):

    global freq

    #print "rpy = \n", rpy, "\n"

    Jr = Jacobian_of_reprsentation(rpy)

    rpy = rpy + np.dot(Jr,np.array([[gyro[0]],[gyro[1]],[gyro[2]]]))*(1.0/freq)

    inov_phi = -sin(atan2(accel[1], accel[2]) - rpy[0])
    inov_theta = sin(asin(accel[0]/g) - rpy[1])

    rpy[0] = rpy[0] + 0.1*inov_phi
    rpy[1] = rpy[1] + 0.1*inov_theta


    q = np.array(quaternion_from_euler(float(rpy[0]),float(rpy[1]),float(rpy[2])))

    rpy_temp = euler_from_quaternion(q.tolist())
    rpy = np.array([[rpy_temp[0]],[rpy_temp[1]],[rpy_temp[2]]])

    return (R, q, rpy)
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
    imu_msg.header.frame_id = "EspeleoRobo"
    #imu_msg.header.frame_id = "world"

    odom_msg = Odometry()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = "EspeleoRobo"


    pub_imu = rospy.Publisher("/espeleo/imu_raw", Imu, queue_size=1)
    #pub_imu = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
    #pub_odom = rospy.Publisher("/espeleo/odom", Odometry, queue_size=1)
    rospy.init_node("ellipse")
    rospy.Subscriber("/sensors/acc", Point, callback_acc)
    rospy.Subscriber("/sensors/gyro", Point, callback_gyro)

    # pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    # pub_rviz_pose = rospy.Publisher("/visualization_marker_pose", Marker, queue_size=1) #rviz marcador de velocidade do robo
    # pub_rviz_ellipse = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1) #rviz array de marcadores no espaco da elipse

    rate = rospy.Rate(freq)

    #pointsMarker = send_ellipse_to_rviz()

    sleep(0.5)
    print ("IMU initialized!")


    i = 0;
    while not rospy.is_shutdown():

        i = i + 1
        time = i / float(freq)

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

    global g
    g = 9.81

    global freq
    freq = 20.0

    global accel, gyro
    accel = [0, 0, -9.81]
    gyro = [0, 0, 0]

    try:
        imunode()
    except rospy.ROSInterruptException:
        pass
