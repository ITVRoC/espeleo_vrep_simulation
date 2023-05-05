#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
from tf2_msgs.msg import TFMessage


"""
Universidade Federal de Minas Gerais (UFMG) - 2019
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
"""

"""
[Adriano] No futuro esse codigo pode ser colocado em um script lua no vrep
"""



# Callback to get the pose of the robot
def callback_pose(data):
    global esp_p, esp_q

    for T in data.transforms:
        # Chose the transform of the EspeleoRobo
        if (T.child_frame_id == "EspeleoRobo"):

            x_n = T.transform.translation.x
            y_n = T.transform.translation.y
            z_n = T.transform.translation.z
            x_q = T.transform.rotation.x
            y_q = T.transform.rotation.y
            z_q = T.transform.rotation.z
            w_q = T.transform.rotation.w

            esp_p = [x_n, y_n, z_n]
            esp_q = [x_q, y_q, z_q, w_q]

    return
# ----------  ----------  ----------  ----------  ----------









# Rotina para piblicar informacoes no rviz
def send_espeleo_to_rviz(esp_p, esp_q):

    global pub_rviz_espeleo

    mark = Marker()

    mark.header.frame_id = "/world"
    mark.header.stamp = rospy.Time.now()
    mark.id = 0
    mark.type = mark.CUBE
    mark.action = mark.ADD
    # Size of espeleo
    mark.scale.x = 0.50
    mark.scale.y = 0.30
    mark.scale.z = 0.12
    # Yellowish color
    mark.color.a = 0.9
    mark.color.r = 0.9
    mark.color.g = 0.9
    mark.color.b = 0.0
    # Ground thruth position
    mark.pose.position.x = esp_p[0]
    mark.pose.position.y = esp_p[1]
    mark.pose.position.z = esp_p[2]
    # Ground thruth orientation
    quaternio = [esp_q[0], esp_q[1], esp_q[2], esp_q[3]]
    mark.pose.orientation.x = quaternio[0]
    mark.pose.orientation.y = quaternio[1]
    mark.pose.orientation.z = quaternio[2]
    mark.pose.orientation.w = quaternio[3]

    # Publish marker
    pub_rviz_espeleo.publish(mark)

    return

# ----------  ----------  ----------  ----------  ----------








# Rotina primaria
def configer():
    global freq
    global pub_rviz_cylinder, pub_rviz_espeleo
    global esp_p, esp_q


    rospy.init_node("config_rviz")
    rospy.Subscriber("/tf", TFMessage, callback_pose)
    pub_rviz_espeleo = rospy.Publisher("/visualization_marker_espeleo", Marker, queue_size=1) #rviz marcador de velocidade de referencia

    rate = rospy.Rate(freq)


    sleep(1)

    while not rospy.is_shutdown():

        #Send block to rviz - representing the espeleo robo
        send_espeleo_to_rviz(esp_p,esp_q)

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

    # Program frequence
    global freq
    freq = 10.0

    # Espeleo pose
    global esp_p, esp_q
    esp_p = [0.0, 0.0, 0.0]
    esp_q = [0.0, 0.0, 0.0, 0.0]

    try:
        configer()
    except rospy.ROSInterruptException:
        pass
