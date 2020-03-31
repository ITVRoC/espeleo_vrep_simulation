#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
from tf2_msgs.msg import TFMessage




global freq
freq = 10.0

global esp_p, esp_q
esp_p = [0.0, 0.0, 0.0]
esp_q = [0.0, 0.0, 0.0, 0.0]


# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global esp_p, esp_q


    #print data

    # print "esp_p: ", esp_p
    # print "esp_q: ", esp_q, "\n"


    if (data.transforms[0].child_frame_id == "EspeleoRobo"):

        x_n = data.transforms[0].transform.translation.x  # posicao 'x' do robo no mundo
        y_n = data.transforms[0].transform.translation.y  # posicao 'y' do robo no mundo
        z_n = data.transforms[0].transform.translation.z  # posicao 'z' do robo no mundo
        x_q = data.transforms[0].transform.rotation.x
        y_q = data.transforms[0].transform.rotation.y
        z_q = data.transforms[0].transform.rotation.z
        w_q = data.transforms[0].transform.rotation.w
        #euler = euler_from_quaternion([x_q, y_q, z_q, w_q])

        esp_p = [x_n, y_n, z_n]
        esp_q = [x_q, y_q, z_q, w_q]





        # br1 = tf.TransformBroadcaster()
        # phi = 0
        # #br1.sendTransform((3*0, 0, 0), (sin(phi/2), sin(phi/2), sin(phi/2), cos(phi/2)), rospy.Time.now(), "/EspeleoRobo_2", "EspeleoRobo")
        # br1.sendTransform((3*0, 0, 0), (0.0, 0.7071, 0.0, 0.7071), rospy.Time.now(), "/EspeleoRobo_2", "EspeleoRobo")
        # br2 = tf.TransformBroadcaster()
        # br2.sendTransform((x_n, y_n, z_n), (x_q, y_q, z_q, w_q), rospy.Time.now(), "/EspeleoRobo", "world")

    return


# ----------  ----------  ----------  ----------  ----------









# Rotina executada apenas uma vez para mostrar a ellipse no rviz
def send_ellipse_to_rviz():
    global a, b, cx, cy, phi

    points_marker = MarkerArray()
    marker = Marker()
    for p0 in range(1, 628, 1):
        print "p0 = ", p0
        p = p0 / 100.0
        x = cos(phi) * (a * cos(p)) - sin(phi) * (b * sin(p)) + cx * 1
        y = sin(phi) * (a * cos(p)) + cos(phi) * (b * sin(p)) + cy * 1
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = p0 - 1
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        print "marker = ", marker
        points_marker.markers.append(marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------




# Rotina para piblicar informacoes no rviz
def send_marker_to_rviz(x_ref, y_ref, Vx_ref, Vy_ref, Ux, Uy):
    global x_n, y_n, theta_n
    global x_goal, y_goal
    global pub_rviz_ref, pub_rviz_pose

    mark_ref = Marker()
    mark_pose = Marker()

    mark_ref.header.frame_id = "/world"
    mark_ref.header.stamp = rospy.Time.now()
    mark_ref.id = 0
    mark_ref.type = mark_ref.ARROW
    mark_ref.action = mark_ref.ADD
    mark_ref.scale.x = 0.5 * (Vy_ref ** 2 + Vx_ref ** 2) ** (0.5)
    mark_ref.scale.y = 0.1
    mark_ref.scale.z = 0.1
    mark_ref.color.a = 1.0
    mark_ref.color.r = 1.0
    mark_ref.color.g = 1.0
    mark_ref.color.b = 1.0
    mark_ref.pose.position.x = x_ref
    mark_ref.pose.position.y = y_ref
    mark_ref.pose.position.z = 0.0
    quaternio = quaternion_from_euler(0, 0, atan2(Vy_ref, Vx_ref))
    mark_ref.pose.orientation.x = quaternio[0]
    mark_ref.pose.orientation.y = quaternio[1]
    mark_ref.pose.orientation.z = quaternio[2]
    mark_ref.pose.orientation.w = quaternio[3]

    mark_pose.header.frame_id = "/world"
    mark_pose.header.stamp = rospy.Time.now()
    mark_pose.id = 1
    mark_pose.type = mark_pose.ARROW
    mark_pose.action = mark_pose.ADD
    mark_pose.scale.x = 0.5 * sqrt(Ux ** 2 + Uy ** 2)
    mark_pose.scale.y = 0.1
    mark_pose.scale.z = 0.1
    mark_pose.color.a = 1.0
    mark_pose.color.r = 1.0
    mark_pose.color.g = 0.0
    mark_pose.color.b = 0.0
    mark_pose.pose.position.x = x_n
    mark_pose.pose.position.y = y_n
    mark_pose.pose.position.z = 0.1
    quaternio = quaternion_from_euler(0, 0, theta_n)
    mark_pose.pose.orientation.x = quaternio[0]
    mark_pose.pose.orientation.y = quaternio[1]
    mark_pose.pose.orientation.z = quaternio[2]
    mark_pose.pose.orientation.w = quaternio[3]

    pub_rviz_ref.publish(mark_ref)
    pub_rviz_pose.publish(mark_pose)

    return

# ----------  ----------  ----------  ----------  ----------





# Rotina para piblicar informacoes no rviz
def send_cylinder_to_rviz():

    global pub_rviz_cylinder

    mark = Marker()

    mark.header.frame_id = "/world"
    mark.header.stamp = rospy.Time.now()
    mark.id = 0
    mark.type = mark.CYLINDER
    mark.action = mark.ADD
    mark.scale.x = 0.7
    mark.scale.y = 0.7
    mark.scale.z = 10.0
    mark.color.a = 0.8
    mark.color.r = 0.6
    mark.color.g = 0.6
    mark.color.b = 0.6
    mark.pose.position.x = 0.0
    mark.pose.position.y = 0.0
    mark.pose.position.z = 0.7/2.0
    quaternio = [0.0, 0.7071, 0.0, 0.7071]
    mark.pose.orientation.x = quaternio[0]
    mark.pose.orientation.y = quaternio[1]
    mark.pose.orientation.z = quaternio[2]
    mark.pose.orientation.w = quaternio[3]


    pub_rviz_cylinder.publish(mark)

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
    mark.scale.x = 0.50
    mark.scale.y = 0.30
    mark.scale.z = 0.12
    mark.color.a = 0.9
    mark.color.r = 0.9
    mark.color.g = 0.9
    mark.color.b = 0.0
    mark.pose.position.x = esp_p[0]
    mark.pose.position.y = esp_p[1]
    mark.pose.position.z = esp_p[2]
    quaternio = [esp_q[0], esp_q[1], esp_q[2], esp_q[3]]
    mark.pose.orientation.x = quaternio[0]
    mark.pose.orientation.y = quaternio[1]
    mark.pose.orientation.z = quaternio[2]
    mark.pose.orientation.w = quaternio[3]


    pub_rviz_espeleo.publish(mark)

    return

# ----------  ----------  ----------  ----------  ----------








# Rotina primaria
def configer():
    global freq
    global pub_rviz_cylinder, pub_rviz_espeleo
    global esp_p, esp_q


    # pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.init_node("ellipse")
    rospy.Subscriber("/tf", TFMessage, callback_pose)

    pub_rviz_cylinder = rospy.Publisher("/visualization_marker_cylinder", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    pub_rviz_espeleo = rospy.Publisher("/visualization_marker_espeleo", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    #pub_rviz_pose = rospy.Publisher("/visualization_marker_pose", Marker, queue_size=1) #rviz marcador de velocidade do robo
    #pub_rviz_ellipse = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1) #rviz array de marcadores no espaco da elipse

    rate = rospy.Rate(freq)

    #pointsMarker = send_ellipse_to_rviz()

    # send_cylinder_to_rviz()

    sleep(1)

    while not rospy.is_shutdown():

        send_cylinder_to_rviz()
        send_espeleo_to_rviz(esp_p,esp_q)



        # send_marker_to_rviz(x_ref, y_ref, Vx_ref, Vy_ref, Ux, Uy)

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

    try:
        configer()
    except rospy.ROSInterruptException:
        pass
