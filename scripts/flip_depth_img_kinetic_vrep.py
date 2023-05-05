#!/usr/bin/python3

#from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
    def __init__(self):
        rospy.init_node('image_converter_flip_depth', anonymous=True)

        self.pub_color = rospy.Publisher("/camera/color_flip", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("/camera/depth_flip", Image, queue_size=1)
        self.bridge = CvBridge()

        self.sub_color = rospy.Subscriber("/camera/color", Image, self.callback_color)
        self.sub_depth = rospy.Subscriber("/camera/depth", Image, self.callback_depth)

    def callback_color(self, data, encoding="rgb8"):
        """
        """
        flipped_img = self.flip_img(data)
        if flipped_img is None:
            rospy.logerr("Flipped color img is None")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(flipped_img, encoding)
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "d435i_sim_link"
            self.pub_color.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def callback_depth(self, data, encoding="16UC1"):
        """
        """
        flipped_img = self.flip_img(data, encoding)
        if flipped_img is None:
            rospy.logerr("Flipped color img is None")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(flipped_img, encoding)
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "d435i_sim_link"
            self.pub_depth.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def flip_img(self, data, encoding="rgb8"):
        """
        Flip img horizontally
        """
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, encoding)
            cv_image = cv2.flip(cv_image, 1)
        except CvBridgeError as e:
            print(e)

        return cv_image


def main(args):
    ic = image_converter()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")

    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
