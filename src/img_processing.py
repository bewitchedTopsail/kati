#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from kati.msg import ImgInfo
import cv2
import numpy as np


def callback(msg,pub):
    rospy.loginfo(rospy.get_caller_id() + 'I heard somethin')
    image = np.fromstring(msg.data, np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(gray)

    msg = ImgInfo()
    msg.p1 = 1
    pub.publish(msg)
    #cv2.imshow('Image',equ)
    # cv2.waitKey(0)

def listener():
    rospy.init_node('img_processing', anonymous=True)
    pub = rospy.Publisher('img_info', ImgInfo, queue_size=10)

    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, callback,pub)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
