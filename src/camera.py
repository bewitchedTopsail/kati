#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def camera():
    pub = rospy.Publisher('img', String, queue_size=10)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        img  = "1"
        pub.publish(img)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
