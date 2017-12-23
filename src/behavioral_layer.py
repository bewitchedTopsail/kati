#!/usr/bin/env python

import rospy
from kati.msg import MotionSpec

def behavioral_layer():
    pub = rospy.Publisher('motion_spec', MotionSpec, queue_size=10)
    rospy.init_node('behavioral_layer', anonymous=True)
    rate = rospy.Rate(0.16) # 1hz
    while not rospy.is_shutdown():
        msg = MotionSpec()
        msg.motionSpec = 'follow_lane'
        rospy.loginfo(msg.motionSpec)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        behavioral_layer()
    except rospy.ROSInterruptException:
        pass


