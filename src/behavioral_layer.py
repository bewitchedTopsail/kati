#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def behavioral_layer():
    pub = rospy.Publisher('motion_spec', String, queue_size=10)
    rospy.init_node('behavioral_layer', anonymous=True)
    rate = rospy.Rate(0.16) # 1hz
    while not rospy.is_shutdown():
        motion_spec = 'follow_lane'
        rospy.loginfo(motion_spec)
        pub.publish(motion_spec)
        rate.sleep()

if __name__ == '__main__':
    try:
        behavioral_layer()
    except rospy.ROSInterruptException:
        pass



# Maybe use state mache (Library smach) to in the behaviroal_layer
# Then you can grab the acutal state (follow_lan, turn_left) with a service every time a new image arrives
