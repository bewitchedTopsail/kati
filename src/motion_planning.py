#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from kati.msg import Trajectorie
from kati.msg import MotionSpec
from kati.msg import ImgInfo


## Class Definition CMotionPlaner
class CMotionPlaner:
    def __init__(self):
        self.motion_spec = ''
        self.pub = rospy.Publisher('trajectorie', Trajectorie, queue_size=10)

## Callback new Motion Specification
def callback_motion_spec(data, MotionPlaner):
    MotionPlaner.motion_spec = str(data.motionSpec)
    rospy.loginfo(rospy.get_caller_id() + " New MotionSpec =  %s", MotionPlaner.motion_spec)

## Callback new Img
def callback_new_img(data, MotionPlaner):
    msg = Trajectorie
    if MotionPlaner.motion_spec == 'follow_lane':
        msg.p1 = '1'
    else:
        msg.p1 = '2'
        rospy.logdebug(rospy.get_caller_id() + " No valid MotionSpec! MotionSpec = %s", MotionPlaner.motion_spec )
    MotionPlaner.pub.publish(msg)
    rospy.loginfo(rospy.get_caller_id() + " Path =  %s , Motion Spec =  %s", msg.p1, MotionPlaner.motion_spec)

## ROS Main loop
def motion_planning():
    MotionPlaner = CMotionPlaner()

    # Init
    rospy.init_node('motion_planning', anonymous=True)
    rospy.Subscriber('motion_spec',MotionSpec, callback_motion_spec, MotionPlaner)
    rospy.Subscriber('img_info',ImgInfo, callback_new_img, MotionPlaner)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

## Main function
if __name__ == '__main__':
    motion_planning()
