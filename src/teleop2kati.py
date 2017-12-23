#!/usr/bin/env python


import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kati.msg import SteeringCmds
from threading import Timer

pub = rospy.Publisher('steering_cmds', SteeringCmds, queue_size=10)

def timeout():
    global pub
    msg = SteeringCmds()
    msg.throttle = 0
    msg.curvature = 0
    pub.publish(msg)
    print "ALARM"

def callback(data):
    global pub
    global timer
    timer.cancel()
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s %s', data.linear, data.angular)
    msg = SteeringCmds()
    msg.throttle = 5*data.linear.x
    msg.curvature = data.angular.z
    print "Curv: " + str(msg.curvature)
    pub.publish(msg)
    timer = Timer(1, timeout)
    timer.start()

def listener():
    global timer
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('turtle1/cmd_vel', Twist, callback)
    timer = Timer(1, timeout)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
