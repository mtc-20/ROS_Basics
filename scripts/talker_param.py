#!/usr/bin/env python
# HSRW Robotics, MTC 092019

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('talker', String, queue_size=10)
    rospy.init_node('talker_param', anonymous=True)
    freq=rospy.get_param("~freq",0.5)
    rate = rospy.Rate(freq) # Hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
