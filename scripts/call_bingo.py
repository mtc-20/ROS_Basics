#!/usr/bin/env python
# HSRW Robotics, MTC 092019

import rospy, random
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('bingo', String, queue_size=10)
    rospy.init_node('call_bingo', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        hello_str = "BINGO " + str(random.randint(1,101))
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
