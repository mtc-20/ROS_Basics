#!/usr/bin/env python
'''
HSRW Robotics 
MTC 09/2019
'''

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": I heard %s", data.data)
    print "I heard [%s]"%data.data
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("talker", String, callback)
    rospy.Subscriber("bingo", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
