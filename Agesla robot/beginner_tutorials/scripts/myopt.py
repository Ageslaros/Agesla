#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' msg is %s', data.data)
   
def listener():


    rospy.init_node('myopt', anonymous=True)
    rospy.Subscriber('chatter',String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
