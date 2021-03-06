#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
from std_msgs.msg import Int32
pub = rospy.Publisher('sender', Int32, queue_size=10)
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Distance is %s', data.data)
    my_msg = int(data.data)
    pub.publish(my_msg)
    rate = rospy.Rate(100) # 10hz
    rate.sleep()
def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('sonar', Int32, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()



