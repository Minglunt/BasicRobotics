#!/usr/bin/env python
import rospy

from std_msgs.msg import Int16
from assignment0.msg import TwoInt

result = Int16()

def callback(msg):
    result.data = msg.num1 + msg.num2
    rospy.loginfo(result)

def adder():
    rospy.init_node('adder')
    pub = rospy.Publisher("/sum", Int16, queue_size=10)
    sub = rospy.Subscriber("/numbers", TwoInt, callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
      pub.publish(result)
      rate.sleep()

if __name__=='__main__':

    try:
      adder()
    except rospy.ROSInterruptException:
      pass



