#!/usr/bin/env python
import rospy

from assignment0.msg import TwoInt
from std_msgs.msg import Int8
from random import randint

def main():
    pub = rospy.Publisher('/numbers', TwoInt, queue_size=10)
    rospy.init_node('generator')
    rate = rospy.Rate(0.1)

    msg=TwoInt()
    while not rospy.is_shutdown():
      msg.num1 = randint(0,100)
      msg.num2 = randint(0,100)
      rospy.loginfo(msg)
      pub.publish(msg)
      rate.sleep

if __name__ == '__main__':
    try:
      main()
    except rospy.ROSInterruptException:
      pass


