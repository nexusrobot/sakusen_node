#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/


class SakusenNode():
    def __init__(self):
        self.sakusen_pub = rospy.Publisher('sakusen_number', Int32, queue_size=10)

        self.count = 0

    def run(self):
        rospy.Rate(1000)
        self.count += 1
        self.sakusen_pub.publish(self.count);


if __name__ == '__main__':
    try:
        rospy.init_node('SakusenNode', anonymous=False)
        node = SakusenNode()

        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            node.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


