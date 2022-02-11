#!/usr/bin/env python
# -*- coding: utf-8 -*-

# smach
# https://qiita.com/srs/items/3f5acc64a2faac48b63d

import rospy
import smach
import smach_ros

from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class AutoRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemyFound','attackChance'])

    def execute(self,userdata):
        rospy.loginfo('Executing state AutoRun')
        rospy.sleep(2.0)
        return 'enemyFound'


class EscapeRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['attackChance','enemyLost'])

    def execute(self,userdata):
        rospy.loginfo('Executing state EscapeRun')
        return 'attackChance'


class AttackRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemyLost','enemyFound'])

    def execute(self,userdata):
        rospy.loginfo('Executing state AttackRun')
        return 'enemyLost'


class SakusenNode():
    def __init__(self):
        self.sakusen_pub = rospy.Publisher('sakusen_number', Int32, queue_size=10)

        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            smach.StateMachine.add('Auto', AutoRun(), transitions={'enemyFound':'Escape', 'attackChance':'Attack'})
            smach.StateMachine.add('Escape', EscapeRun(), transitions={'attackChance':'Attack', 'enemyLost':'Auto'})
            smach.StateMachine.add('Attack', AttackRun(), transitions={'enemyLost':'Auto', 'enemyFound':'Escape'})

        sis = smach_ros.IntrospectionServer('sakusen_server', sm_top, '/SM_TOP')
        sis.start()
        outcome = sm_top.execute()
        sis.stop()
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


