#!/usr/bin/env python
# -*- coding: utf-8 -*-

# smach
# https://qiita.com/srs/items/3f5acc64a2faac48b63d
# https://demura.net/education/lecture/21917.html
# http://wiki.ros.org/smach/Tutorials

import rospy
import smach
import smach_ros

from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class BasicRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemyFound','attackChance'])
        self.sakuteki_sub = rospy.Subscriber('sakuteki_result', String, self.sakutekiCallback)
        self.sakutekiResult = ""

    def sakutekiCallback(self, data):
        self.sakutekiResult = data

    def execute(self,userdata):
        rospy.loginfo('Executing state BasicRun')
        rospy.sleep(1.0)

        if data == "Front":
            return 'enemyFound'
        elif data == "Side":
            return 'attackChance'


class RunawayRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['attackChance','enemyLost'])
        self.sakuteki_sub = rospy.Subscriber('sakuteki_result', String, self.sakutekiCallback)
        self.sakutekiResult = ""

    def sakutekiCallback(self, data):
        self.sakutekiResult = data

    def execute(self,userdata):
        rospy.loginfo('Executing state RunawayRun')
        rospy.sleep(1.0)

        if data == "Front":
            return 'enemyFound'
        elif data == "Side":
            return 'attackChance'


class ChaseRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemyLost','enemyFound'])
        self.sakuteki_sub = rospy.Subscriber('sakuteki_result', String, self.sakutekiCallback)
        self.sakutekiResult = ""

    def execute(self,userdata):
        rospy.loginfo('Executing state ChaseRun')
        rospy.sleep(1.0)

        if data == "Front":
            return 'enemyFound'
        elif data == "Side":
            return 'attackChance'


class SakusenNode():
    def __init__(self):
        self.sakusen_pub = rospy.Publisher('sakusen_number', Int32, queue_size=1)

        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            smach.StateMachine.add('Basic', BasicRun(), transitions={'enemyFound':'Runaway', 'attackChance':'Chase'})
            smach.StateMachine.add('Runaway', RunawayRun(), transitions={'attackChance':'Chase', 'enemyLost':'Basic'})
            smach.StateMachine.add('Chase', ChaseRun(), transitions={'enemyLost':'Basic', 'enemyFound':'Runaway'})

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


