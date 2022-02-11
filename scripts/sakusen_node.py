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
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class BasicRun(smach.State):
    def __init__(self):
        #smach.State.__init__(self, outcomes=['enemyFound','attackChance','continue'])
        smach.State.__init__(self, outcomes=['enemyFound','attackChance'])
        #self.sakuteki_sub = rospy.Subscriber('sakuteki_result', String, self.sakutekiCallback)
        self.sakuteki_sub = rospy.Subscriber('test_topic', Int32, self.sakutekiCallback)
        self.sakutekiResult = ""
        self.count = 0

    def sakutekiCallback(self, data):
        rospy.loginfo('sakuteki_result received {}'.format(data))
        self.sakutekiResult = data

    def execute(self,userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('Executing state BasicRun')
            rospy.sleep(1.0)

            print self.sakutekiResult
            if self.count > 10:
                return 'enemyFound'
            else:
                self.count+=1
#            if self.sakutekiResult == "Front":
#                print "enemy found" 
#                return 'enemyFound'
#            elif self.sakutekiResult == "Side":
#                return 'attackChance'
        return 'continue'


class RunawayRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['attackChance','enemyLost','continue'])
        self.sakuteki_sub = rospy.Subscriber('sakuteki_result', String, self.sakutekiCallback)
        self.sakutekiResult = ""

    def sakutekiCallback(self, data):
        rospy.loginfo('sakuteki_result received {}'.format(data))
        self.sakutekiResult = data

    def execute(self,userdata):
        rospy.loginfo('Executing state RunawayRun')
        rospy.sleep(1.0)

        if self.sakutekiResult == "Front":
            return 'enemyFound'
        elif self.sakutekiResult == "Side":
            return 'attackChance'
        return 'continue'


class ChaseRun(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemyLost','enemyFound','continue'])
        self.sakuteki_sub = rospy.Subscriber('sakuteki_result', String, self.sakutekiCallback)
        self.sakutekiResult = ""

    def sakutekiCallback(self, data):
        rospy.loginfo('sakuteki_result received {}'.format(data))
        self.sakutekiResult = data

    def execute(self,userdata):
        rospy.loginfo('Executing state ChaseRun')
        rospy.sleep(1.0)

        if self.sakutekiResult == "Front":
            return 'enemyFound'
        elif self.sakutekiResult == "Side":
            return 'attackChance'
        return 'continue'


class SakusenNode():
    def __init__(self):
        self.sakusen_pub = rospy.Publisher('sakusen_number', Int32, queue_size=1)

        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            #smach.StateMachine.add('Basic', BasicRun(), transitions={'enemyFound':'Runaway', 'attackChance':'Chase', 'continue':'Basic'})
            smach.StateMachine.add('Basic', BasicRun(), transitions={'enemyFound':'Runaway', 'attackChance':'Chase'})
            smach.StateMachine.add('Runaway', RunawayRun(), transitions={'attackChance':'Chase', 'enemyLost':'Basic', 'continue':'Runaway'})
            smach.StateMachine.add('Chase', ChaseRun(), transitions={'enemyLost':'Basic', 'enemyFound':'Runaway', 'continue':'Chase'})

        sis = smach_ros.IntrospectionServer('sakusen_server', sm_top, '/SM_TOP')
        sis.start()
        outcome = sm_top.execute()
        sis.stop()
        rospy.signal_shutdown('All done.')

#        self.count = 0
#    def run(self):
#        self.count += 1
#        self.sakusen_pub.publish(self.count);


if __name__ == '__main__':
    rospy.init_node('SakusenNode', anonymous=False)
    node = SakusenNode()
    node.run()

#    try:
#        rospy.init_node('SakusenNode', anonymous=False)
#        node = SakusenNode()
#
#        rate = rospy.Rate(1) # 10hz
#        while not rospy.is_shutdown():
#            node.run()
#            rate.sleep()
#    except rospy.ROSInterruptException:
#        pass


