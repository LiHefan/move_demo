#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import smach
from move_demo.srv import *


class Add(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['add'],input_keys=['add_in'],output_keys=['add_out'])
        print("Waiting for service AddObject.")
        rospy.wait_for_service('add_object')
        self.adding_object = rospy.ServiceProxy('add_object',AddObject)
    def execute(self,ud):
        rospy.loginfo('Executing state ADD')
        self.adding_object()
        ud.add_out = ud.add_in + 1
        return 'add'

class Above(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['above'],
                             input_keys=['above_in'],
                             output_keys=['above_out'])
        print("Waiting for service MoveAbove.")
        rospy.wait_for_service('move_above')
        self.moving_above = rospy.ServiceProxy('move_above',MoveAbove)
    def execute(self,ud):
        rospy.loginfo('Executing state ABOVE')
        self.moving_above()
        ud.above_out = ud.above_in + 1
        return 'above'

class Edge(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['edge'],
                             input_keys=['edge_in'],
                             output_keys=['edge_out'])
        print("Waiting for service MoveToEdge.")
        rospy.wait_for_service('move_to_edge')
        self.moving_to_edge = rospy.ServiceProxy('move_to_edge',MoveToEdge)
    def execute(self,ud):
        rospy.loginfo('Executing state EDGE')
        self.moving_to_edge()
        ud.edge_out = ud.edge_in + 1
        return 'edge'

class Contour(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['contour'],
                             input_keys=['contour_in'],
                             output_keys=['contour_out'])
        print("Waiting for service MoveAlongContour.")
        rospy.wait_for_service('move_along_contour')
        self.moving_along_contour = rospy.ServiceProxy('move_along_contour',MoveAlongContour)
    def execute(self,ud):
        rospy.loginfo('Executing state CONTOUR')
        self.moving_along_contour()
        ud.contour_out = ud.contour_in + 1
        return 'contour'

       
def main():
    rospy.init_node('move_demo_state_machine')
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.step = 0
    # sm.userdata.position_x = 0
    # sm.userdata.position_y = 0
    # sm.userdata.position_z = 0


    with sm:
        smach.StateMachine.add('ADD',Add(),
                                transitions={'add':'ABOVE'},
                                remapping={'add_in':'step','add_out':'step'})
        smach.StateMachine.add('ABOVE',Above(),
                                transitions={'above':'EDGE'},
                                remapping={'above_in':'step','above_out':'step'})
        smach.StateMachine.add('EDGE',Edge(),
                                transitions={'edge':'CONTOUR'},
                                remapping={'edge_in':'step','edge_out':'step'})
        smach.StateMachine.add('CONTOUR',Contour(),
                                transitions={'contour':'finished'},
                                remapping={'contour_in':'step','contour_out':'step'})

   
    outcome = sm.execute()
    print("**************")
    print(sm.userdata.step)
if __name__ == "__main__":
    main()








    






