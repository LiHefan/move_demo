#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import smach
import smach_ros
from move_demo.srv import *


class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['toAbove'])
        print("Waiting for service AddObject.")
        rospy.wait_for_service('add_object')
        self.adding_object = rospy.ServiceProxy('add_object',AddObject)
    def execute(self,ud):
        rospy.loginfo('Executing state ADD')
        self.adding_object()
        return 'toAbove'

class Above(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['toEdge','toCircle'],
                             input_keys=['strategy'])
        print("Waiting for service MoveAbove.")
        rospy.wait_for_service('move_above')
        self.moving_above = rospy.ServiceProxy('move_above',MoveAbove)
    def execute(self,ud):
        rospy.loginfo('Executing state ABOVE')
        self.moving_above()
        if  ud.strategy == 0:
            return 'toEdge'
        else:
            return 'toCircle'

class Edge(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['toContour'])
        print("Waiting for service MoveToEdge.")
        rospy.wait_for_service('move_to_edge')
        self.moving_to_edge = rospy.ServiceProxy('move_to_edge',MoveToEdge)
    def execute(self,ud):
        rospy.loginfo('Executing state EDGE')
        self.moving_to_edge()
        return 'toContour'

class Contour(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['contourDone'])
        print("Waiting for service MoveAlongContour.")
        rospy.wait_for_service('move_along_contour')
        self.moving_along_contour = rospy.ServiceProxy('move_along_contour',MoveAlongContour)
    def execute(self,ud):
        rospy.loginfo('Executing state CONTOUR')
        self.moving_along_contour()
        return 'contourDone'

class Fixed_Circle(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['circleDone'])
        print("Waiting for service DrawCircle.")
        rospy.wait_for_service('draw_circle')
        self.drawing_circle = rospy.ServiceProxy('draw_circle',DrawCircle)
    def execute(self,ud):
        rospy.loginfo('Executing state CIRCLE')
        self.drawing_circle()
        return 'circleDone'


       
def main():
    rospy.init_node('move_demo_state_machine')
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.sensor_strategy = 1
    sm.userdata.detection = 1
    # sm.userdata.position_x = 0
    # sm.userdata.position_y = 0
    # sm.userdata.position_z = 0


    with sm:
        smach.StateMachine.add('HOME',Home(),
                                transitions={'toAbove':'ABOVE'})
        smach.StateMachine.add('ABOVE',Above(),
                                transitions={'toEdge':'EDGE','toCircle':'FIXED_CIRCLE'},
                                remapping={'strategy':'sensor_strategy'})
        smach.StateMachine.add('EDGE',Edge(),
                                transitions={'toContour':'CONTOUR'})
        smach.StateMachine.add('CONTOUR',Contour(),
                                transitions={'contourDone':'finished'})
        smach.StateMachine.add('FIXED_CIRCLE',Fixed_Circle(),
                                transitions={'circleDone':'finished'})

   
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('exploration', sm, '/EXPLORE')
    sis.start()
    # Execute the state machine
    outcome = sm.execute() 
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()








    






