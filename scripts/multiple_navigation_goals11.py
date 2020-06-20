#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Create a action server
class WPServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                Waypoints,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
        elif msg.goal == 1:
            self._sas.set_aborted()
        elif msg.goal == 2:
            self._sas.set_preempted()

def main():
    rospy.init_node('waypoint_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm:
        # Add states to the container

    wp1 = MoveBaseGoal()
    wp1.target_pose.header.frame_id = '/map'
    wp1.target_pose.pose.position.x = 1
    wp1.target_pose.pose.position.y = 0
    wp1.target_pose.pose.orientation.w = 1.0
    StateMachine.add('Waypoint1',
                      SimpleActionState('WPServer',
                                        Waypoints,
                                        goal=wp1),
                      transitions={'succeeded':'Waypoint2'})

    wp2 = MoveBaseGoal()
    wp2.target_pose.header.frame_id = '/map'
    wp2.target_pose.pose.position.x = 1
    wp2.target_pose.pose.position.y = 0
    wp2.target_pose.pose.orientation.z = 0.70710678118
    wp2.target_pose.pose.orientation.w = 0.70710678118
    StateMachine.add('Waypoint2',
                      SimpleActionState('WPServer',
                                        Waypoints,
                                        goal=wp2),
                      transitions={'succeeded':'Waypoint3'})

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()