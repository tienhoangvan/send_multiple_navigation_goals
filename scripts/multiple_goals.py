#!/usr/bin/env python
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

            rospy.init_node('send_multiple_goals')
        
            # The list of goals
            p_1 = [1.8, 0, -math.pi/2]
            pv_1 = [0, -1.8, -math.pi]
            p_2 = [-1.8, 0, math.pi/2]
            pv_2 = [0, 1.8, 0]

            goal_list = [pv_1, p_2, pv_2, p_1]
            # The list of goal poses:
            self.pose_seq = goal_list
            # the index of a goal in the goal list
            self.goal_cnt = 0
            # the list of goal type

             #Create action client
            self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            rospy.loginfo("Waiting for move_base action server...")
            wait = self.client.wait_for_server()
            rospy.loginfo("Connected to move base server")
            rospy.loginfo("Starting goals achievements ...")
            self.movebase_client()

    def active_cb(self):
            rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
            #To print current pose at each feedback:
            #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
            rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")      

    def done_cb(self, status, result):
            # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
            if status == 2:
                  rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

            if status == 3:             
                  self.goal_cnt += 1
                  rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
                  if self.goal_cnt< len(self.pose_seq):
                        next_goal = MoveBaseGoal()
                        next_goal.target_pose.header.frame_id = "map"
                        next_goal.target_pose.header.stamp = rospy.Time.now()

                        goal_pose = Pose()
                        point = Point()
                        quat = Quaternion()

                        point.x = self.pose_seq[self.goal_cnt][0]
                        point.y = self.pose_seq[self.goal_cnt][1]
                        goal_pose.position = Point(point.x, point.y, 0)
                
                        quat = quaternion_from_euler(0.0, 0.0, self.pose_seq[self.goal_cnt][2])
                        goal_pose.orientation = Quaternion(*quat)

                        next_goal.target_pose.pose = goal_pose

                        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                        rospy.loginfo(str(goal_pose))
                        self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
                  else:
                        rospy.loginfo("Final goal pose reached!")
                        rospy.signal_shutdown("Final goal pose reached!")
                        return

            if status == 4:
                  rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
                  rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
                  return

            if status == 5:
                  rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
                  rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
                  return

            if status == 8:
                  rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal_pose = Pose()
        point = Point()
        quat = Quaternion()
        
        point.x = self.pose_seq[self.goal_cnt][0]
        point.y = self.pose_seq[self.goal_cnt][1]
        goal_pose.position = Point(point.x, point.y, 0)
        
        quat = quaternion_from_euler(0.0, 0.0, self.pose_seq[self.goal_cnt][2])
        goal_pose.orientation = Quaternion(*quat)
        
        goal.target_pose.pose = goal_pose

        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(goal_pose))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
          rospy.loginfo("Navigation finished.")