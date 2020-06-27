#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def set_pose():
    rospy.init_node('set_initial_pose')
    # Set a pose
    pub_set_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # the list of set poses
    set_pose_0 = [0.5, 0.5, 0]
    set_pose_1 = [1.8, 0, -math.pi/2]

    set_pose_list = [set_pose_0, set_pose_1]
    num_set_pose = len(set_pose_list)

    sp_id = 0

    set_pose = PoseWithCovarianceStamped()
    quat = Quaternion()

    set_pose.header.frame_id = "map"
    set_pose.header.stamp = rospy.Time.now()

    set_pose.pose.pose.position.x = set_pose_list[sp_id][0]
    set_pose.pose.pose.position.y = set_pose_list[sp_id][0]
    
    quat = quaternion_from_euler(0.0, 0.0, set_pose_list[sp_id][2])
    set_pose.pose.pose.orientation = Quaternion(*quat)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        set_pose.pose.pose.position.x = set_pose_list[sp_id][0]
        set_pose.pose.pose.position.y = set_pose_list[sp_id][0]
    
        quat = quaternion_from_euler(0.0, 0.0, set_pose_list[sp_id][2])
        set_pose.pose.pose.orientation = Quaternion(*quat)
 
        pub_set_pose.publish(set_pose)
        rospy.loginfo("Set Pose:" + str(set_pose))
        rate.sleep()

if __name__ == '__main__':
    try:
        set_pose()
    except rospy.ROSInterruptException:
        pass
