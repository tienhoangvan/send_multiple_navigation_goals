// This program is used to send the goal to the robot.
// The goal is in local map
#include <iostream>
#include <vector>
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
//
#define N_GOALS 2
#define PI 3.139601
#define PI2 1.569799
#define PI4 0.784899
//
ros::Publisher goal_pose_pub, goal_index_pub;
geometry_msgs::Pose2D new_goal_;
unsigned int counter_ = 0;
std_msgs::Int8 update_index_old;
geometry_msgs::Pose update_goal_pose_old;
//
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// N_GOALS x [x,y,w]
double_t landmarks_pose[N_GOALS][3] = {{4,3,0},{46,3,0}};// N_GOALS=2
//double_t landmarks_pose[N_GOALS][3] = {{0.0,0.0,0.0},{7.0,0.0,0.0},{7.5,22.0,1.57},{-12.0,22.0,3.14},{-12.0,-1.0,-1.57}};// N_GOALS=5
//double_t landmarks_pose[N_GOALS][3] = {{0.0,0.0,0.0},{7.0,0.0,0.0},{7.5,22.0,1.57},{-12.0,22.0,3.14},{-47.0,21.0,3.14},{-47.0,-2.0,-1.57},{-12.0,-1.0,0}};// N_GOALS=7
std::vector<geometry_msgs::Pose> landmarks(N_GOALS);
// init the goal pose
void initLandmarks()
{
    geometry_msgs::Pose goal_pose;
    for(unsigned int i =0; i<landmarks.size();i++)
    {
        goal_pose.position.x = landmarks_pose[i][0];
        goal_pose.position.y = landmarks_pose[i][1];
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,landmarks_pose[i][2]);
        goal_pose.orientation = quat;
        landmarks[i] = goal_pose;
    }
}
//
double convertQuaternionToAngle(geometry_msgs::Quaternion quat_in){
    tf::Quaternion quat;
    double dummy;
    double cur_ptheta;
    tf::quaternionMsgToTF(quat_in, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(dummy, dummy, cur_ptheta);

    return cur_ptheta;
}
//
void goalPoseVisualize(geometry_msgs::Pose goal_pose, int n_goal)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "goal_pose_" << n_goal;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = n_goal; // this number should be different
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //
    p.x = goal_pose.position.x;
    p.y = goal_pose.position.y;
    p.z = 0;
    marker.points.push_back(p);
    //
    double ang = convertQuaternionToAngle(goal_pose.orientation);
    p.x = goal_pose.position.x + 0.4*cos(ang);
    p.y = goal_pose.position.y + 0.4*sin(ang);
    p.z = 0;
    marker.points.push_back(p);
    goal_pose_pub.publish(marker);
    marker.points.clear();

}
//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "qbot_send_multi_goals_node");
  ros::NodeHandle nh;
  goal_pose_pub = nh.advertise<visualization_msgs::Marker>("send_goal_pose/goal_pose",0);
  goal_index_pub = nh.advertise<std_msgs::Int8>("/send_goal_pose/goal_index",100);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient action_goal("move_base", true);
  //
  initLandmarks();
  //wait for the action server to come up
  while(!action_goal.waitForServer(ros::Duration(10.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //
  //initLandmarks();
  //ROS_INFO("Number of goals: %d",landmarks.size());
  //
  unsigned int counter = 0;
  while(1)
  {
      if (counter>N_GOALS-1){
          counter=0;
      }

      move_base_msgs::MoveBaseGoal next_goal;
      geometry_msgs::Pose goal_pose;
      goal_pose = landmarks[counter];
      goalPoseVisualize(goal_pose, 1);
      std_msgs::Int8 goal_ind;
      goal_ind.data = counter;
      goal_index_pub.publish(goal_ind);

      //we'll send a goal to the robot to move 1 meter forward
      //goal.target_pose.header.frame_id = "base_link"; // use "base_link" this one for local map and "map" for global map
      next_goal.target_pose.header.frame_id = "map";
      next_goal.target_pose.header.stamp = ros::Time::now();
      next_goal.target_pose.pose = goal_pose;

      ROS_INFO("Sending goal [%d] (x, y, w) = (%f, %f, %f)",counter,next_goal.target_pose.pose.position.x,next_goal.target_pose.pose.position.y, next_goal.target_pose.pose.orientation.w);
      action_goal.sendGoal(next_goal);

      action_goal.waitForResult();

      if(action_goal.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("The robot aproached the goal [%d]",counter);
        //counter = counter + 1;
      }
      else
      {
        ROS_INFO("The robot failed to move to the goal for some reason");
      }
      /* Select the next landmark*/
      ros::Duration(1).sleep();// sleep 2 second

      counter++;
  }

  return 0;
}

