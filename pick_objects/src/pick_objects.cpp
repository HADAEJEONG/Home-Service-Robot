#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal_point;
  move_base_msgs::MoveBaseGoal origin_point;  

  // set up the frame parameters
  goal_point.target_pose.header.frame_id = "map";
  goal_point.target_pose.header.stamp = ros::Time::now();
  origin_point.target_pose.header.frame_id = "map";
  origin_point.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal_point.target_pose.pose.position.x = 0.5;
  goal_point.target_pose.pose.position.y = 5.8;
  goal_point.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal_point");
  ac.sendGoal(goal_point);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("The robot has arrived at the goal point");
      sleep(5);
    }
  else
    ROS_INFO("The base failed to move to the goal point for some reason");

  origin_point.target_pose.pose.position.x = 0.1;
  origin_point.target_pose.pose.position.y = 3.0;
  origin_point.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending origin_point");
  ac.sendGoal(origin_point);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("The robot has arrived at the origin point");
      sleep(5);
    }
  else
    ROS_INFO("The base failed to move to the origin point for some reason");

  return 0;
}
