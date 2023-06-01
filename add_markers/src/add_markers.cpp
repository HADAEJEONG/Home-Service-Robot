#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

double poseAMCLx, poseAMCLy, poseAMCLw;
visualization_msgs::Marker goal_marker;
visualization_msgs::Marker origin_marker;
double goal_flag, origin_flag;
double goalx=0.5, goaly=5.8, goalw=1.0;
double originx=0.1, originy=3.0, originw=1.0;

void carry_marker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy= msgAMCL->pose.pose.position.y;
    poseAMCLw = msgAMCL->pose.pose.orientation.w;
    
    double diff_goalx = fabs(poseAMCLx - goalx);
    double diff_goaly = fabs(poseAMCLy - goaly);
    double diff_goalw = fabs(poseAMCLw - goalw);

    double diff_originx = fabs(poseAMCLx - originx);
    double diff_originy = fabs(poseAMCLy - originy);
    double diff_originw = fabs(poseAMCLw - originw);

    if(goal_flag == 0)
    {
    ROS_INFO("diff_goalX:%f, diff_goalY:%f, diff_goalW:%f", diff_goalx,diff_goaly,diff_goalw);
    }
    else
    ROS_INFO("diff_originX:%f, diff_originY:%f, diff_originW:%f", diff_originx,diff_originy,diff_originw);


    if(diff_goalx < 0.25 && diff_goaly < 0.2 && diff_goalw < 0.2)
    {
      ROS_INFO("The robot has arrived at the goal point");
      goal_flag=1;
    }
    
    else if(diff_originx < 0.4 && diff_originy < 0.2 && diff_originw < 0.2)
    {
      ROS_INFO("The robot has arrived at the origin point");
      origin_flag=1;
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher goal_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher origin_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while(ros::ok())
  {
    ros::Subscriber amcl_pose_sub = n.subscribe("/amcl_pose", 10, carry_marker);

    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = ros::Time::now();
    origin_marker.header.frame_id = "map";
    origin_marker.header.stamp = ros::Time::now();
 
    goal_marker.type = shape;
    origin_marker.type = shape; 

    goal_marker.ns = "goal_marker";
    goal_marker.id = 0;
    origin_marker.ns = "origin_marker";
    origin_marker.id = 1;

    goal_marker.pose.position.x = 0.8;
    goal_marker.pose.position.y = 5.8;
    goal_marker.pose.position.z = 0;
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;

    goal_marker.scale.x = 0.2;
    goal_marker.scale.y = 0.2;
    goal_marker.scale.z = 0.2;

    goal_marker.color.r = 0.0f;
    goal_marker.color.g = 1.0f;
    goal_marker.color.b = 0.0f;
    goal_marker.color.a = 1.0;
    // goal_marker.lifetime = ros::Duration(5);

    origin_marker.pose.position.x = 0.3;
    origin_marker.pose.position.y = 3.0;
    origin_marker.pose.position.z = 0;
    origin_marker.pose.orientation.x = 0.0;
    origin_marker.pose.orientation.y = 0.0;
    origin_marker.pose.orientation.z = 0.0;
    origin_marker.pose.orientation.w = 0.1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    origin_marker.scale.x = 0.2;
    origin_marker.scale.y = 0.2;
    origin_marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    origin_marker.color.r = 0.0f;
    origin_marker.color.g = 1.0f;
    origin_marker.color.b = 0.0f;
    origin_marker.color.a = 1.0;
    // origin_marker.lifetime = ros::Duration(5);

    if(goal_flag == 0 && origin_flag == 0)
    {
      goal_marker.action = visualization_msgs::Marker::ADD;
      goal_marker_pub.publish(goal_marker);   
      goal_flag == 0;
      sleep(1);
    }

    else if(goal_flag == 1 && origin_flag == 0)
    {
      goal_marker.action = visualization_msgs::Marker::DELETE;
      goal_marker_pub.publish(goal_marker);   
      sleep(1);
    }

    else if(goal_flag == 1 && origin_flag == 1)
    {
      origin_marker.action = visualization_msgs::Marker::ADD;
      origin_marker_pub.publish(origin_marker);   
      sleep(1);
    }    
    
    ros::spinOnce();
    r.sleep();
  } 
}
