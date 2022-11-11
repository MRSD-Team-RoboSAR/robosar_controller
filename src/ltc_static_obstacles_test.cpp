// Created by Indraneel on 27/09/22

#include <ros/ros.h>
#include "robosar_messages/robosar_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ltc_static_obstacles_test_node");
  ros::NodeHandle nh_;
  robosar_messages::robosar_controller srv;
  std::vector<geometry_msgs::PoseStamped> traj;
  std::vector<geometry_msgs::PoseStamped> traj_2;
  geometry_msgs::PoseStamped pose1;
  pose1.header.seq=0;
  pose1.header.stamp = ros::Time::now();
  pose1.header.frame_id = "map";
  pose1.pose.position.x = 44.5;
  pose1.pose.position.y = 9.5;
  traj.push_back(pose1);
  geometry_msgs::PoseStamped pose2;
  pose2.header.seq=0;
  pose2.header.stamp = ros::Time::now();
  pose2.header.frame_id = "map";
  pose2.pose.position.x = 46.5;
  pose2.pose.position.y = 10.0;
  traj.push_back(pose2);
  // geometry_msgs::PoseStamped pose3;
  // pose3.header.seq=0;
  // pose3.header.stamp = ros::Time::now();
  // pose3.header.frame_id = "map";
  // pose3.pose.position.x = 44.5;
  // pose3.pose.position.y = 8.5;
  // traj.push_back(pose3);
  // geometry_msgs::PoseStamped pose4;
  // pose4.header.seq=0;
  // pose4.header.stamp = ros::Time::now();
  // pose4.header.frame_id = "map";
  // pose4.pose.position.x = 45.0;
  // pose4.pose.position.y = 8.0;
  // traj.push_back(pose4);
  // geometry_msgs::PoseStamped pose5;
  // pose5.header.seq=0;
  // pose5.header.stamp = ros::Time::now();
  // pose5.header.frame_id = "map";
  // pose5.pose.position.x = 44.5;
  // pose5.pose.position.y = 7.5;
  // traj.push_back(pose5);
  nav_msgs::Path path_agent;
  for(auto pose:traj)
      path_agent.poses.push_back(pose);
  srv.request.paths.push_back(path_agent);
  srv.request.agent_names.push_back("agent_0");
  // srv.request.agent_names.push_back("agent_1");
  srv.request.goal_type.push_back(0);
  ros::ServiceClient controller_client = nh_.serviceClient<robosar_messages::robosar_controller>("robosar_controller/lazy_traffic_controller");

  // Call service
  controller_client.call(srv);

  // Add a delay
  ros::Duration(2.0).sleep();



  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}