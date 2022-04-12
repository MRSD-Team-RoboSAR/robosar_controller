#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robosar_controller/RobosarControllerAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_controller");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<robosar_controller::RobosarControllerAction> ac("agent1", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  robosar_controller::RobosarControllerGoal goal;

  goal.path.header.frame_id = "map";
  goal.path.header.stamp = ros::Time::now();

  //goal.path.poses[0].pose.position.x = 1.0;
  //goal.path.poses[0].pose.orientation.w = 1.0;
  geometry_msgs::PoseStamped pose;
  
  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0.3;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0.6;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.9;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.4;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 1.2;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 1.5;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.6;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 1.8;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.7;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 2.1;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.7;
  pose.pose.position.y = 0.1;
  pose.pose.position.z = 2.4;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.7;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 2.7;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.7;
  pose.pose.position.y = 0.3;
  pose.pose.position.z = 3.0;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);

  pose.header.seq=0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.7;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 3.3;
  pose.pose.orientation.w = 1.0;
  goal.path.poses.push_back(pose);
  

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}