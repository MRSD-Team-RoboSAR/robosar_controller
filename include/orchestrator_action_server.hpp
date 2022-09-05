// Created by Indraneel on 5/9/22

#ifndef ORCHESTRATOR_ACTION_SERVER_HPP
#define ORCHESTRATOR_ACTION_SERVER_HPP

#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <actionlib/server/simple_action_server.h>
#include <robosar_controller/RobosarControllerAction.h>
#include <robosar_controller/OrchestratorControllerAction.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <kdl/frames.hpp>
#include <robosar_controller/PurePursuitConfig.h>
#include <angles/angles.h>

class OrchestratorControllerAction
{
private:
  ros::NodeHandle nh_;
  ros::Timer controller_timer;
  ros::Publisher pub_vel_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;

protected:
  std::string action_name_;
  // create messages that are used to published feedback/result
  robosar_controller::OrchestratorControllerFeedback feedback_;
  robosar_controller::OrchestratorControllerResult result_;

public:
  actionlib::SimpleActionServer<robosar_controller::OrchestratorControllerAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

  OrchestratorControllerAction(std::string name) :
    as_(nh_, name, boost::bind(&OrchestratorControllerAction::executeCB, this, _1), false),action_name_(name),
    tf_listener_(tf_buffer_)
  {
    as_.start();
  }

  void executeCB(const robosar_controller::OrchestratorControllerGoalConstPtr &goal)
  {


  }
};

#endif