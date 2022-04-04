// Created by Charvi and Indraneel on 4/04/22

#ifndef LG_ACTION_SERVER_HPP
#define LG_ACTION_SERVER_HPP

#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <actionlib/server/simple_action_server.h>
#include <robosar_controller/PurePursuitAction.h>
#include <robosar_controller/RobosarControllerAction.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <kdl/frames.hpp>
#include <robosar_controller/PurePursuitConfig.h>
#include <angles/angles.h>

class LGControllerAction
{
private:
  ros::NodeHandle nh_, nh_private_;
  ros::Timer controller_timer;
  ros::Publisher pub_vel_, pub_acker_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  std::string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_;

  bool rotate_to_global_plan;
  double v_linear_last, time_last;
  unsigned int controller_it;
  double controller_period_s;
  // Vehicle parameters
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_;
  // Generic control variables
  double v_max_, v_, w_max_;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  double delta_, delta_vel_, acc_, jerk_, delta_max_;
  std::queue<std::vector<double>> path_;
  unsigned idx_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  nav_msgs::Path cartesian_path_;
protected:

  actionlib::SimpleActionServer<robosar_controller::RobosarControllerAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  robosar_controller::RobosarControllerFeedback feedback_;
  robosar_controller::RobosarControllerResult result_;

public:

  LGControllerAction(std::string name) :
    as_(nh_, name, boost::bind(&LGControllerAction::executeCB, this, _1), false),action_name_(name),
    ld_(1.0), v_max_(0.5), v_(v_max_), w_max_(0.3), pos_tol_(0.1), idx_(0),goal_reached_(true), 
    nh_private_("~"), tf_listener_(tf_buffer_), map_frame_id_("map"), robot_frame_id_("base_link"),
    lookahead_frame_id_("lookahead"), controller_period_s(0.1), controller_it(0), v_linear_last(0.0),
    time_last(0.0), rotate_to_global_plan(true)
  {
    
    as_.start();
  }

  void executeCB(const robosar_controller::RobosarControllerGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("Executing Action");
    controller_it = 0; //Setting controller iterator to 0 every time action is called
    receivePath(goal->path);
    controller_timer = nh_.createTimer(ros::Duration(controller_period_s),boost::bind(&LGControllerAction::computeVelocities, this, _1));
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>(goal->agent_name+"/cmd_vel", 1);

    while(!goal_reached_)
    {
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        controller_timer.stop();
        break;
      }
    }

    if(goal_reached_)
    {
      controller_timer.stop();
      result_.goal_reached = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
      controller_timer.stop();
    }
  }

  ~LGControllerAction(void)
  {
  }
  void receivePath(nav_msgs::Path new_path)
  {
    ROS_INFO("Receiving path!");

    if(new_path.poses.size()>0)
    {
      for (int idx_ = 0; idx_ < new_path.poses.size(); idx_++){
        std::vector<double> coordinates;
        coordinates.push_back(new_path.poses[idx_].pose.position.x);
        coordinates.push_back(new_path.poses[idx_].pose.position.y);
        coordinates.push_back(new_path.poses[idx_].pose.position.z);
        path_.push(coordinates);
        
        geometry_msgs::PoseStamped cartesian_pose;
        cartesian_pose.pose.position.x = new_path.poses[idx_].pose.position.x;
        cartesian_pose.pose.position.y = new_path.poses[idx_].pose.position.y;
        
        if(idx_ == 0 || !checkIfPosesEqual(cartesian_pose,cartesian_path_.poses[cartesian_path_.poses.size()-1]) )
        {
          cartesian_path_.poses.push_back(cartesian_pose);
        }
      }
       goal_reached_ = false;
    }
    else
    {
      goal_reached_ = true;
      ROS_WARN_STREAM("Received empty path!");
    }
    // When a new path received, the previous one is simply discarded
    // It is up to the planner/motion manager to make sure that the new
    // path is feasible.
    // Callbacks are non-interruptible, so this will
    // not interfere with velocity computation callback.
    ROS_INFO("Received path!");
  }

  bool compare_float(float x, float y, float epsilon = 0.01f){
   if(fabs(x - y) < epsilon)
      return true; //they are same
    return false; //they are not same
  }

  bool checkIfPosesEqual(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2) {

    if(compare_float(pose1.pose.position.x,pose2.pose.position.x) 
          && compare_float(pose1.pose.position.y,pose2.pose.position.y))
      return true;
    else
      return false;
  }


  void computeVelocities(const ros::TimerEvent&)
  {

    // Get current pose

    geometry_msgs::TransformStamped tf;
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    double yaw = tf::getYaw(tf.transform.rotation);
    ROS_INFO("Transform x: %f y:%f yaw:%f",tf.transform.translation.x,tf.transform.translation.y,yaw);

    if(rotate_to_global_plan) {
        double angle_to_global_plan = calculateGlobalPlanAngle(tf.transform.translation.x,tf.transform.translation.y,yaw);
        ROS_INFO("Shortest angle to goal %f",angle_to_global_plan);
        rotate_to_global_plan = rotateToOrientation(angle_to_global_plan,0.1);
    }
    else {

      ROS_INFO("Computing Velocity");
      controller_it++;
      double time_elapsed = controller_it*controller_period_s;

      // Synchronise controller time with path_time
      double path_time = path_.front()[2];
      int it=0;
      while(!path_.empty() && time_elapsed>path_time) {
        it++;
        // Update time last
        time_last = path_.front()[2];
        path_.pop();
      }
      ROS_INFO("Popped %d velocities",it);

      if(!path_.empty()){

        std::vector<double> coordinates = path_.front();
        std::vector<double> currState{tf.transform.translation.x,tf.transform.translation.y,yaw,v_linear_last}; //TODO getting current v value

        double time_next = coordinates[2];
        double dx = coordinates[0] - currState[0];
        double dy = coordinates[1] - currState[1];
        double v_f = sqrt(dx*dx + dy*dy)/(time_next-time_last);
        double theta_f = atan(dy/dx);

        double vd_x = v_f*cos(theta_f) - currState[3]*cos(currState[2]);
        double vd_y = v_f*sin(theta_f) - currState[3]*sin(currState[2]);
        double vd = sqrt(vd_x*vd_x + vd_y*vd_y);
        double alpha = atan(vd_y/vd_x);

        v_linear_last = vd;
        publishVelocitiesGlobal(vd_x,vd_y,alpha);
      }
      else {
        //Path list is empty -> goal should have been reached
        //Stop moving
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
        pub_vel_.publish(cmd_vel_);
        goal_reached_ = true;
        ROS_INFO("Stopping!!");
      }

    }

   
  }

  double calculateGlobalPlanAngle(double x, double y, double yaw) {

      //Calculate the angles between robotpose and global plan point pose
      double angle_to_goal = atan2(path_.front()[1] - y,
                                        path_.front()[0] - x);

      
      return angles::shortest_angular_distance(yaw, angle_to_goal);
  }

  bool rotateToOrientation(double angle, double accuracy) {

      if(fabs(angle)>accuracy)
      {
          // Nothing fancy just rotate with a fixed velocity
          cmd_vel_.linear.x = 0.0;
          if(angle < 0)
          {
            cmd_vel_.angular.z = -w_max_;
          }
          else
          {
            cmd_vel_.angular.z = w_max_;
          }
          pub_vel_.publish(cmd_vel_);
          return true;
      }
      else
      {
        ROS_INFO("Rotate to orientation complete!");
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
        pub_vel_.publish(cmd_vel_);
        return false;
      }

  }

  void publishVelocitiesGlobal(double vx, double vy, double theta) {

    //Transform commands from global frame to robot coordinate system
    geometry_msgs::Vector3Stamped cmd_global, cmd_robot;
    cmd_global.header.frame_id = map_frame_id_;
    cmd_global.header.stamp = ros::Time::now();
    cmd_global.vector.x = vx;
    cmd_global.vector.y = vy;

    try
    {
      geometry_msgs::TransformStamped tfGeom;
      tfGeom = tf_buffer_.lookupTransform(robot_frame_id_, cmd_global.header.frame_id, cmd_global.header.stamp, ros::Duration(1.0));
      tf2::doTransform(cmd_global,cmd_robot,tfGeom);
    } catch (tf2::TransformException &ex){
      ROS_ERROR("%s",ex.what());
      cmd_robot.vector.x = 0.0f;
      cmd_robot.vector.y = 0.0f;
    }

    // Apply limits
    if ( std::isnan(cmd_robot.vector.x) || std::isnan(cmd_robot.vector.y) || std::isnan(theta) )
    {
      ROS_WARN("[RoboSAR Controller]: Output velocity contains NaN!");
      theta = 0.0; cmd_robot.vector.x = 0.0f; cmd_robot.vector.y = 0.0f;
    }
    else if (fabs(cmd_robot.vector.x) > v_max_)
    {
        ROS_WARN("[RoboSAR Controller]: Output velocity exceeds max value, clipping!");
        cmd_robot.vector.x = copysignf(v_max_,cmd_robot.vector.x);
    }
    else if (fabs(theta) > w_max_)
    {
      ROS_WARN("[RoboSAR Controller]: Angular velocity exceeds max value, clipping!");
        theta = copysign(w_max_,theta);
    }

    ROS_INFO("[RoboSAR Controller]: CMD_LIN %f CMD_ANG %f Tracking %f %f at %f",cmd_robot.vector.x,theta,
                                                                      path_.front()[0],path_.front()[1],path_.front()[2]);

    cmd_vel_.linear = cmd_robot.vector;
    cmd_vel_.angular.z = theta;
    // Remove other DOFs
    cmd_vel_.linear.y = 0.0; cmd_vel_.linear.z  = 0.0;
    cmd_vel_.angular.x = 0.0; cmd_vel_.angular.y = 0.0;
    pub_vel_.publish(cmd_vel_);
  }



  //! Helper founction for computing eucledian distances in the x-y plane.
  template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }
};



#endif