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
  std::queue<geometry_msgs::PoseStamped> goalQueue;
  unsigned pp_idx_;
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
    ld_(1.0), v_max_(0.1), v_(v_max_), w_max_(1.0), pos_tol_(0.1), pp_idx_(0),goal_reached_(true), 
    nh_private_("~"), tf_listener_(tf_buffer_), map_frame_id_("map"), robot_frame_id_("base_link"),
    lookahead_frame_id_("lookahead"), controller_period_s(0.1), controller_it(0), v_linear_last(0.0),
    time_last(0.0), rotate_to_global_plan(true)
  {
    // Populate messages with static data
    lookahead_.header.frame_id = robot_frame_id_;
    lookahead_.child_frame_id = lookahead_frame_id_;
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
        
        // Add points to the goal queue
        if(idx_!=0 && checkIfPosesEqual(cartesian_pose,cartesian_path_.poses.back()) && !checkIfPosesEqual(cartesian_pose,goalQueue.back()))
        {
          goalQueue.push(cartesian_pose);
        }

        // Add final goal too
        if(idx_ == new_path.poses.size()-1 &&  !checkIfPosesEqual(cartesian_pose,goalQueue.back())) {
          goalQueue.push(cartesian_pose);
        }

        // Create cartesian path for pure pursuit
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

    // Reset variables
    pp_idx_ = 0;
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
      ppProcessLookahead(tf.transform);

      controller_it++;
      cycleWaypointsUsingTime();

      if(!path_.empty()){

        // TODO @Charvi linear velocity
        v_ = 0.0;
        cmd_vel_.linear.x = v_;

         // Compute the angular velocity.
        // Lateral error is the y-value of the lookahead point (in base_link frame)
        double yt = lookahead_.transform.translation.y;
        double ld_2 = ld_ * ld_;
        cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );


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

 // Synchronise controller time with path_time
  void cycleWaypointsUsingTime() {

    double time_elapsed = controller_it*controller_period_s;
    double path_time = path_.front()[2];
    int it=0;
    
    while(!path_.empty() && time_elapsed>path_time) {
      it++;
      // Update time last
      time_last = path_.front()[2];
      path_.pop();
    }
    ROS_INFO("Popped %d velocities",it);

  }
  void ppProcessLookahead(geometry_msgs::Transform current_pose) {

    for (; pp_idx_ < cartesian_path_.poses.size(); pp_idx_++)
    {
      if (distance(cartesian_path_.poses[pp_idx_].pose.position, current_pose.translation) > ld_)
      {

        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = transformToBaseLink(cartesian_path_.poses[pp_idx_].pose, current_pose);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);
        
        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs

        break;
      }
    }

    if (!cartesian_path_.poses.empty() && pp_idx_ >= cartesian_path_.poses.size()) {

        // We need to extend the lookahead distance
        // beyond the goal point.
      
        // This is the pose of the goal w.r.t. the base_link frame
        KDL::Frame F_bl_end = transformToBaseLink(cartesian_path_.poses.back().pose, current_pose);

        // Find the intersection between the circle of radius ld
        // centered at the robot (origin)
        // and the line defined by the last path pose
        double roll, pitch, yaw;
        F_bl_end.M.GetRPY(roll, pitch, yaw);
        double k_end = tan(yaw); // Slope of line defined by the last path pose
        double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
        double a = 1 + k_end * k_end;
        double b = 2 * l_end;
        double c = l_end * l_end - ld_ * ld_;
        double D = sqrt(b*b - 4*a*c);
        double x_ld = (-b + copysign(D,v_)) / (2*a);
        double y_ld = k_end * x_ld + l_end;
        
        lookahead_.transform.translation.x = x_ld;
        lookahead_.transform.translation.y = y_ld;
        lookahead_.transform.translation.z = F_bl_end.p.z();
        F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
                                 lookahead_.transform.rotation.y,
                                 lookahead_.transform.rotation.z,
                                 lookahead_.transform.rotation.w);
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


  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse()*F_map_pose;
}



  //! Helper founction for computing eucledian distances in the x-y plane.
  template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }
};



#endif