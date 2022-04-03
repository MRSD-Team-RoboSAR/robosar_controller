#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robosar_controller/PurePursuitAction.h>
#include <robosar_controller/RobosarControllerAction.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <kdl/frames.hpp>
#include <robosar_controller/PurePursuitConfig.h>
class ControllerAction
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
  ackermann_msgs::AckermannDriveStamped cmd_acker_;
protected:

  actionlib::SimpleActionServer<robosar_controller::RobosarControllerAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  robosar_controller::RobosarControllerFeedback feedback_;
  robosar_controller::RobosarControllerResult result_;

public:

  ControllerAction(std::string name) :
    as_(nh_, name, boost::bind(&ControllerAction::executeCB, this, _1), false),action_name_(name),
    ld_(1.0), v_max_(0.1), v_(v_max_), w_max_(1.0), pos_tol_(0.1), idx_(0),goal_reached_(true), 
    nh_private_("~"), tf_listener_(tf_buffer_), map_frame_id_("map"), robot_frame_id_("base_link"),
    lookahead_frame_id_("lookahead"), controller_period_s(0.1)
  {
    // Get parameters from the parameter server
    nh_private_.param<double>("wheelbase", L_, 1.0);
    nh_private_.param<double>("lookahead_distance", ld_, 1.0);
    //nh_private_.param<double>("linear_velocity", v_, 0.1);
    nh_private_.param<double>("max_rotational_velocity", w_max_, 1.0);
    nh_private_.param<double>("position_tolerance", pos_tol_, 0.5);
    nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);
    nh_private_.param<double>("acceleration", acc_, 100.0);
    nh_private_.param<double>("jerk", jerk_, 100.0);
    nh_private_.param<double>("steering_angle_limit", delta_max_, 1.57);
    nh_private_.param<std::string>("map_frame_id", map_frame_id_, "map");
    // Frame attached to midpoint of rear axle (for front-steered vehicles).
    nh_private_.param<std::string>("robot_frame_id", robot_frame_id_, "robot_0/base_link");
    // Lookahead frame moving along the path as the vehicle is moving.
    nh_private_.param<std::string>("lookahead_frame_id", lookahead_frame_id_, "robot_0/base_laser_link");
    // Frame attached to midpoint of front axle (for front-steered vehicles).
    nh_private_.param<std::string>("ackermann_frame_id", acker_frame_id_, "robot_0/base_link");

    // Populate messages with static data
    lookahead_.header.frame_id = robot_frame_id_;
    lookahead_.child_frame_id = lookahead_frame_id_;

    cmd_acker_.header.frame_id = acker_frame_id_;
    cmd_acker_.drive.steering_angle_velocity = delta_vel_;
    cmd_acker_.drive.acceleration = acc_;
    cmd_acker_.drive.jerk = jerk_;


    as_.start();
  }

  void executeCB(const robosar_controller::RobosarControllerGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("Executing Action");
    receivePath(goal->path);
    controller_timer = nh_.createTimer(ros::Duration(controller_period_s),boost::bind(&ControllerAction::computeVelocities, this, _1));
    controller_timer.stop();
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>(goal->agent_name+"/cmd_vel", 1);

    while(!goal_reached_)
    {
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
    }

    if(goal_reached_)
    {
      result_.goal_reached = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

  ~ControllerAction(void)
  {
  }
  void receivePath(nav_msgs::Path new_path)
  {
    ROS_INFO("Receiving path!");

    if(new_path.poses.size()>0)
    {
      for (int idx_; idx_ < new_path.poses.size(); idx_++){
        std::vector<double> coordinates;
        coordinates.push_back(new_path.poses[idx_].pose.position.x);
        coordinates.push_back(new_path.poses[idx_].pose.position.y);
        coordinates.push_back(new_path.poses[idx_].pose.position.z);
        path_.push(coordinates);
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
    
  }


  void computeVelocities(const ros::TimerEvent&)
  {
    if(!path_.empty()){
      std::vector<double> coordinates = path_.front();
      std::vector<double> currState{0,0,0,0}; //TODO getting current x,y,theta,v values

      double dx = coordinates[0] - currState[0];
      double dy = coordinates[1] - currState[1];
      double v_f = sqrt(dx*dx + dy*dy);
      double theta_f = atan(dy/dx);

      double vd_x = v_f*cos(theta_f) - currState[3]*cos(currState[2]);
      double vd_y = v_f*sin(theta_f) - currState[3]*sin(currState[2]);
      double vd = sqrt(vd_x*vd_x + vd_y*vd_y);
      double alpha = atan(vd_y/vd_x);

      cmd_vel_.linear.x = vd;
      cmd_vel_.angular.z = alpha;
      pub_vel_.publish(cmd_vel_);
      path_.pop();
    }
    else{
      //Path list is empty -> goal should have been reached
      //Stop moving
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
      goal_reached_ = true;
    }

    /*
    // Get the current robot pose
    geometry_msgs::TransformStamped tf;
    try
    {
      tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
      // We first compute the new point to track, based on our current pose,
      // path information and lookahead distance.
      for (; idx_ < path_.poses.size(); idx_++)
      {
        if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > ld_)
        {

          // Transformed lookahead to base_link frame is lateral error
          KDL::Frame F_bl_ld = transformToBaseLink(path_.poses[idx_].pose, tf.transform);
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

      if (!path_.poses.empty() && idx_ >= path_.poses.size())
      {
        // We are approaching the goal,
        // which is closer than ld

        // This is the pose of the goal w.r.t. the base_link frame
        KDL::Frame F_bl_end = transformToBaseLink(path_.poses.back().pose, tf.transform);

        if (fabs(F_bl_end.p.x()) <= pos_tol_)
        {
          // We have reached the goal
          goal_reached_ = true;

          // Reset the path
          path_ = nav_msgs::Path();
        }
        else
        {
          // We need to extend the lookahead distance
          // beyond the goal point.
        
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

      if (!goal_reached_)
      {
        // We are tracking.

        // Compute linear velocity.
        // Right now,this is not very smart :)
        v_ = copysign(v_max_, v_);
        
        // Compute the angular velocity.
        // Lateral error is the y-value of the lookahead point (in base_link frame)
        double yt = lookahead_.transform.translation.y;
        double ld_2 = ld_ * ld_;
        cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );

        // Compute desired Ackermann steering angle
        cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );
        
        // Set linear velocity for tracking.
        cmd_vel_.linear.x = v_;
        cmd_acker_.drive.speed = v_;

        cmd_acker_.header.stamp = ros::Time::now();
      }
      else
      {
        // We are at the goal!

        // Stop the vehicle
        
        // The lookahead target is at our current pose.
        lookahead_.transform = geometry_msgs::Transform();
        lookahead_.transform.rotation.w = 1.0;
        
        // Stop moving.
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;

        cmd_acker_.header.stamp = ros::Time::now();
        cmd_acker_.drive.steering_angle = 0.0;
        cmd_acker_.drive.speed = 0.0;
      }

      // Publish the lookahead target transform.
      lookahead_.header.stamp = ros::Time::now();
      tf_broadcaster_.sendTransform(lookahead_);
      
      // Publish the velocities
      pub_vel_.publish(cmd_vel_);
      
      // Publish ackerman steering setpoints
      pub_acker_.publish(cmd_acker_);
    }
    catch (tf2::TransformException &ex)
    {
      //ROS_WARN_STREAM(ex.what());
    }*/
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "purepursuit");
  ControllerAction purepursuit("purepursuit");
  ros::spin();

  return 0;
}