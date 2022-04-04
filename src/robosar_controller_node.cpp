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
#include <angles/angles.h>

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
    ld_(1.0), v_max_(0.1), v_(v_max_), w_max_(0.3), pos_tol_(0.1), idx_(0),goal_reached_(true), 
    nh_private_("~"), tf_listener_(tf_buffer_), map_frame_id_("map"), robot_frame_id_("base_link"),
    lookahead_frame_id_("lookahead"), controller_period_s(0.1), controller_it(0), v_linear_last(0.0),
    time_last(0.0), rotate_to_global_plan(false)
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
    controller_it = 0; //Setting controller iterator to 0 every time action is called
    receivePath(goal->path);
    controller_timer = nh_.createTimer(ros::Duration(controller_period_s),boost::bind(&ControllerAction::computeVelocities, this, _1));
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

  ~ControllerAction(void)
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


  void computeVelocities(const ros::TimerEvent&)
  {

    // Get current pose

    geometry_msgs::TransformStamped tf;
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));

    tf::Quaternion q(tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,-
        tf.transform.rotation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Transform x: %f y:%f yaw:%f",tf.transform.translation.x,tf.transform.translation.y,yaw);

    if(rotate_to_global_plan)
    {
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
      while(!path_.empty() && time_elapsed>path_time)
      {
        it++;
        path_.pop();
      }
      ROS_INFO("Popped %d velocities",it);

      if(!path_.empty()){

        std::vector<double> coordinates = path_.front();
        std::vector<double> currState{tf.transform.translation.x,tf.transform.translation.y,yaw,v_linear_last}; //TODO getting current v value

        double dx = coordinates[0] - currState[0];
        double dy = coordinates[1] - currState[1];
        double v_f = sqrt(dx*dx + dy*dy)/(controller_period_s);
        double theta_f = atan(dy/dx);

        double vd_x = v_f*cos(theta_f) - currState[3]*cos(currState[2]);
        double vd_y = v_f*sin(theta_f) - currState[3]*sin(currState[2]);
        double vd = sqrt(vd_x*vd_x + vd_y*vd_y);
        double alpha = atan(vd_y/vd_x);

        v_linear_last = vd;
        cmd_vel_.linear.x = vd;
        cmd_vel_.angular.z = alpha;
        pub_vel_.publish(cmd_vel_);
      }
      else {
        //Path list is empty -> goal should have been reached
        //Stop moving
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
        pub_vel_.publish(cmd_vel_);
        goal_reached_ = true;
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
  ros::init(argc, argv, "robosar_controller");
  ControllerAction purepursuit("robosar_controller");
  ros::spin();

  return 0;
}