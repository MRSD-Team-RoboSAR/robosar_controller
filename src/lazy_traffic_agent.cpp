// Created by Indraneel on 22/09/22

#include "lazy_traffic_agent.hpp"

  void Agent::stopAgent(void) {
        // TODO Check if velocity is non zero 
        geometry_msgs::Twist vel;
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        pub_vel_.publish(vel);
    }

void Agent::updatePreferredVelocity() {

    if(current_path_.empty()) {
        //stopAgent();
        //return;
    }
    else {
        ppProcessLookahead(current_pose_.transform);

        // Calculate preferred velocity vector from current pose to lookahead point
        preferred_velocity_ = RVO::Vector2(lookahead_.transform.translation.x - current_pose_.transform.translation.x,
                                           lookahead_.transform.translation.y - current_pose_.transform.translation.y);  
        preferred_velocity_ = norm(preferred_velocity_);
        preferred_velocity_ *= v_max_;

    }
}

//! Helper founction for computing eucledian distances in the x-y plane.
template<typename T1, typename T2>
double distance(T1 pt1, T2 pt2)
{
  return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
}


void Agent::ppProcessLookahead(geometry_msgs::Transform current_pose) {

    //for (; pp_idx_ < cartesian_path_.poses.size(); pp_idx_++)
    int pp_idx_ = 0;
    while(current_path_.size()>=1)
    {
      if (distance( current_path_.front().pose.position, current_pose.translation) > ld_ )
      {
        // Save this as the lookahead point
        lookahead_.transform.translation.x = current_path_.front().pose.position.x;
        lookahead_.transform.translation.y = current_path_.front().pose.position.y;
        
        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs
        ROS_INFO("[LT_CONTROLLER-%s]: Lookahead X: %f Y: %f",&name_[0],lookahead_.transform.translation.x , lookahead_.transform.translation.y);
        return;
      }
      else 
      {
        current_path_.pop();
      }
    }

    if (!current_path_.empty()) {

        // Lookahead point is the last point in the path
        lookahead_.transform.translation.x = current_path_.front().pose.position.x;
        lookahead_.transform.translation.y = current_path_.front().pose.position.y;

        ROS_INFO("[LT_CONTROLLER-%s]:***** Lookahead X: %f Y: %f",&name_[0],lookahead_.transform.translation.x , lookahead_.transform.translation.y);
    }
    else {
      ROS_ERROR("[LT_CONTROLLER-%s]: No path to follow. Stopping agent.",&name_[0]);
    }

  }
