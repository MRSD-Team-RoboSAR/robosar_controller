// Created by Indraneel on 19/09/22

#include <ros/ros.h>
#include "lazy_traffic_controller.hpp"
#include "reciprocal_velocity_obstacle.hpp"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosar_traffic_controller_node");

  //move_base::MoveBase move_base( buffer );
  
  LazyTrafficController traffic_controller_session;
  RecVelocityObs rvo;

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}