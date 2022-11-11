// Created by Indraneel on 27/09/22

#include <ros/ros.h>
#include "robosar_messages/task_allocation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ltc_head_on_collision_test_node");
  
  robosar_messages::task_allocation task_allocation_msg;

  // Add agent 0
  task_allocation_msg.id.push_back("agent_0");
  task_allocation_msg.startx.push_back(45.0);
  task_allocation_msg.starty.push_back(10.0);
  task_allocation_msg.goalx.push_back(50.5);
  task_allocation_msg.goaly.push_back(10.0);

   // Add agent 1
  task_allocation_msg.id.push_back("agent_1");
  task_allocation_msg.startx.push_back(49.0);
  task_allocation_msg.starty.push_back(11.0);
  task_allocation_msg.goalx.push_back(49.0);
  task_allocation_msg.goaly.push_back(10.0);

  // Add agent 2
  task_allocation_msg.id.push_back("agent_2");
  task_allocation_msg.startx.push_back(50.0);
  task_allocation_msg.starty.push_back(10.0);
  task_allocation_msg.goalx.push_back(46.0);
  task_allocation_msg.goaly.push_back(10.0);

  // Create publisher
  ros::NodeHandle n;
  ros::Publisher task_allocation_pub = 
                n.advertise<robosar_messages::task_allocation>("task_allocation", 1000);
  // Add a delay
  ros::Duration(2.0).sleep();

  // Publish message
  task_allocation_pub.publish(task_allocation_msg);


  //move_base::MoveBase move_base( buffer );
  
  //LazyTrafficController traffic_controller_session;

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}