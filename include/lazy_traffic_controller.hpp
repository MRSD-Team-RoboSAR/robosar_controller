// Created by Indraneel and Naren on 19/09/22
#ifndef LAZY_TRAFFIC_CONTROLLER_H
#define LAZY_TRAFFIC_CONTROLLER_H

#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <thread>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

class LazyTrafficController {

public:

    LazyTrafficController(void);
    ~LazyTrafficController(void);


private:

    // typedef Agent {
    //     string name;
    // } Agent_s;

    // std::set<Agent_s> agent_set_;
    void RunController(void);
    void statusCallback(const std_msgs::Bool &status_msg);

    std::thread traffic_controller_thread_;
    bool controller_active_;
    ros::Subscriber status_subscriber_;
    ros::NodeHandle nh_;
    bool fleet_status_outdated_;
    ros::ServiceClient status_client; 

};
#endif // LAZY_TRAFFIC_CONTROLLER_H