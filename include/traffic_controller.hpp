// Created by Indraneel and Naren on 19/09/22
#ifndef TRAFFIC_CONTROLLER_H
#define TRAFFIC_CONTROLLER_H

#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <thread>
#include <ros/console.h>
#include <ros/ros.h>

class TrafficController {

public:

    TrafficController();
    ~TrafficController(void);


private:

    void RunController(void);

    std::thread traffic_controller_thread_;
    bool controller_active_;

};
#endif // TRAFFIC_CONTROLLER_H