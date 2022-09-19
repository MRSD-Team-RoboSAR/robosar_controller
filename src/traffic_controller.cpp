// Created by Indraneel and Naren on 19/09/22

#include "traffic_controller.hpp"


TrafficController::TrafficController(): controller_active_(true) {
    
    traffic_controller_thread_ = std::thread(&TrafficController::RunController, this);
}

TrafficController::~TrafficController() {

    controller_active_ = false;
    traffic_controller_thread_.join();
}

void TrafficController::RunController() {

     ROS_INFO("[RoboSAR Controller] Opening the floodgates! ");

    while(controller_active_) {


    }

}