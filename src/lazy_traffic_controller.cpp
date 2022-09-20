// Created by Indraneel and Naren on 19/09/22

#include "lazy_traffic_controller.hpp"


LazyTrafficController::LazyTrafficController(): controller_active_(true) {
    
    traffic_controller_thread_ = std::thread(&LazyTrafficController::RunController, this);
}

LazyTrafficController::~LazyTrafficController() {

    controller_active_ = false;
    traffic_controller_thread_.join();
}

void LazyTrafficController::RunController() {

     ROS_INFO("[RoboSAR Controller] Opening the floodgates! ");

    while(controller_active_) {
        
        
    }

}