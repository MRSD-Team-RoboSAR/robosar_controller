// Created by Indraneel and Naren on 19/09/22

#include "lazy_traffic_controller.hpp"
#include "robosar_messages/agent_status.h"


LazyTrafficController::LazyTrafficController(): controller_active_(true) {
    
    status_subscriber_ = nh_.subscribe("/robosar_agent_bringup_node/status", 1, &LazyTrafficController::statusCallback, this);
    // Get latest fleet info from agent bringup
    status_client = nh_.serviceClient<robosar_messages::agent_status>("/robosar_agent_bringup_node/agent_status");
    traffic_controller_thread_ = std::thread(&LazyTrafficController::RunController, this);

    // Get active agents from agent bringup
    active_agents = getFleetStatusInfo();
    ROS_INFO(" [LAZY_TRAFFIC_CONTROLLER] Active fleet size %ld",active_agents.size());

    // Initialise agent map
    initialiseAgentMap(active_agents);
}

void LazyTrafficController::statusCallback(const std_msgs::Bool &status_msg) {
    fleet_status_outdated_ = true;
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

void LazyTrafficController::initialiseAgentMap(std::set<std::string> active_agents) {
    
    for (auto agent : active_agents) {
        agent_map_[agent] = Agent_s();
        agent_map_[agent].name = agent;
    }
}

std::set<std::string> LazyTrafficController::getFleetStatusInfo() {

    robosar_messages::agent_status srv;

    if (status_client.call(srv)) {
        std::vector<std::string> agentsVec = srv.response.agents_active;
        std::set<std::string> agentsSet;

        for(auto agent:agentsVec)
            agentsSet.insert(agent);

        return agentsSet;
    }
    else
    {
        ROS_ERROR("[MISSION_EXEC] Failed to call fleet info service");
        return active_agents;
    }
}