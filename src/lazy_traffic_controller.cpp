// Created by Indraneel and Naren on 19/09/22

#include "lazy_traffic_controller.hpp"
#include "robosar_messages/agent_status.h"


LazyTrafficController::LazyTrafficController(): controller_active_(true), fleet_status_outdated_(false), map_frame_id_("map"),
                                                controller_period_s(0.1)  {
    
    status_subscriber_ = nh_.subscribe("/robosar_agent_bringup_node/status", 1, &LazyTrafficController::statusCallback, this);
    // Get latest fleet info from agent bringup
    status_client = nh_.serviceClient<robosar_messages::agent_status>("/robosar_agent_bringup_node/agent_status");
    traffic_controller_thread_ = std::thread(&LazyTrafficController::RunController, this);

    // Get active agents from agent bringup
    active_agents = getFleetStatusInfo();
    ROS_INFO(" [LAZY_TRAFFIC_CONTROLLER] Active fleet size %ld",active_agents.size());

    // Initialise agent map
    initialiseAgentMap(active_agents);

    // Start controller timer
    controller_timer = nh_.createTimer(ros::Duration(controller_period_s),boost::bind(&LazyTrafficController::computeVelocities, this, _1));
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

    ros::Rate r(1);
    while(ros::ok() && controller_active_) {
        ros::spinOnce();
        // Check if fleet status is outdated
        if(fleet_status_outdated_) {
            // TODO: Update agent map
            fleet_status_outdated_ = false;
        }
        
        r.sleep();
    }

}

void LazyTrafficController::computeVelocities(const ros::TimerEvent&)
{
    
}


void LazyTrafficController::initialiseAgentMap(std::set<std::string> active_agents) {
    
    for (auto agent : active_agents) {
        agent_map_[agent] = Agent_s();
        agent_map_[agent].name = agent;
        agent_map_[agent].pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/robosar_agent_bringup_node/" + agent + "/cmd_vel", 1);
        agent_map_[agent].robot_frame_id_ = agent + "/base_link";
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