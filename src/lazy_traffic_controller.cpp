// Created by Indraneel and Naren on 19/09/22

#include "lazy_traffic_controller.hpp"
#include "robosar_messages/agent_status.h"


LazyTrafficController::LazyTrafficController(): controller_active_(true), fleet_status_outdated_(false), map_frame_id_("map"),
                                                controller_period_s(0.1), nh_("robosar_controller")  {
    
    status_subscriber_ = nh_.subscribe("/robosar_agent_bringup_node/status", 1, &LazyTrafficController::statusCallback, this);
    // Get latest fleet info from agent bringup
    status_client = nh_.serviceClient<robosar_messages::agent_status>("/robosar_agent_bringup_node/agent_status");
    traffic_controller_thread_ = std::thread(&LazyTrafficController::RunController, this);

    // Get active agents from agent bringup
    active_agents = getFleetStatusInfo();
    ROS_INFO(" [LT_CONTROLLER] Active fleet size %ld",active_agents.size());

    // Initialise agent map
    initialiseAgentMap(active_agents);

    // advertise controller service
    controller_service = nh_.advertiseService("lazy_traffic_controller", &LazyTrafficController::controllerServiceCallback, this);

    // Start controller timer
    controller_timer = nh_.createTimer(ros::Duration(controller_period_s),boost::bind(&LazyTrafficController::computeVelocities, this, _1));
}

void LazyTrafficController::statusCallback(const std_msgs::Bool &status_msg) {
    fleet_status_outdated_ = true;
}

bool LazyTrafficController::controllerServiceCallback(robosar_messages::robosar_controller::Request &req,
                                                      robosar_messages::robosar_controller::Response &res) {
    
    if(req.stop_controller) {
        ROS_INFO(" [LT_CONTROLLER] Emergency stop requested");
        // TODO
        // Stop all agents
    }
    else {
        ROS_INFO(" [LT_CONTROLLER] New %ld paths received!", req.paths.size());
        std::lock_guard<std::mutex> lock(map_mutex);
        for(int i = 0; i < req.paths.size(); i++) {
            
            // Ensure agent is already in the map
            if(agent_map_.find(req.agent_names[i]) == agent_map_.end()) {
                ROS_ERROR(" [LT_CONTROLLER] Agent %s not found in the map", &req.agent_names[i][0]);
                continue;
            }
            // Parse path and update agent map
            if(req.paths[i].poses.size() > 0) {
                std::queue<geometry_msgs::PoseStamped> path_queue;
                for(int j = 0; j < req.paths[i].poses.size(); j++) {
                    path_queue.push(req.paths[i].poses[j]);
                }
                agent_map_[req.agent_names[i]].current_path = path_queue;
            }
            else {
                ROS_ERROR(" [LT_CONTROLLER] Empty path received for agent %s", &req.agent_names[i][0]);
            }

        }
    }

    res.success = true;
   
    return true;
}


LazyTrafficController::~LazyTrafficController() {

    controller_active_ = false;
    traffic_controller_thread_.join();
}

void LazyTrafficController::RunController() {

    ROS_INFO("[LT_CONTROLLER] Opening the floodgates! ");

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
        ROS_ERROR("[LT_CONTROLLER] Failed to call fleet info service");
        return active_agents;
    }
}