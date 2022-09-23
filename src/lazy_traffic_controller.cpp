// Created by Indraneel and Naren on 19/09/22

#include "lazy_traffic_controller.hpp"
#include "robosar_messages/agent_status.h"


LazyTrafficController::LazyTrafficController(): controller_active_(true), fleet_status_outdated_(false), map_frame_id_("map"),
                                                controller_period_s(0.1), nh_("robosar_controller"),tf_listener_(tf_buffer_)  {
    
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

LazyTrafficController::~LazyTrafficController() {

    controller_active_ = false;
    traffic_controller_thread_.join();
}

void LazyTrafficController::statusCallback(const std_msgs::Bool &status_msg) {
    fleet_status_outdated_ = true;
}

bool LazyTrafficController::controllerServiceCallback(robosar_messages::robosar_controller::Request &req,
                                                      robosar_messages::robosar_controller::Response &res) {
    
    std::lock_guard<std::mutex> lock(map_mutex);
    if(req.stop_controller) {
        ROS_INFO(" [LT_CONTROLLER] Emergency stop requested");
        // TODO
        // Stop all agents
        for(auto &agent : agent_map_) {
            agent.second.stop_agent();
        }
    }
    else {
        ROS_INFO(" [LT_CONTROLLER] New %ld paths received!", req.paths.size());
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
                //agent_map_[req.agent_names[i]].current_path = path_queue;
            }
            else {
                ROS_ERROR(" [LT_CONTROLLER] Empty path received for agent %s", &req.agent_names[i][0]);
            }

        }
    }

    res.success = true;
   
    return true;
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
    std::lock_guard<std::mutex> lock(map_mutex);
    // Update current poses of all agents from tf
    updateAgentPoses();
    

    // Calculate preferred velocities for all agents
    updatePreferredVelocities();


}

void LazyTrafficController::updatePreferredVelocities() {

    for(auto &agent : agent_map_) {
        //agent.second.updatePreferredVelocity();

    }
}

void LazyTrafficController::updateAgentPoses() {
    
    for(auto it = agent_map_.begin(); it != agent_map_.end(); it++) {
        // Get current pose of agent
        geometry_msgs::TransformStamped current_pose;
        try {
            current_pose = tf_buffer_.lookupTransform(map_frame_id_, it->second.robot_frame_id_, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            //ros::Duration(1.0).sleep();
            continue;
        }
        // Update current pose
        it->second.current_pose_ = current_pose;
    }
}

void LazyTrafficController::initialiseAgentMap(std::set<std::string> active_agents) {
    
    for (auto agent : active_agents) {
        agent_map_[agent] = Agent(agent, nh_);
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