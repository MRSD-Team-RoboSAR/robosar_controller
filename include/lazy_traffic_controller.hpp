// Created by Indraneel and Naren on 19/09/22
#ifndef LAZY_TRAFFIC_CONTROLLER_H
#define LAZY_TRAFFIC_CONTROLLER_H
#define NUMBER_OF_PAUSES (3)
#define USE_SIM (1)
#define NUMBER_OF_VICTIMS (12)
#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <thread>
#include <mutex>
#include<unordered_map>
#include <vector>
#include<time.h>
#include "robosar_messages/robosar_controller.h"
#include "lazy_traffic_agent.hpp"
// ROS stuff
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
/***
 * TODO :
 * Measured velocity for one iteration
 * Find out what else does RVO contain
 * 
 * */
class LazyTrafficController {

public:

    LazyTrafficController(void);
    ~LazyTrafficController(void);
   

private:

    void RunController(void);
    void statusCallback(const std_msgs::Bool &status_msg);
    std::set<std::string> getFleetStatusInfo(void);
    void initialiseAgentMap(std::set<std::string> active_agents);
    void computeVelocities(const ros::TimerEvent&);
    void occupancyGridCallback(const nav_msgs::OccupancyGrid &occupancy_grid_msg);
    bool controllerServiceCallback(robosar_messages::robosar_controller::Request &req,
                                   robosar_messages::robosar_controller::Response &res);
    void updateAgentPoses(void);

    // miscellanous
    std::thread traffic_controller_thread_;
    bool controller_active_;
    bool fleet_status_outdated_;
    std::string map_frame_id_;
    double controller_period_s;
    double velocity_calc_period_s;
    std::mutex map_mutex;

    // controller data structures
    std::unordered_map<std::string, Agent> agent_map_;
    std::set<std::string> active_agents;

    // ROS stuff
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::ServiceClient status_client_; 
    ros::ServiceServer controller_service_;
    ros::Timer controller_timer_;
    ros::Subscriber status_subscriber_;
    ros::Subscriber occupancy_grid_subscriber_;
    ros::Subscriber gui_subscriber_;
    ros::NodeHandle nh_;
    nav_msgs::OccupancyGrid occupancy_grid_map_;
    //AprtilTag poses
    std::vector<geometry_msgs::PoseStamped> apriltag_poses_;
    void initializeApriltagPoses() ;
    std::pair<float, float> pixelsToMap(int x_pixel, int y_pixel);
    visualization_msgs::Marker apriltag_marker_;
    ros::Publisher apriltag_marker_pub_;
    void publishAprilTagMarkers() ;
};
#endif // LAZY_TRAFFIC_CONTROLLER_H