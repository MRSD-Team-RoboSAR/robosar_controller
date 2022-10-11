// Created by Indraneel on 22/09/22

#ifndef LAZY_TRAFFIC_AGENT_H
#define LAZY_TRAFFIC_AGENT_H

#include <string>
#include <queue>

// ROS stuff
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "robosar_messages/controller_status.h"
#include <nav_msgs/OccupancyGrid.h>
#include "Vector2.h"
#include "lazy_traffic_rvo.hpp"

typedef pair<string, float> AgentDistPair;

using namespace std;

#define MAX_NEIGHBORS 4 // Maximum number of neighbors to consider
#define MAX_NEIGH_DISTANCE 2.00 //Max distance among neighbors
#define COLLISION_THRESH 50 // Collision threshold
#define USE_STATIC_OBSTACLE_AVOIDANCE 0
class Agent {

public:
    Agent() : name_(""), robot_frame_id_(""), current_path_(), current_pose_() {}
    Agent(std::string name, ros::NodeHandle nh) : name_(name), robot_frame_id_(name + "/base_link"), nh_(nh),
                                                  ld_(0.4), v_max_(0.2), goal_threshold(0.2), w_max_(1.0),
                                                  preferred_velocity_(RVO::Vector2(0.0, 0.0)), current_velocity_(RVO::Vector2(0.0, 0.0)) {
        // Initialise publisher
        pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/robosar_agent_bringup_node/" + name + "/cmd_vel", 1);
        pub_status_ = nh_.advertise<robosar_messages::controller_status>("/lazy_traffic_controller/" + name + "/status", 1);
        status.data = status.IDLE;
    }
    ~Agent() {}

    void publishStatus() {
        pub_status_.publish(status);
    }
    void sendVelocity(RVO::Vector2 vel);
    void stopAgent(void);
    
    void updatePreferredVelocity(void);
    // Function to call reciprocal Velocity Obstacles
    void invokeRVO(std::unordered_map<std::string, Agent> agent_map, const nav_msgs::OccupancyGrid& new_map);

    std::string robot_frame_id_;
    //Velocity Obstacle related members
    std::queue<geometry_msgs::PoseStamped> current_path_;
    geometry_msgs::TransformStamped current_pose_;
    RVO::Vector2 preferred_velocity_;
    RVO::Vector2 current_velocity_;
    RVO::Vector2 rvo_velocity_;

private:
    void ppProcessLookahead(geometry_msgs::Transform current_pose);
    bool checkifGoalReached();
    //Function to compute Nearest Neighbors of an agent using euclidian distance
    void computeNearestNeighbors(std::unordered_map<std::string, Agent> agent_map);
    void computeStaticObstacles(std::unordered_map<std::string, Agent> agent_map, const nav_msgs::OccupancyGrid& new_map);
    void breadthFirstSearch(const RVO::Vector2& start, const std::vector<int8_t>& map_data, const int& map_width, const int& map_height, const double& map_resolution,const geometry_msgs::Point& map_origin);
    RVO::Vector2 getCurrentHeading();
    unsigned int size_width;
    unsigned int size_height;
    double origin_x;
    double origin_y;
    double resolution;
    std::string map_frame_;  /// @brief frame that map is located in
    unsigned char* costmap_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_status_;
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped lookahead_;
    robosar_messages::controller_status status;

    double v_max_;
    double w_max_;
    double ld_;
    double goal_threshold;
    std::string name_;

    // Velocity obstacles related members
    std::vector<rvo_agent_obstacle_info_s> neighbors_list_;
    rvo_agent_obstacle_info_s my_info;
    std::vector<int8_t> map_data;
    int dir[8][2] = {{-1 , -1}, {-1 , 0}, {-1 , 1}, {0 , -1}, {0 , 1}, {1 , -1}, {1 , 0}, {1 , 1}};
};

#endif // LAZY_TRAFFIC_AGENT_H