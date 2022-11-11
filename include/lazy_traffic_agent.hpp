// Created by Indraneel on 22/09/22

#ifndef LAZY_TRAFFIC_AGENT_H
#define LAZY_TRAFFIC_AGENT_H

#include <string>
#include <queue>
#include <set>

// ROS stuff
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "robosar_messages/controller_status.h"
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>


#include "Vector2.h"
#include "lazy_traffic_rvo.hpp"

typedef std::pair<std::string, float> AgentDistPair;

using namespace std;

#define MAX_NEIGHBORS (5) // Maximum number of neighbors to consider
#define MAX_NEIGH_DISTANCE (2.00) //Max distance among neighbors
#define COLLISION_THRESH (50) // Collision threshold
#define USE_STATIC_OBSTACLE_AVOIDANCE (1)
#define MAX_STATIC_OBS_DIST (0.5)

#define SEARCH_ANGULAR_VELOCITY (0.7)
class Agent {

public:
    Agent() : name_(""), robot_frame_id_(""), current_path_(), current_pose_() {}
    Agent(std::string name, ros::NodeHandle nh) : name_(name), robot_frame_id_(name + "/base_link"), nh_(nh),
                                                  ld_(0.4), v_max_(0.2), goal_threshold_(0.2), w_max_(0.5), at_rest(true),
                                                  preferred_velocity_(RVO::Vector2(0.0, 0.0)), current_velocity_(RVO::Vector2(0.0, 0.0)) {
        // Initialise publisher
        pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/robosar_agent_bringup_node/" + name + "/cmd_vel", 1);
        pub_status_ = nh_.advertise<robosar_messages::controller_status>("/lazy_traffic_controller/" + name + "/status", 1);
        vel_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/lazy_traffic_controller/" + name + "/vel_marker", 1);
        status.data = status.IDLE;

        // Initialise the marker message
        vel_marker_.header.frame_id = "map";
        vel_marker_.ns = "vel_marker";
        vel_marker_.id = 0;
        vel_marker_.type = visualization_msgs::Marker::ARROW;
        vel_marker_.action = visualization_msgs::Marker::ADD;
        vel_marker_.scale.x = 0.5;
        vel_marker_.scale.y = 0.05;
        vel_marker_.scale.z = 0.05;
        vel_marker_.color.a = 1.0; // Don't forget to set the alpha!
        vel_marker_.lifetime = ros::Duration(1.0);

        // Initialise dir model for bfs
        dir_ = {{-1 , -1}, {-1 , 0}, {-1 , 1}, {0 , -1}, {0 , 1}, {1 , -1}, {1 , 0}, {1 , 1}};
    }
    ~Agent() {}

    void publishStatus() {
        pub_status_.publish(status);
    }
    void sendVelocity(RVO::Vector2 vel);
    void stopAgent(void);
    void clearPath(void);
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

    enum GOAL_TYPE 
    {   
        SURVEILLANCE,
        EXPLORE, 
        HOME
    };

    int goal_type_ = 0;
private:
    void ppProcessLookahead(geometry_msgs::Transform current_pose);
    bool checkifGoalReached();
    //Function to compute Nearest Neighbors of an agent using euclidian distance
    void computeNearestNeighbors(std::unordered_map<std::string, Agent> agent_map);
    void computeStaticObstacles(const nav_msgs::OccupancyGrid& new_map);
    void staticObstacleBfs(const RVO::Vector2& start, const std::vector<int8_t>& map_data, 
                            const int& map_width, const int& map_height, 
                            const float& map_resolution,const geometry_msgs::Point& map_origin);
    RVO::Vector2 getCurrentHeading();
    void publishPreferredVelocityMarker(void);
    void publishVOVelocityMarker(void);
    
    ros::Publisher pub_vel_;
    ros::Publisher pub_status_;
    ros::Publisher vel_marker_pub_;
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped lookahead_;
    robosar_messages::controller_status status;
    visualization_msgs::Marker vel_marker_;

    double v_max_;
    double w_max_;
    double ld_; // Lookahead distance
    double goal_threshold_;
    bool at_rest;
    std::string name_;

    // Velocity obstacles related members
    std::vector<rvo_agent_obstacle_info_s> neighbors_list_;
    std::vector<std::vector<int>> dir_;

    void rotateInPlace(void);
    int rot_count_ = 0;
    bool rot_completed_ = false;
    int pause_count_ = 0;
    int agent_state_ = TRACKING;
    int search_phase_ = -1;
    enum AGENT_STATE {
        TRACKING,
        ROTATION,
        SEARCHING,
        ROTATION_COMPLETED,
        GOAL_REACHED
    };
    enum SEARCH_PHASE {
        PHASE_1,
        PHASE_2,
        PHASE_3
    };
};

#endif // LAZY_TRAFFIC_AGENT_H