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

#include "Vector2.h"

class Agent {

public:
    Agent() : name_(""), robot_frame_id_(""), current_path_(), current_pose_() {}
    Agent(std::string name, ros::NodeHandle nh) : name_(name), robot_frame_id_(name + "/base_link"), nh_(nh),
                                                  ld_(0.4), v_max_(0.2), goal_threshold(0.2), w_max_(1.0) {
        // Initialise publisher
        pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/robosar_agent_bringup_node/" + name + "/cmd_vel", 1);
    }
    ~Agent() {}

    void sendVelocity(RVO::Vector2 vel);
    void stopAgent(void);
    void updatePreferredVelocity(void);

    std::string robot_frame_id_;
    std::queue<geometry_msgs::PoseStamped> current_path_;
    geometry_msgs::TransformStamped current_pose_;
    RVO::Vector2 preferred_velocity_;
private:
    void ppProcessLookahead(geometry_msgs::Transform current_pose);
    bool checkifGoalReached();
    RVO::Vector2 getCurrentHeading();
    double goal_threshold;
    
    ros::Publisher pub_vel_;
    std::string name_;
    ros::NodeHandle nh_;
    double ld_;
    geometry_msgs::TransformStamped lookahead_;

    double v_max_;
    double w_max_;


};

#endif // LAZY_TRAFFIC_AGENT_H