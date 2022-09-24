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

#include "Vector2.h"

class Agent {

public:
    Agent() : name_(""), robot_frame_id_(""), current_path_(), current_pose_() {}
    Agent(std::string name, ros::NodeHandle nh) : name_(name), robot_frame_id_(name + "/base_link"), nh_(nh),
                                                  ld_(0.4), v_max_(0.2), goal_threshold(0.2), w_max_(1.0) {
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

    std::string robot_frame_id_;
    std::queue<geometry_msgs::PoseStamped> current_path_;
    geometry_msgs::TransformStamped current_pose_;
    RVO::Vector2 preferred_velocity_;

private:
    void ppProcessLookahead(geometry_msgs::Transform current_pose);
    bool checkifGoalReached();
    RVO::Vector2 getCurrentHeading();
    
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


};

#endif // LAZY_TRAFFIC_AGENT_H