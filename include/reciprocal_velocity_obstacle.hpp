// Created by Naren on 22/09/22
#ifndef RECIPROCAL_VELOCITY_OBSTACLE_H
#define RECIPROCAL_VELOCITY_OBSTACLE_H

#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <thread>
#include <mutex>

#include "robosar_messages/robosar_controller.h"
#include "lazy_traffic_agent.hpp"
// ROS stuff
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <queue>

//A class that operates on 2D
#include "Vector2.h"
#include <queue>
#include <map>
#define NUM_VELOCITY_SAMPLES 5 //NUMBER OF SAMPLES PER EACH AGENT
#define MAX_SPEED 1.5 // Maximum speed of agent
#define AGENT_RADIUS 1.3 // Radius of agent
#define RADIUS_MULT_FACTOR 2 // since r1 + r2 = 2*r for RoboSAR, Define agent_radius and mult_factor for obstacle cone
#define TIME_STEP 1 //frequence at which controller runs ( 1/ timestep)
#define SAFETY_FACTOR 2 //The safety factor of the agent (weight for penalizing candidate velocities - the higher the safety factor, the less 'aggressive' an agent is)
#define MAX_NEIGHBORS 4 // Maximum number of neighbors to consider
#define MAX_NEIGH_DISTANCE 2.00 //Max distance among neighbors
using namespace std;
typedef pair<string, float> dist;

class RecVelocityObs {
    
    private:
        vector<dist> neighbors_ = vector<dist>(MAX_NEIGHBORS);
        map<string, Agent> agent_map_;
        RVO::Vector2 vel_pref_;
        map<string,RVO::Vector2> velocity_vector_;
        map<string, RVO::Vector2> position_vector_;
        RVO::Vector2 pos_curr_;
        RVO::Vector2 vel_curr_;
        void computeNearestNeighbors();
        RVO::Vector2 computeNewVelocity();
        bool is_collision_ = false;
        float timeToCollision(const RVO::Vector2& p, const RVO::Vector2& v, const RVO::Vector2& p2, float radius, bool& collision);
    
    public:

        RecVelocityObs(map<string, Agent> agent_map, string agent_name)
        {
            
            // Pass agent map and current position to computeNeighbors 
            //populate velocity vector based on neighbor
            //call compute velocity
            
            agent_map_ = agent_map;      
            RVO::Vector2 current_pose(agent_map_[agent_name].current_pose_.transform.translation.x, agent_map_[agent_name].current_pose_.transform.translation.y);
            pos_curr_ = current_pose;
            vel_curr_ = agent_map_[agent_name].current_velocity_;
            vel_pref_ = agent_map_[agent_name].preferred_velocity_;
            computeNearestNeighbors();
            for(auto n:neighbors_)
            {
                velocity_vector_[n.first] = agent_map_[n.first].current_velocity_;
                RVO::Vector2 pose_(agent_map_[n.first].current_pose_.transform.translation.x, agent_map_[n.first].current_pose_.transform.translation.y);
                position_vector_[n.first] = pose_;
            }
            vel_computed_ = computeNewVelocity();
        }

        ~RecVelocityObs(void){};
                                // int agent_id, RVO::Vector2 vel_pref, vector<RVO::Vector2>& velocity_vector, vector<RVO::Vector2>& position_vector, priority_queue< pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>> > neighbors
        
        
        RVO::Vector2 vel_computed_;
        
};
#endif // RECIPROCAL_VELOCITY_OBSTACLE_H