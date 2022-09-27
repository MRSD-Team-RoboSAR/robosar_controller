// Created by Naren on 22/09/22
#ifndef LAZY_TRAFFIC_RVO_H
#define LAZY_TRAFFIC_RVO_H


//A class that operates on 2D
#include "Vector2.h"
#include <queue>
#include <map>
#include <unordered_map>

#define RVO_VELOCITY_SAMPLES 250 //NUMBER OF SAMPLES PER EACH AGENT
#define RVO_AGENT_RADIUS 0.15 // Radius of agent
#define RVO_RADIUS_MULT_FACTOR 2 // since r1 + r2 = 2*r for RoboSAR, Define agent_radius and mult_factor for obstacle cone
#define TIME_STEP 1 //frequence at which controller runs ( 1/ timestep)
#define RVO_SAFETY_FACTOR 7.5 //The safety factor of the agent (weight for penalizing candidate velocities - the higher the safety factor, the less 'aggressive' an agent is)
#define RVO_INFTY 9e9f

using namespace std;
typedef pair<string, float> AgentDistPair;

typedef struct rvo_agent_info {
  string agent_name;
  RVO::Vector2 currrent_velocity;
  RVO::Vector2 preferred_velocity;
  RVO::Vector2 current_position;
  double max_vel;
} rvo_agent_info_s;

//Function to compute if agent is in collision
inline float rvoTimeToCollision(const RVO::Vector2& ego_position, const RVO::Vector2& vel_a_to_b, const RVO::Vector2& neighbor_position, float obstacle_radius, bool& is_in_collision) {
    RVO::Vector2 minkowski_diff = neighbor_position - ego_position;
    float sq_diam = obstacle_radius * obstacle_radius;
    float time;

    float discr = -sqrt(det(vel_a_to_b, minkowski_diff)) + sq_diam * absSq(vel_a_to_b);
    if (discr > 0) 
    {
      if (is_in_collision) 
      {
        time = (vel_a_to_b*minkowski_diff + sqrt(discr)) / absSq(vel_a_to_b);
        if (time < 0) {
          time = -RVO_INFTY;
        }
      } 
      else 
      {
        time = (vel_a_to_b*minkowski_diff - sqrt(discr)) / absSq(vel_a_to_b);
        if (time < 0) 
        {
          time = RVO_INFTY;
        }
      }
    } 
    else 
    {
      if (is_in_collision) 
      {
        time = -RVO_INFTY;
      } 
      else 
      {
        time = RVO_INFTY;
      }
    } 
    return time;
}


//Function to compute New Velocity using Reciprocal Velocity obstacles
inline RVO::Vector2 rvoComputeNewVelocity(rvo_agent_info_s ego_agent_info, 
                                   std::vector<rvo_agent_info_s> neighbors_list) {
    

    // Local variables
    RVO::Vector2 vel_cand;
    RVO::Vector2 vel_computed;
    float min_penalty = RVO_INFTY;
    RVO::Vector2 vel_pref = ego_agent_info.preferred_velocity;
    RVO::Vector2 pos_curr = ego_agent_info.current_position;
    RVO::Vector2 vel_curr = ego_agent_info.currrent_velocity;

    // TODO figure out later
    bool is_collision = false;

    // Main loop
    for(int i=0;i<RVO_VELOCITY_SAMPLES;i++) {

        //First candidate velocity is always preferred velocity
        if(i==0) {
            vel_cand = vel_pref;
        } else {
            do 
            {
              vel_cand = RVO::Vector2(2.0f*rand() - RAND_MAX, 2.0f*rand() - RAND_MAX);
            } while(absSq(vel_cand) > sqrt((float) RAND_MAX));
            vel_cand *= (ego_agent_info.max_vel / RAND_MAX);
        }

        float dist_to_pref_vel ; // distance between candidate velocity and preferred velocity
        if(is_collision)
        {
            dist_to_pref_vel = 0;
        }
        else
        {
            dist_to_pref_vel = abs(vel_cand - vel_pref);
        }

        // searching for smallest time to collision with this velocity sample
        float min_t_to_collision = RVO_INFTY;
        
        // iterate over neighbors
        for(auto n: neighbors_list) {

            // If neighbor is an obstacle, agent_Radius, position and other attributes would change
            // Change code accordingly
            float t_to_collision; // time to collision with neighbor
            RVO::Vector2 vel_a_to_b;
            RVO::Vector2 vel_b = n.currrent_velocity;
            vel_a_to_b = 2*vel_cand - vel_curr - vel_b;
            RVO::Vector2 neigh_pos = n.current_position;
            float time = rvoTimeToCollision(pos_curr, vel_a_to_b, neigh_pos, 
                              RVO_RADIUS_MULT_FACTOR*RVO_AGENT_RADIUS, is_collision);
            if(is_collision)  {
                t_to_collision = -ceil(time / TIME_STEP);
                t_to_collision -= absSq(vel_cand) / (ego_agent_info.max_vel*ego_agent_info.max_vel);

            }
            else
            {
                t_to_collision = time;
            }
            if(t_to_collision<min_t_to_collision) {

                min_t_to_collision = t_to_collision;
                if(RVO_SAFETY_FACTOR / min_t_to_collision + dist_to_pref_vel>= min_penalty)
                    break;
            }
        }
        float penalty = RVO_SAFETY_FACTOR / min_t_to_collision + dist_to_pref_vel;
        if(penalty < min_penalty)
        {
            min_penalty = penalty;
            vel_computed = vel_cand;
        }

    }
    return vel_computed;
}


#endif // LAZY_TRAFFIC_RVO_H