// Created by Naren on 22/09/22
#ifndef LAZY_TRAFFIC_RVO_H
#define LAZY_TRAFFIC_RVO_H


//A class that operates on 2D
#include "Vector2.h"
#include <queue>
#include <map>
#include <unordered_map>
#include <ros/console.h>

#define RVO_VELOCITY_SAMPLES (1000) //NUMBER OF SAMPLES PER EACH AGENT
#define RVO_AGENT_RADIUS (0.15) // Radius of agent
#define RVO_RADIUS_MULT_FACTOR (4) // since r1 + r2 = 2*r for RoboSAR, Define agent_radius and mult_factor for obstacle cone
#define RVO_RADIUS_MULT_FACTOR_HOMING (2) // since r1 + r2 = 2*r for RoboSAR, Define agent_radius and mult_factor for obstacle cone
#define TIME_STEP (1) //frequence at which controller runs ( 1/ timestep)
#define RVO_SAFETY_FACTOR (20.0f) //The safety factor of the agent (weight for penalizing candidate velocities - the higher the safety factor, the less 'aggressive' an agent is)
#define RVO_INFTY (9e9f)

typedef std::pair<std::string, float> AgentDistPair;

typedef struct rvo_agent_obstacle_info {
  std::string agent_name;
  RVO::Vector2 currrent_velocity;
  RVO::Vector2 preferred_velocity;
  RVO::Vector2 current_position;
  double max_vel;
  bool homing = false;
} rvo_agent_obstacle_info_s;


inline float sqr(float a) {
    return a*a;
}

inline bool AreSame(double a, double b)
{
    return fabs(a - b) < std::numeric_limits<double>::epsilon();
}

//Check if float values are greater than or equal to each other
inline bool AreSameOrGreater(double a, double b)
{
    return (a > b) || AreSame(a, b);
}

//Check if float values are less than or equal to each other
inline bool AreSameOrLess(double a, double b)
{
    return (a < b) || AreSame(a, b);
}

//Function to compute if agent is in collision
  inline float rvoTimeToCollision(const RVO::Vector2& p, const RVO::Vector2& v,
                         const RVO::Vector2& p2, float radius, bool collision) {

    //ROS_INFO(" RVO received p1: %f, %f, p2: %f, %f, v: %f, %f r:%f", p.x(), p.y(), p2.x(), p2.y(), v.x(), v.y(),radius);
    RVO::Vector2 ba = p2 - p;
    float relative_position = std::sqrt(absSq(ba));
    while(AreSameOrLess(relative_position,radius))// && AreSameOrGreater(radius/2,RVO_AGENT_RADIUS))
      radius = radius/2;

    float sq_diam = sqr(radius); // radius or diameter?? will be confusing while tuning
    float time;

    float discr = -sqr(det(v, ba)) + sq_diam * absSq(v);
    if (discr > 0) {
      if (collision) {
        time = (v * ba + std::sqrt(discr)) / absSq(v);
        if (time < 0) {
          time = -RVO_INFTY;
        }
      } else {
        time = (v * ba - std::sqrt(discr)) / absSq(v);
        if (time < 0) {
          time = RVO_INFTY;
        }
      }
    } else {
      if (collision) {
        time = -RVO_INFTY;
      } else {
        time = RVO_INFTY;
      }
    }
    return time;
  }


//Function to compute New Velocity using Reciprocal Velocity obstacles
inline RVO::Vector2 rvoComputeNewVelocity(rvo_agent_obstacle_info_s ego_agent_info, 
                                   const std::vector<rvo_agent_obstacle_info_s>& neighbors_list, bool isHoming) {
    
    //ROS_INFO(" ");
    //ROS_INFO(" ");
    // Local variables
    RVO::Vector2 vel_cand;
    RVO::Vector2 vel_computed;
    float min_penalty = RVO_INFTY;
    const RVO::Vector2 vel_pref = ego_agent_info.preferred_velocity;
    const RVO::Vector2 pos_curr = ego_agent_info.current_position;
    const RVO::Vector2 vel_curr = ego_agent_info.currrent_velocity;

    // TODO figure out later
    const bool is_collision = false;

    // Main loop
    int print_count = 0;
    for(int i=0;i<RVO_VELOCITY_SAMPLES;++i) {

        //First candidate velocity is always preferred velocity
        if(i==0) {
            vel_cand = vel_pref;
        } else {
            do 
            {
              vel_cand = RVO::Vector2(2.0f*rand() - RAND_MAX, 2.0f*rand() - RAND_MAX);
            } while(absSq(vel_cand) > sqr((float) RAND_MAX));
            vel_cand *= (ego_agent_info.max_vel / RAND_MAX);
        }

        float dist_to_pref_vel ; // distance between candidate velocity and preferred velocity
        float dist_to_cur_vel ; // distance between candidate velocity and current velocity
        if(is_collision) {
            dist_to_pref_vel = 0;
            dist_to_cur_vel = 0;
        }
        else {
            dist_to_pref_vel = abs(vel_cand - vel_pref);
            dist_to_cur_vel = abs(vel_cand - vel_curr);
        }

        // searching for smallest time to collision with this velocity sample
        float min_t_to_collision = RVO_INFTY;
        
        // iterate over neighbors
        for(const auto& n: neighbors_list) {


            // If neighbor is an obstacle, agent_Radius, position and other attributes would change
            // Change code accordingly
            float t_to_collision; // time to collision with neighbor
            RVO::Vector2 vel_a_to_b;
            RVO::Vector2 vel_b = n.currrent_velocity;
            vel_a_to_b = vel_cand - vel_b;
            RVO::Vector2 neigh_pos = n.current_position;
            float time;
            if(isHoming){
              time = rvoTimeToCollision(pos_curr, vel_a_to_b, neigh_pos, RVO_RADIUS_MULT_FACTOR_HOMING*RVO_AGENT_RADIUS, is_collision);
            }else{
              time = rvoTimeToCollision(pos_curr, vel_a_to_b, neigh_pos, RVO_RADIUS_MULT_FACTOR*RVO_AGENT_RADIUS, is_collision);
            }
            if(is_collision)  {
                t_to_collision = -std::ceil(time / TIME_STEP);
                t_to_collision -= absSq(vel_cand) / (ego_agent_info.max_vel*ego_agent_info.max_vel);

            }
            else
            {
                t_to_collision = time;
            }
            if(t_to_collision<min_t_to_collision) {

                min_t_to_collision = t_to_collision;
                if(RVO_SAFETY_FACTOR / min_t_to_collision + dist_to_pref_vel + dist_to_cur_vel>= min_penalty)
                    break;
            }
        }
        float penalty = RVO_SAFETY_FACTOR / min_t_to_collision + dist_to_pref_vel + dist_to_cur_vel;
        if(penalty < min_penalty)
        {
            min_penalty = penalty;
            vel_computed = vel_cand;
        }

        // if(true) {
        //     ROS_INFO("*** %f %f  %f %f",penalty, min_t_to_collision, dist_to_pref_vel, dist_to_cur_vel);
        // }

    }
    // ROS_INFO("Computed Velocity: %f %f ", vel_computed.x(), vel_computed.y());
    // ROS_INFO(" ");
    // ROS_INFO(" ");

    return vel_computed;
}

inline RVO::Vector2 flockControlVelocity(rvo_agent_obstacle_info_s ego_agent_info,
                                         const std::vector<rvo_agent_obstacle_info_s>& repulsion_list, RVO::Vector2& rvo_velocity)
{
  RVO::Vector2 vel_computed;
  RVO::Vector2 dist_vect;
  const RVO::Vector2 ego_pos = ego_agent_info.current_position;

  for (const auto &n : repulsion_list)
  {
    dist_vect = ego_pos - n.current_position;
    vel_computed += dist_vect; //TODO: Repulsion inversely proportional to distance?
  }
  if(!repulsion_list.empty()) {
    vel_computed = norm(vel_computed)*ego_agent_info.max_vel/2;
    return vel_computed + rvo_velocity;
  }
  else
    return rvo_velocity;
}

inline RVO::Vector2 flockControlVelocity_weighted(rvo_agent_obstacle_info_s ego_agent_info,
                                         const std::vector<rvo_agent_obstacle_info_s>& repulsion_list, RVO::Vector2& rvo_velocity)
{
  RVO::Vector2 vel_computed;
  RVO::Vector2 dist_vect;
  const RVO::Vector2 ego_pos = ego_agent_info.current_position;

  for (const auto &n : repulsion_list)
  {
    dist_vect = ego_pos - n.current_position;
    //vel_computed += dist_vect; //larger the distance, larger the addition in repulsion, doesn't make sense
    vel_computed += (1-abs(dist_vect)/0.5f)*norm(dist_vect);
  }
  if(!repulsion_list.empty()) {
    vel_computed = norm(vel_computed)*ego_agent_info.max_vel/2;
    return vel_computed + rvo_velocity;
  }
  else
    return rvo_velocity;
}
#endif // LAZY_TRAFFIC_RVO_H