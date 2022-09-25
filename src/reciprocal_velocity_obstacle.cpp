// Created by Indraneel and Naren on 19/09/22

#include "reciprocal_velocity_obstacle.hpp"
#include "robosar_messages/agent_status.h"
using namespace RVO;
using namespace std;

Vector2 RecVelocityObs::computeNewVelocity() {

    Vector2 vel_cand;
    Vector2 vel_computed;
    float min_penalty = INFINITY;
    for(int i=0;i<NUM_VELOCITY_SAMPLES;i++)
    {

        if(i==0) 
        {
            vel_cand = vel_pref_;
        }
        else 
        {
            do 
            {
                vel_cand = Vector2(2.0f*rand() - RAND_MAX, 2.0f*rand() - RAND_MAX);
            } while(absSq(vel_cand) > sqrt((float) RAND_MAX));
            vel_cand *= (MAX_SPEED / RAND_MAX);
        }

        float dist_to_pref_vel ;
        if(is_collision_)
        {
            dist_to_pref_vel = 0;
        }
        else
        {
            dist_to_pref_vel = abs(vel_cand - vel_pref_);
        }
        float min_t_to_collision = INFINITY;
        for(auto n: neighbors_)
        {
            // If neighbor is an obstacle, agent_Radius, position and other attributes would change
            // Change code accordingly
            float t_to_collision;
            Vector2 vel_a_to_b;
            Vector2 vel_b = velocity_vector_[n.first];
            vel_a_to_b = 2*vel_cand - vel_curr_ - vel_b;
            
            float time = RecVelocityObs::timeToCollision(pos_curr_, vel_a_to_b, position_vector_[n.first], RADIUS_MULT_FACTOR*AGENT_RADIUS, is_collision_);
            if(is_collision_) 
            {
                t_to_collision = -ceil(time / TIME_STEP);
                t_to_collision -= absSq(vel_cand) / (MAX_SPEED*MAX_SPEED);

            }
            else
            {
                t_to_collision = time;
            }
            if(t_to_collision<min_t_to_collision)
            {
                min_t_to_collision = t_to_collision;
                if(SAFETY_FACTOR / min_t_to_collision + dist_to_pref_vel>= min_penalty)
                    break;
            }
        }
        float penalty = SAFETY_FACTOR / min_t_to_collision + dist_to_pref_vel;
        if(penalty < min_penalty)
        {
            min_penalty = penalty;
            vel_computed = vel_cand;
        }

    }
    return vel_computed;
}

float RecVelocityObs::timeToCollision(const Vector2& ego_position, const Vector2& vel_a_to_b, const Vector2& neighbor_position, float obstacle_radius, bool& is_in_collision) {
    Vector2 minkowski_diff = neighbor_position - ego_position;
    float sq_diam = obstacle_radius * obstacle_radius;
    float time;

    float discr = -sqrt(det(vel_a_to_b, minkowski_diff)) + sq_diam * absSq(vel_a_to_b);
    if (discr > 0) 
    {
      if (is_in_collision) 
      {
        time = (vel_a_to_b* minkowski_diff + sqrt(discr)) / absSq(vel_a_to_b);
        if (time < 0) {
          time = -INFINITY;
        }
      } 
      else 
      {
        time = (vel_a_to_b* minkowski_diff - sqrt(discr)) / absSq(vel_a_to_b);
        if (time < 0) 
        {
          time = INFINITY;
        }
      }
    } 
    else 
    {
      if (is_in_collision) 
      {
        time = -INFINITY;
      } 
      else 
      {
        time = INFINITY;
      }
    } 
    return time;
}
void RecVelocityObs::computeNearestNeighbors()
{
    priority_queue<dist, vector<dist>, greater<dist>> all_agents;
    for(auto agent:agent_map_)
    {
        
        Vector2 pos_neigh_agent(agent.second.current_pose_.transform.translation.x, agent.second.current_pose_.transform.translation.y);
        string agent_name = agent.first;
        float euc_distance = euc_dist(pos_neigh_agent,pos_curr_);
        if(euc_distance < MAX_NEIGH_DISTANCE)
            all_agents.push(make_pair(agent_name,euc_distance));
        // compute euclidian distance between ego agent and every other agent
        // if(dist < max_dist)
        //      add to priority queue
        // prune the priority queue based on max_number of neighbors
    }
    for(int i=0;i<MAX_NEIGHBORS;i++)
    {
        
        if(all_agents.empty()) 
            break;
        dist agent_pair = all_agents.top();
        all_agents.pop();
        neighbors_[i] = agent_pair;
    }
}
