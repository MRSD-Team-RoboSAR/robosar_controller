#include <gtest/gtest.h>
#include <climits>
#include "Vector2.h"
#include "lazy_traffic_rvo.hpp"
#include <geometry_msgs/Point.h>

// #include <ros/ros.h>

// bad function:
// for example: how to deal with overflow?
// int add(int a, int b){
//     return a + b;
// }

#define MAX_STATIC_OBS_DIST (0.3)
std::vector<rvo_agent_obstacle_info_s> neighbors_list_;
std::vector<std::vector<int>> dir_ = {{-1 , -1}, {-1 , 0}, {-1 , 1}, {0 , -1}, {0 , 1}, {1 , -1}, {1 , 0}, {1 , 1}};

void staticObstacleBfs(const RVO::Vector2& start, const std::vector<int8_t>& map_data, 
                                const int& map_width, const int& map_height, const float& map_resolution,
                                const geometry_msgs::Point& map_origin) {
  
  // Local variables
  std::queue<std::pair<int,int>> queue;
  std::set<int> visited;
  int grids_explored=0;
  int obstacle_count = 0;

  //Convert to pixel and add it to queue
  std::pair<int,int> start_ind = std::make_pair((int)((start.x()-map_origin.x)/map_resolution),
                                                (int)((start.y()-map_origin.y)/map_resolution));
  queue.push(start_ind);
  
  while(!queue.empty()) {
    std::pair<int,int> current = queue.front();
    queue.pop();

    //if already visited, continue to next iteration
    if(visited.find((current.first + current.second*map_width)) != visited.end()) 
      continue;
    // insert into visited
    else
      visited.insert(current.first + current.second*map_width);

    // Check if reached end of BFS radius
    RVO::Vector2 start_position(start.x(), start.y());
    //convert to world coordinates
    RVO::Vector2 current_position(map_origin.x + map_resolution*(float)(current.first), 
                                  map_origin.y + map_resolution*(float)(current.second));
    std::cout<<"Checking "<<current_position.x()<<" "<<current_position.y()<<" "<<start_position.x()<<" "<<start_position.y()<<std::endl;

    float dist = euclidean_dist(current_position, start_position);
    if(dist > MAX_STATIC_OBS_DIST) {
      ROS_INFO("Outside radius. Exiting obstacle search %f %f", dist, MAX_STATIC_OBS_DIST);
      break;
    }

    // Check if obstacle
    if (map_data[current.first + current.second*map_width] > 0) {
      
      // Add to obstacle list
      ROS_INFO("adding static obstacle at %f, %f", current_position.x(), current_position.y());
      obstacle_count++;
      rvo_agent_obstacle_info_s obs;
      obs.agent_name = "obstacle"+std::to_string(obstacle_count);
      RVO::Vector2 obs_pos(current_position.x(), current_position.y());
      obs.current_position = obs_pos;
      obs.currrent_velocity = RVO::Vector2(0.0,0.0);
      neighbors_list_.push_back(obs);
    }

    //expand node
    for(const auto& dir : dir_) {
      std::pair<int,int> neighbour_ind = std::make_pair(current.first + dir[0], current.second + dir[1]);
      //Boundary checks
      if (neighbour_ind.first >= 0 && neighbour_ind.first < map_height && 
          neighbour_ind.second >= 0 && neighbour_ind.second < map_width ) {
            queue.push(neighbour_ind);
      }
    }
  }
}


TEST(NumStaticObstaclesTest, NumStaticObstaclesTest){
    
    RVO::Vector2 start(0.5,0.5);
    int map_width = 10;
    int map_height = 10;
    std::vector<int8_t> map_data;
    map_data.clear();
    map_data.resize(map_width*map_height);
    float map_resolution = 0.1;
    geometry_msgs::Point map_origin;
    map_origin.x = 0.0;
    map_origin.y = 0.0;
    map_origin.z = 0.0;
    staticObstacleBfs(start, map_data, map_width, map_height, map_resolution, map_origin);
    EXPECT_EQ(neighbors_list_.size(), 0);

    // Add some obstacles
    map_data[55] = 1;
    map_data[44] = 1;

    staticObstacleBfs(start, map_data, map_width, map_height, map_resolution, map_origin);
    EXPECT_EQ(neighbors_list_.size(), 2);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}