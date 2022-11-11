// Created by Indraneel on 22/09/22

#include "lazy_traffic_agent.hpp"

#define PI (3.14159265)
#define CONTROL_ANGLE_THRESHOLD (PI/2.0)
#define CONTROL_ANGLE_THRESHOLD_INIT (0.17) //10 degrees
#define USE_STATE_MACHINE (true)
void Agent::stopAgent(void) {
  // TODO Check if velocity is non zero
  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.angular.z = 0.0;
  pub_vel_.publish(vel);
  at_rest = true;
}


// Write a function to clear the paths 

void Agent::clearPath(void){
  
  std::queue<geometry_msgs::PoseStamped>().swap(current_path_);

}
void Agent::sendVelocity(RVO::Vector2 velo) {

  // Check if velocity is non zero
  if (AreSame(velo.x(), 0.0) && AreSame(velo.y(), 0.0)) {
    return;
  }
  // Update status because sending non zero velocity
  status.data = status.BUSY;

  // TODO Put limits on acceleration

  geometry_msgs::Twist vel;

  // Get heading diff with a dot product
  RVO::Vector2 heading = getCurrentHeading();
  RVO::Vector2 velo_norm = norm(velo);
  // Calculate cross product
  double cross_product = heading.x() * velo_norm.y() - heading.y() * velo_norm.x();
  // Calculate dot product
  vel.angular.z = acos(heading * velo_norm);

  // Map linear velocity based on error in angular velocity
  vel.linear.x = 0.0 + v_max_ * (1.0 - fabs(vel.angular.z) / CONTROL_ANGLE_THRESHOLD);
  vel.linear.x = fabs(vel.angular.z)>CONTROL_ANGLE_THRESHOLD ? 0.0 : vel.linear.x;
  vel.linear.x = at_rest && fabs(vel.angular.z)>CONTROL_ANGLE_THRESHOLD_INIT ? 0.0 : vel.linear.x;
  at_rest = AreSame(vel.linear.x, 0.0) ? true : false;

  vel.angular.z = std::min(fabs(vel.angular.z), w_max_);
  vel.angular.z = copysign(vel.angular.z, cross_product);
  
  //vel.linear.x = v_max_;
  pub_vel_.publish(vel);

  //ROS_INFO("[LT_CONTROLLER-%s]: Sent Velo LIN: %f ANG: %f", &name_[0], vel.linear.x, vel.angular.z);
}
void Agent::rotateInPlace() {
  
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = SEARCH_ANGULAR_VELOCITY;
    // vel.angular.x = 0.0;
    // vel.angular.y = 0.0;
    vel.linear.y = 0.0;
    pub_vel_.publish(vel);
    ROS_DEBUG("Rotating in place");
  }

void Agent::updatePreferredVelocity()
{

  if (current_path_.empty())
 {
    preferred_velocity_ = RVO::Vector2(0.0, 0.0);
    // stopAgent();
    // return;
  }
  else if (current_path_.size() == 1 && checkifGoalReached()) {

    
    preferred_velocity_ = RVO::Vector2(0.0, 0.0);
    if(goal_type_ == robosar_messages::task_graph_getter::Response::COVERAGE && agent_state_!=ROTATION_COMPLETED) {
      
      switch (agent_state_)
      {
        case TRACKING:
          ROS_WARN("[LT_CONTROLLER-%s] Coverage goal received, entering turn in place! ", &name_[0]);
          agent_state_ = ROTATION;
          rot_count_ = 0;
          pause_count_ = 0;
        case ROTATION:
          rotateInPlace();
          rot_count_++;
          ROS_WARN("[LT_CONTROLLER-%s] Search task in rotation state %d ", &name_[0],rot_count_);
          if(rot_count_ == SEARCH_ROTATION_TIMESTEPS*SEARCH_NUM_ROTATIONS){
            agent_state_ = ROTATION_COMPLETED;
          }
          else if(rot_count_%SEARCH_ROTATION_TIMESTEPS==0)
          {
            agent_state_ = SEARCHING;
          }
          break;
        case SEARCHING:
          stopAgent();
          pause_count_++;
          ROS_WARN("[LT_CONTROLLER-%s] ________________________________", &name_[0]);
          if(pause_count_==SEARCH_PAUSE_TIMESTEPS )
          {
            agent_state_ = ROTATION;
            pause_count_ = 0;
          }
          break;
        default:
          ROS_ERROR("Invalid state %d", agent_state_);
      }
     
    }
    else if(goal_type_ != robosar_messages::task_graph_getter::Response::COVERAGE || agent_state_ == ROTATION_COMPLETED) {
      current_path_.pop();
      preferred_velocity_ = RVO::Vector2(0.0, 0.0);
      stopAgent();
      ROS_WARN("[LT_CONTROLLER-%s] Goal reached!", &name_[0]);
      status.data = status.SUCCEEDED;
      agent_state_ =   TRACKING;
    }
    else {
      ROS_WARN("[LT_CONTROLLER-%s]: Undefined agent state %d \n",&name_[0],agent_state_);
    }
   
  }
  else
  {
    ppProcessLookahead(current_pose_.transform);

    // Calculate preferred velocity vector from current pose to lookahead point
    preferred_velocity_ = RVO::Vector2(lookahead_.transform.translation.x - current_pose_.transform.translation.x,
                                       lookahead_.transform.translation.y - current_pose_.transform.translation.y);
    preferred_velocity_ = norm(preferred_velocity_);
    preferred_velocity_ *= v_max_;
    ROS_INFO("[LT_CONTROLLER-%s]: Preferred Velo X: %f Y: %f", &name_[0], preferred_velocity_.x(), preferred_velocity_.y());
    publishPreferredVelocityMarker();
  }

}

//! Helper founction for computing eucledian distances in the x-y plane.
template <typename T1, typename T2>
double distance(T1 pt1, T2 pt2)
{
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

void Agent::ppProcessLookahead(geometry_msgs::Transform current_pose)
{

  // for (; pp_idx_ < cartesian_path_.poses.size(); pp_idx_++)
  int pp_idx = 0;
  // Find closest point on path and pop all the previous points
  std::queue<geometry_msgs::PoseStamped> local_path_ = current_path_;
  double min_dist = INFINITY;
  int min_pp_idx = 0;
  while (!local_path_.empty())
  {
    double dist_to_front_point = distance(local_path_.front().pose.position, current_pose.translation); 
    if (dist_to_front_point < min_dist)
    {
      min_dist = dist_to_front_point;
      min_pp_idx = pp_idx;
    }
    local_path_.pop();
    pp_idx++;
  }

  // pop points till min pp_idx
  for (int idx = 0; idx < min_pp_idx; idx++)
  {
    current_path_.pop();
  }

  while (current_path_.size() > 1)
  {
    double dist_to_path = distance(current_path_.front().pose.position, current_pose.translation);
    if (dist_to_path > ld_)
    {
      // Save this as the lookahead point
      lookahead_.transform.translation.x = current_path_.front().pose.position.x;
      lookahead_.transform.translation.y = current_path_.front().pose.position.y;

      // TODO: See how the above conversion can be done more elegantly
      // using tf2_kdl and tf2_geometry_msgs
      ROS_INFO("[LT_CONTROLLER-%s]: Lookahead X: %f Y: %f", &name_[0], lookahead_.transform.translation.x, lookahead_.transform.translation.y);
      return;
    }
    else
    {
      current_path_.pop();
    }
  }

  if (!current_path_.empty())
  {

    // Lookahead point is the last point in the path
    lookahead_.transform.translation.x = current_path_.front().pose.position.x;
    lookahead_.transform.translation.y = current_path_.front().pose.position.y;

    ROS_INFO("[LT_CONTROLLER-%s]:***** Lookahead X: %f Y: %f", &name_[0], lookahead_.transform.translation.x, lookahead_.transform.translation.y);
  }
  else
  {
    ROS_ERROR("[LT_CONTROLLER-%s]: No path to follow. Stopping agent.", &name_[0]);
  }
}
// If goal reached, ask robot to spin around once 

bool Agent::checkifGoalReached()
{

  double distance_to_goal = distance(current_pose_.transform.translation, current_path_.front().pose.position);
  if (distance_to_goal <= goal_threshold_)
  {
    return true;
  }
  else
    return false;
}

RVO::Vector2 Agent::getCurrentHeading()
{

  tf2::Quaternion quat(current_pose_.transform.rotation.x, current_pose_.transform.rotation.y, current_pose_.transform.rotation.z, current_pose_.transform.rotation.w);

  // Convert to RPY
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // Convert to unit vector
  RVO::Vector2 heading(cos(yaw), sin(yaw));
  myheading_ = heading;
  return heading;
}

void Agent::invokeRVO(std::unordered_map<std::string, Agent> agent_map, const nav_msgs::OccupancyGrid& ocm) {
  // Dont invoke RVO if the preferred velocity is zero
  // or if there is no path to follow
  if ((AreSame(preferred_velocity_.x(), 0.0) && AreSame(preferred_velocity_.y(), 0.0)) ||
       current_path_.empty()) {
    rvo_velocity_ = RVO::Vector2(0.0, 0.0);
    return;
  }
  bool isCollision = false;
  // Calculate dynamic and static neighbours
  isCollision = computeNearestNeighbors(agent_map);
  computeStaticObstacles(ocm);

  RVO::Vector2 current_position(current_pose_.transform.translation.x, current_pose_.transform.translation.y);
  // Create new self structure for RVO
  rvo_agent_obstacle_info_s my_info{name_, current_velocity_, preferred_velocity_, current_position, v_max_};
  
  //ROS_INFO("[LT_CONTROLLER-%s]: Neighbours: %ld", &name_[0], neighbors_list_.size());

  // Calculate new velocity
  if(!isCollision) {
    rvo_velocity_ = rvoComputeNewVelocity(my_info, neighbors_list_);
  } else {
    rvo_velocity_ = rvoComputeNewVelocity(my_info, neighbors_list_);
    rvo_velocity_ = flockControlVelocity_weighted(my_info, repulsion_list_, rvo_velocity_);
  }

  publishVOVelocityMarker(isCollision);
  publishHeading();
  // Handle the calculated velocity
  ROS_INFO("[LT_CONTROLLER-%s]: RVO Velo X: %f Y: %f", &name_[0], rvo_velocity_.x(), rvo_velocity_.y());
}

void Agent::staticObstacleBfs(const RVO::Vector2& start, const std::vector<int8_t>& map_data, 
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

    float dist = euclidean_dist(current_position, start_position);
    if(dist > MAX_STATIC_OBS_DIST) {
      //ROS_INFO("Outside radius. Exiting obstacle search");
      break;
    }

    // Check if obstacle
    if (map_data[current.first + current.second*map_width] > 0) {
      
      // Add to obstacle list
      //ROS_INFO("adding static obstacle at %f, %f", current_position.x(), current_position.y());
      obstacle_count++;
      rvo_agent_obstacle_info_s obs;
      obs.agent_name = "obstacle"+std::to_string(obstacle_count);
      RVO::Vector2 obs_pos(current_position.x(), current_position.y());
      obs.current_position = obs_pos;
      obs.currrent_velocity = RVO::Vector2(0.0,0.0);
      neighbors_list_.push_back(obs);
      // repulsion_list_.push_back(obs); // TODO : Will the other agents at home be considered obstacles?
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

void Agent::computeStaticObstacles(const nav_msgs::OccupancyGrid& new_map) {

  if(USE_STATIC_OBSTACLE_AVOIDANCE != 1)
    return;

  // Get the map resolution
  float map_resolution = new_map.info.resolution;

  // Get the map origin
  geometry_msgs::Point map_origin;
  map_origin.x = new_map.info.origin.position.x;
  map_origin.y = new_map.info.origin.position.y;

  // Get the map dimensions
  int map_width = new_map.info.width;
  int map_height = new_map.info.height;

  // Get the map data
  std::vector<int8_t> map_data;
  map_data.clear();
  map_data.resize(map_width*map_height);
  map_data = new_map.data;
  RVO::Vector2 current_position(current_pose_.transform.translation.x, current_pose_.transform.translation.y);
  //call bfs on agent to detect static obstacles
  staticObstacleBfs(current_position, map_data, map_width, map_height, map_resolution, map_origin);

}
bool Agent::computeNearestNeighbors(std::unordered_map<std::string, Agent> agent_map)
{
  bool result = false;
  priority_queue<AgentDistPair, vector<AgentDistPair>, greater<AgentDistPair>> all_neighbors;
  std::vector<std::string> repulsion_neighbours;
  RVO::Vector2 my_pose(current_pose_.transform.translation.x, current_pose_.transform.translation.y);
  neighbors_list_.clear();
  repulsion_list_.clear();

  // Crop agents based on distance
  for (const auto &agent : agent_map) {
    
    // Ignore self
    if(agent.first == name_)
      continue;

    RVO::Vector2 neigh_agent_pos(agent.second.current_pose_.transform.translation.x, agent.second.current_pose_.transform.translation.y);
    string neighbour_agent_name = agent.first;
    float euc_distance = euclidean_dist(neigh_agent_pos, my_pose);
    if(euc_distance < REPULSION_RADIUS) {
      if(!(AreSame(agent_map[neighbour_agent_name].preferred_velocity_.x(),0.0) &&
         AreSame(agent_map[neighbour_agent_name].preferred_velocity_.y(),0.0))) {
        result = true;
        repulsion_neighbours.push_back(neighbour_agent_name);
      }
    }
    if (euc_distance < MAX_NEIGH_DISTANCE)
      all_neighbors.push(make_pair(neighbour_agent_name, euc_distance));
  }

  // Crop agents based on max neighbours
  for (int i = 0; i < MAX_NEIGHBORS && !all_neighbors.empty(); i++) {

    // Get the nearest neighbor from the priority queue
    AgentDistPair agent_dist_pair = all_neighbors.top();
    all_neighbors.pop();

    // Create and add the nearest neighbor to the list of neighbors
    rvo_agent_obstacle_info_s neigh;
    neigh.agent_name = agent_dist_pair.first;
    RVO::Vector2 neigh_agent_pos(agent_map[neigh.agent_name].current_pose_.transform.translation.x, 
                                  agent_map[neigh.agent_name].current_pose_.transform.translation.y);
    neigh.current_position = neigh_agent_pos;
    neigh.currrent_velocity = agent_map[neigh.agent_name].current_velocity_;
    neigh.preferred_velocity = agent_map[neigh.agent_name].preferred_velocity_;
    neigh.max_vel = agent_map[neigh.agent_name].v_max_;
    neighbors_list_.push_back(neigh);
  }

  for(int i=0;i<repulsion_neighbours.size();i++) {
    rvo_agent_obstacle_info_s neigh;
    neigh.agent_name = repulsion_neighbours[i];
    if(AreSame(agent_map[neigh.agent_name].preferred_velocity_.x(),0.0) && AreSame(agent_map[neigh.agent_name].preferred_velocity_.y(),0.0))
      continue;
    RVO::Vector2 neigh_agent_pos(agent_map[neigh.agent_name].current_pose_.transform.translation.x,
                                  agent_map[neigh.agent_name].current_pose_.transform.translation.y);
    neigh.current_position = neigh_agent_pos;
    neigh.currrent_velocity = agent_map[neigh.agent_name].current_velocity_;
    neigh.preferred_velocity = agent_map[neigh.agent_name].preferred_velocity_;
    neigh.max_vel = agent_map[neigh.agent_name].v_max_;
    repulsion_list_.push_back(neigh);
  }

  return result;
}

void Agent::publishPreferredVelocityMarker(void) {

  // update marker and publish it on ROS
  vel_marker_.header.stamp = ros::Time();
  vel_marker_.id = vel_marker_.id + 1;
  vel_marker_.pose.position.x = current_pose_.transform.translation.x;
  vel_marker_.pose.position.y = current_pose_.transform.translation.y;
  vel_marker_.pose.position.z = 0.0;

  // Set the orientation from preferred velocity direction
  double yaw = atan2(preferred_velocity_.y(), preferred_velocity_.x());
  tf2::Matrix3x3 rot;
  rot.setEulerYPR(yaw,0.0,0.0);
  tf2::Quaternion quat;
  rot.getRotation(quat);
  vel_marker_.pose.orientation.x = quat.x();
  vel_marker_.pose.orientation.y = quat.y();
  vel_marker_.pose.orientation.z = quat.z();
  vel_marker_.pose.orientation.w = quat.w();

  vel_marker_.color.r = 1.0;
  vel_marker_.color.g = 0.0;
  vel_marker_.color.b = 0.0;

  // Publish the marker
  vel_marker_pub_.publish(vel_marker_);
}

void Agent::publishVOVelocityMarker(bool flag) {

  // update marker and publish it on ROS
  vel_marker_.header.stamp = ros::Time();
  vel_marker_.id = vel_marker_.id + 1;
  vel_marker_.pose.position.x = current_pose_.transform.translation.x;
  vel_marker_.pose.position.y = current_pose_.transform.translation.y;
  vel_marker_.pose.position.z = 0.0;

  // Set the orientation from preferred velocity direction
  double yaw = atan2(rvo_velocity_.y(), rvo_velocity_.x());
  tf2::Matrix3x3 rot;
  rot.setEulerYPR(yaw,0.0,0.0);
  tf2::Quaternion quat;
  rot.getRotation(quat);
  vel_marker_.pose.orientation.x = quat.x();
  vel_marker_.pose.orientation.y = quat.y();
  vel_marker_.pose.orientation.z = quat.z();
  vel_marker_.pose.orientation.w = quat.w();

  if(flag) {
    vel_marker_.color.r = 0.0;
    vel_marker_.color.g = 1.0;
    vel_marker_.color.b = 0.0;
  } else {
    vel_marker_.color.r = 0.0;
    vel_marker_.color.g = 0.0;
    vel_marker_.color.b = 1.0;
  }

  // Publish the marker
  vel_marker_pub_.publish(vel_marker_);
}

void Agent::publishHeading() {

  // update marker and publish it on ROS
  vel_marker_.header.stamp = ros::Time();
  vel_marker_.id = vel_marker_.id + 1;
  vel_marker_.pose.position.x = current_pose_.transform.translation.x;
  vel_marker_.pose.position.y = current_pose_.transform.translation.y;
  vel_marker_.pose.position.z = 0.0;

  // Set the orientation from preferred velocity direction
  double yaw = atan2(myheading_.y(), myheading_.x());
  tf2::Matrix3x3 rot;
  rot.setEulerYPR(yaw,0.0,0.0);
  tf2::Quaternion quat;
  rot.getRotation(quat);
  vel_marker_.pose.orientation.x = quat.x();
  vel_marker_.pose.orientation.y = quat.y();
  vel_marker_.pose.orientation.z = quat.z();
  vel_marker_.pose.orientation.w = quat.w();

    vel_marker_.color.r = 1.0;
    vel_marker_.color.g = 1.0;
    vel_marker_.color.b = 0.0;

  // Publish the marker
  vel_marker_pub_.publish(vel_marker_);
}