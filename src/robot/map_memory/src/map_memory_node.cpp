#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  //tells ROS what function to call when new message from a specific topic (and 10 queue size)
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)); 
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(MAP_PUB_RATE), std::bind(&MapMemoryNode::timerCallback, this));


  //iniatlizing member variables 

  //creates empty occupancy grid messages to fill with map data 
  global_map_ = nav_msgs::msg::OccupancyGrid(); 
  latest_costmap_ = nav_msgs::msg::OccupancyGrid();
  //tracks the robot's current position and orientation
  robot_x_ = 0;
  robot_y_ = 0;
  theta_ = 0.0;
  //robots previous position (to detect movement)
  prev_x_ = 0;
  prev_y_ = 0;
  
  //adds timestamp of when the map was created 
  global_map_.header.stamp = this->now();
  //sets coordinate frame to sim_world (tells that the map is drawn relative to this)
  global_map_.header.frame_id = "sim_world";

  //defines map size, map placement, sets all cells as empty 
  global_map_.info.resolution = MAP_RES;
  global_map_.info.width = MAP_WIDTH; 
  global_map_.info.height = MAP_HEIGHT;
  global_map_.info.origin.position.x = -MAP_WIDTH * MAP_RES / 2.0;
  global_map_.info.origin.position.y = -MAP_HEIGHT * MAP_RES / 2.0;
  global_map_.data.assign(MAP_WIDTH * MAP_HEIGHT, 0);

  //calls function to copy data from latest costmap into the global map
  updateMap();
  //sends global map to ROS (publishing it)
  map_pub_->publish(global_map_);
}

//automatically called when new input from /costmap topic
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  latest_costmap_ = *msg; //new costmap data stored into variable (local copy)
  costmap_updated_ = true; //boolean that tells the rest of the program that there is a new costmap and to update the global map
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  
  //read current position of robot
  double curr_x = msg->pose.pose.position.x;
  double curr_y = msg->pose.pose.position.y;
  
  //calculates movement distance 
  double dist = std::hypot(curr_x - prev_x_, curr_y - prev_y_);
  //if robot hasn't moved far enough, returns 
  if (dist < DIST_UPDATE) return; 
  
  //update robot position 
  robot_x_ = curr_x;
  robot_y_ = curr_y; 
  
  //takes robot's orientation from odometry message and stores into quaternion (x, y, z w) (robot is rotated thsi way in 3D space)
  auto q_msg = msg->pose.pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

  //converts quaternion to a rotation matrix (3x3)
  tf2::Matrix3x3 m(q);

  //converts to Euler angles 
  //roll -> rotation around x axis 
  //pitch -> rotation around y axis 
  //yaw -> rotation around z-axis 
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //direction robot is facing
  theta_ = yaw;
  
  //flag map for update
  update_map_ = true;
}

void MapMemoryNode::timerCallback(){

  //check if update is needed (if robot hasn't moved enough and new new sensor data) -> returns 
  if (!update_map_ || !costmap_updated_) return; 
  
  //updates global map
  updateMap();

  //updates timestamp
  global_map_.header.stamp = this->now();

  //publishes global map
  map_pub_->publish(global_map_);
  
  //resets boolean/flags to mark update is complete and wait for next update
  update_map_ = false;
  costmap_updated_ = false;
}

void MapMemoryNode::updateMap() {

  //check if robot position is valid
  if (std::isnan(robot_x_) || std::isnan(robot_y_)) return;
  
  //takes local and global map info
  double l_res = latest_costmap_.info.resolution;
  int l_width = latest_costmap_.info.width;
  int l_height = latest_costmap_.info.height;
  auto& l_data = latest_costmap_.data;
  double g_res = global_map_.info.resolution;
  double g_origin_x = global_map_.info.origin.position.x;
  double g_origin_y = global_map_.info.origin.position.y;
  int g_width = global_map_.info.width;
  int g_height = global_map_.info.height;
  auto& g_data = global_map_.data;

  //loops through every cell in local costmap
  for (int y = 0; y < l_height; y++) {
    for (int x = 0; x < l_width; x++) {

      //converts local cell to coordinates (cell indices -> meters)
      double l_x = (x - l_width / 2) * l_res;
      double l_y = (y - l_height / 2) * l_res;
      
      //using rotational formula, translates local coordinates to global coordinates using robot's heading and position 
      double g_x = robot_x_ + (l_x * std::cos(theta_) - l_y * std::sin(theta_));
      double g_y = robot_y_ + (l_x * std::sin(theta_) + l_y * std::cos(theta_));
      
      //converts global coordinates to grid cell 
      int idx_g_x = static_cast<int>(std::round(g_x / g_res + g_width / 2));
      int idx_g_y = static_cast<int>(std::round(g_y / g_res + g_height / 2));
      
      //if index is outside the boundaries of the global map, skip 
      if (idx_g_x < 0 || idx_g_x >= g_width || idx_g_y < 0 || idx_g_y >= g_height) continue;

      //turns grid into 1D array
      int8_t& g_cost = g_data[idx_g_y * g_width + idx_g_x];

      //assigns the cost from the local costmap
      g_cost = std::max(g_cost, l_data[y * l_width + x]);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
