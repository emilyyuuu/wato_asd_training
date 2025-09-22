#include "planner_node.hpp"
#include <queue>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(PATH_TIMEOUT), std::bind(&PlannerNode::timerCallback, this));
  
  state_ = State::WAITING_FOR_GOAL;

  current_map_ = nav_msgs::msg::OccupancyGrid();
  goal_ = geometry_msgs::msg::PointStamped();
  robot_pose_ = geometry_msgs::msg::Pose();
}

//when new map is recieved, update current map and if robot reaches goal, recalculate path
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::REACHING_GOAL) {
    planPath();
  }
}

//called whenevr robot recieves new goal
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  
  //stores recieved goal point
  goal_ = *msg;

  //marks as recieved
  goal_received_ = true;

  //changes state
  state_ = State::REACHING_GOAL;

  //calls on function to get path to reach new goal 
  planPath();
}

//called whenever robot has new position
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  //stores the robot's current pose
  robot_pose_ = msg->pose.pose;
}

//periodically checks if robot is moving towards a goal
void PlannerNode::timerCallback() {
  if (state_ == State::REACHING_GOAL) {

    //checks to see if robot is close enough to goal
    if (goalReached()) {

      //if reached, prints message and changes state back to waiting for goal 
      RCLCPP_INFO(this->get_logger(), "Goal Reached!");
      state_ = State::WAITING_FOR_GOAL;

      //if not reached, will call on plan path functino
      //if map chances or obstacle appears, might need new path 
    } else {
      // RCLCPP_INFO(this->get_logger(), "Replanning path...");
      planPath();
    }
  }
}

//checks whenever the robot has reached its goal
bool PlannerNode::goalReached() {

  //calculates the distance from the robot to goal
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;

  //compares distance to radius around goal that is close enough 
  return std::hypot(dx, dy) < SETTLE_RADIUS;
}

void PlannerNode::planPath() {
  
  //if no goal or map is empty, warn that cannot plan path
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal");
    return;
  }
  
  //creates path message that will hold the sequence of points
  nav_msgs::msg::Path path;

  //sets timestamp 
  path.header.stamp = this->now();

  //sets coordinate frame / grid that path is defined in
  path.header.frame_id = "sim_world";

  //holds the path in grid space (cells in occupancy grid) 
  //uses grid for obstacle checking, A* can use cells in 2D array, etc
  std::vector<CellIndex> path_cells;
  
  //gets map size 
  int width = current_map_.info.width;
  int height = current_map_.info.height;
  
  //finds the robot's grid cell location by taking how far the robot is from the origin and dividing by resolution
  int init_x = static_cast<int>(std::round((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution));
  int init_y = static_cast<int>(std::round((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution));
  
  //creates object to hold robot's current position (start node of A*)
  CellIndex init_idx(init_x, init_y);
  
  //does the same but with the goal's grid cell location
  int goal_x = static_cast<int>(std::round((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution));
  int goal_y = static_cast<int>(std::round((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution));
  CellIndex goal_idx(goal_x, goal_y);

  //keeps track of parent cell that led to where each cell came from
  //A* constructs path by going backwards 
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

 //stores f-cost (A* uses f to decide which cell to "expand" which is taking node out of queue and explore ways to move from there)
  std::unordered_map<CellIndex, double, CellIndexHash> f_scores;

  //stores g-cost (cost of how start cell to current cell)
  std::unordered_map<CellIndex, double, CellIndexHash> g_scores;
  
  //checks if inittal robot position is in the map
  if (init_x < 0 || init_x >= width || init_y < 0 || init_y >= height) {

    //will warn and exit function if out of map range
    RCLCPP_WARN(this->get_logger(), "Robot position is outside map bounds: grid(%d, %d), map size(%d, %d)", init_x, init_y, width, height);
    return;
  }

  //same but with goal position
  if (goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
    RCLCPP_WARN(this->get_logger(), "Goal position is outside map bounds: grid(%d, %d), map size(%d, %d)", goal_x, goal_y, width, height);
    return;
  }
   
  //priority queue (open set) that gives the node with the lowest score 
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> pq;
  
  //iniatlizes with robot's start cell as first entry 
  pq.push(AStarNode(init_idx, 0));

  //iniatlizes the f and g scores 
  f_scores[init_idx] = std::hypot(init_idx.x - goal_x, init_idx.y - goal_y);
  g_scores[init_idx] = 0;

  //loops for all nodes still to explore 
  while (!pq.empty()) {

    //takes the node with lowest f
    CellIndex idx = pq.top().idx;
    int x = idx.x, y = idx.y;
    pq.pop(); //removes it from the queue to process 
    
    //if node is goal cell, will backtrack from goal to start to build final path and break out of loop
    if (idx == goal_idx) {
      reconstructPath(came_from, idx, path_cells);
      break;
    }
    
    //loop to check all 8 neighboring to the current cell
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {

        //skips invalid case (if in the same cell or out of bounds)
        if (dx == 0 && dy == 0) continue;
        if (x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) continue;
        
        //define neighbor cell
        CellIndex new_idx(x + dx, y + dy);
         
        //looks for occupancy grid value
        int cell_value = current_map_.data[(y + dy) * width + (x + dx)];
      
        //if straight, cost to move is 1 and if diagonale, cost is sqrt2 since diagonals longer
        double movement_cost = (dx == 0 || dy == 0) ? 1.0 : sqrt(2);
        
        //how expensive it is to reach neighboring cell
        double g = g_scores[idx] + movement_cost + cell_value * COST_WEIGHTING;
        
        //if never visted this neighbor or found cheaper path 
        if (g_scores.find(new_idx) == g_scores.end() || g < g_scores[new_idx]) {
          
          //store best cost to reach this neighbor
          g_scores[new_idx] = g;

          //finds h-cost (straight line distance from this neighbor to goal)
          double h = std::hypot((x + dx) - goal_x, (y + dy) - goal_y);

          //total estimated cost (what A* will use to pick which cell to look at next)
          f_scores[new_idx] = g + h;

          //records where the new cell came from 
          came_from[new_idx] = idx;

          //push neighbor into priority queue so will expand to node with lowest f-score next
          pq.push(AStarNode(new_idx, f_scores[new_idx]));
        }
      }
    }
  }

  //removes any old path 
  path.poses.clear();

  //convert each cell in path to robot's position and orientation
  for (auto& cell : path_cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    
    //convert grid coordinates back to world positions (meters and relative to where map starts)
    double x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
    double y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    
    //adds pose to ROS path message
    path.poses.push_back(pose);
  }
  
  //check if path exist (if A* failed there will be no path)
  if (path_cells.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found to goal!");
    return;
  }
  
  //publish path 
  path_pub_->publish(path);
}

//function to go from goal to robot origin using how we got to each cell and store in path (vector/sequence of grid cells)
void PlannerNode::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, CellIndex current, std::vector<CellIndex>& path) {
  
  //clears any old path and adds the goal cell as starting point 
  path.clear();
  path.push_back(current);
  
  //looks up parent cell and moves current cell to parent and then inserts it at the front of path
  //repeats until we reach the start cell (no parent cell)
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.insert(path.begin(), current);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
