#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

//represent coordinates in a 2D grid (simple structure to identify and compare grid cells used for A* algorithm)
struct CellIndex {
  int x, y;
  
  //constructors to create cell at specific coordinate
  CellIndex(int x, int y) : x(x), y(y) {}

  //default constructor
  CellIndex() : x(0), y(0) {}
  
  //checks if 2 cells are the same 
  bool operator==(const CellIndex& other) const {
    return (x == other.x && y == other.y);
  }
  
  //checks if 2 cells are different 
  bool operator!=(const CellIndex& other) const {
    return (x != other.x || y != other.y);
  }
};

//uses hash function (turns ur object into a single unique number).
//used in A* algorithm to track which grid cell has already been visited (number makes it be looked up more efficiently)
struct CellIndexHash {
  
  //passes x and y coordinate and then returns a hashed integer value 
  //y-coordinate moves all the bits in the number one position to the left so x and y don't collide
  //& bit operation that combines the hashed x and shifted y 
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) & (std::hash<int>()(idx.y) << 1);
  }
};

//keeps track of position and A* cost so A* algorithm knows which square on the map the robot should move to 
struct AStarNode {

  //position of node 
  CellIndex idx;

  //A* cost function (f = g + h)
  double f;

  //constructor to create node by giving its position and cost
  AStarNode(CellIndex idx, double f) : idx(idx), f(f) {}
};

//tells priority queue (open list) how to order nodes
struct CompareF {

  //comparator for priority queue 
  //returns true if a has higehr f cost than b 
  //tells the priority queue to treat lower f cost nodes as higher priority 
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f > b.f;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    
    //enum -> only have a 2 values 
    enum class State {
      //robot is waiting for where to go 
      WAITING_FOR_GOAL,
      //robot is moving towards it 
      REACHING_GOAL
    };
    
    //keeps trach of current state 
    State state_;
    
    //generate and updates paths 
    robot::PlannerCore planner_;
    
    //stores latest map, goal position, current position and orientation of robot 
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    
    //checks if goal has been sent to planner
    bool goal_received_ = false;
     
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    //if robot exceeds timeout, planner will run again 
    static constexpr int PATH_TIMEOUT = 500;

    //distance around the goal considered "reached" 
    static constexpr double SETTLE_RADIUS = 0.5;

    //for A* to calculate the weight of moving through cells 
    static constexpr double COST_WEIGHTING = 0.1;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    bool goalReached();

    void planPath();
    void reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, CellIndex current, std::vector<CellIndex>& path);
};

#endif 
