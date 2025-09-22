#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    
    void updateMap();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void timerCallback();

  private:
    robot::MapMemoryCore map_memory_;
    
    //ROS messages that store 2D grid data about 2 maps 
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    
    //robot's current and previous coordinates and heading
    double robot_x_, robot_y_;
    double theta_;
    double prev_x_, prev_y_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    //publishes global map to other nodes 
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    //timer to limit frequency of map updates 
    rclcpp::TimerBase::SharedPtr timer_;
    
    //boolean to check if robot moves and map needs update AND new sensor data
    bool update_map_ = false;
    bool costmap_updated_ = false;

    //constants 
    //publishes map every second
    static constexpr uint32_t MAP_PUB_RATE = 1000;
    //updates map only if the map moved more than 1.5 meters
    static constexpr double DIST_UPDATE = 1.5;
    static constexpr double MAP_RES = 0.1;
    static constexpr int MAP_WIDTH = 300;
    static constexpr int MAP_HEIGHT = 300;
    static constexpr double MAP_ORIGIN_X = (-MAP_WIDTH)*MAP_RES/2;
    static constexpr double MAP_ORIGIN_Y = (-MAP_WIDTH)*MAP_RES/2;
};

#endif 
