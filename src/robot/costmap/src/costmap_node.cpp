#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::lidarCallback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  constexpr int WIDTH = 300, HEIGHT = 300;
  constexpr double RES = 0.1;
  constexpr int MAX_COST = 100;
  constexpr double INFLATION_RADIUS = 1.5;

  std::vector<std::vector<int8_t>> costmap(WIDTH, std::vector<int8_t>(HEIGHT, 0));
  
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max || std::isnan(msg->ranges[i])) continue; 

    double angle = msg->angle_min + i * msg->angle_increment;
    double x = msg->ranges[i] * cos(angle);
    double y = msg->ranges[i] *sin(angle);
    
    int x_coord = static_cast<int>(x / RES + WIDTH / 2);
    int y_coord = static_cast<int>(y / RES + HEIGHT / 2);

    if (x_coord < 0 || x_coord >= WIDTH || y_coord < 0 || y_coord >= HEIGHT) continue;

    costmap[x_coord][y_coord] = MAX_COST;
    for (int dx = -INFLATION_RADIUS / RES; dx <= INFLATION_RADIUS / RES; dx++) {
      for (int dy = -INFLATION_RADIUS / RES; dy <= INFLATION_RADIUS / RES; dy++){
        if (x_coord + dx < 0 || x_coord + dx >= WIDTH || y_coord + dy < 0 || y_coord +dy >= HEIGHT) continue;

        double dist = sqrt(dx * dx + dy * dy) * RES;
        if (dist > INFLATION_RADIUS) continue;
        costmap[x_coord + dx] [y_coord + dy] = std::max(static_cast<int>(costmap[x_coord + dx][y_coord + dy]), static_cast<int>(MAX_COST * (1 - std::min(1.0, dist / INFLATION_RADIUS))));
      }
    }
  }

  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = msg->header.stamp;
  occupancy_grid.header.frame_id = msg->header.frame_id;
  occupancy_grid.info.resolution = RES;
  occupancy_grid.info.width = WIDTH;
  occupancy_grid.info.height = HEIGHT;
  occupancy_grid.info.origin.position.x = (-WIDTH)*RES/2;
  occupancy_grid.info.origin.position.y = (-WIDTH)*RES/2;
 
  occupancy_grid.data.resize(WIDTH * HEIGHT);
  for (int y = 0; y < HEIGHT; y++){
    for (int x = 0; x < WIDTH; x++){
      occupancy_grid.data[y * WIDTH + x] = costmap[x][y];
    }
  }

  costmap_pub->publish(occupancy_grid);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}