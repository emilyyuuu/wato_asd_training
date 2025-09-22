#include "control_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; });
  
  //send velocity commands to robot 
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(dt), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {

  //if no path or don't have current robot odometry, do not control robot amd wait
  if (!current_path_ || !odom_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for path and odometry data...");
    return;
  }

  //calculates the distance between robots current position and goal 
  double dist = computeDist(odom_->pose.pose.position, current_path_->poses.back().pose.position);
  
  //checks if robot is close enough to goal
  if (dist < goal_tolerance_) {
    //if true, prints message
    RCLCPP_INFO(this->get_logger(), "Goal reached, stopping robot.");
    
    //publishes twist message and stops the robot
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }
  
  //finding lookahead point that robot looks for 
  auto lookahead_point = findLookaheadPoint();
  
  //checks if closer to the gal than lookahead distance
  if (dist < ld_) {
    //if so, will move directly to goal 
    lookahead_point = current_path_->poses.back();
  } else if (!lookahead_point.has_value()) {
    //if not and if no valid lookahead point, have warning
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found.");
    //robot recieves command with 0 linear or angular velocity so robot stops 
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  //calculates velocity command (how to move the robot towards the point considering position, orientation, and controller kp)
  auto cmd = computeVel(lookahead_point.value());
  //sends velocity command to tell robot to move towards lookahead point
  cmd_pub_->publish(cmd);
}

//returns position and orientation if valid lookahead point found
std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  
  //looks through all points in current path that robot should follow
  for (auto& pose : current_path_->poses) {
    //distance from robots current position to path point
    double dist = computeDist(odom_->pose.pose.position, pose.pose.position);
    
    //if path point is at least look ahead distance away from robot, return as lookahead point
    if (dist >= ld_) {
      return pose;
    }
  }

  //if no point is farther than lookahead distance, return nothing (no valid lookahead point)
  return std::nullopt;
}

//pure pursuit control (computes linear and angular velocity commands to drive robot towards target point)
geometry_msgs::msg::Twist ControlNode::computeVel(const geometry_msgs::msg::PoseStamped& tgt) {
  
  //takes robot's current heading angle
  double yaw = extractYaw(odom_->pose.pose.orientation);
  //finds difference between robot and target point of x and y 
  double dx = tgt.pose.position.x - odom_->pose.pose.position.x;
  double dy = tgt.pose.position.y - odom_->pose.pose.position.y;
  
  //angle from the robot to target point
  double tgt_angle = std::atan2(dy, dx);
  
  //calculates angular error (how much robot needs to turn)
  double angle_err = tgt_angle - yaw;
  //ensures robot always turns the shortest way
  while (angle_err > M_PI) angle_err -= 2 * M_PI;
  while (angle_err < -M_PI) angle_err += 2 * M_PI;

  //computes distance from robot to final goal 
  double dist = computeDist(odom_->pose.pose.position, current_path_->poses.back().pose.position);
  
  //sets forward speed (proporitonal to distance to the goal so multipe by kp)
  geometry_msgs::msg::Twist cmd;
  //capped at linear velocity so robot doesnt go too fast
  cmd.linear.x = std::min(linear_vel_, linear_kp * dist);
  //sets rotational speed and does the same as above with angular kp 
  //if target point is the final goal, sets angular speed to 0 so robot stops rotating when done
  cmd.angular.z = tgt == current_path_->poses.back() ? 0.0 : angular_kp * angle_err;
  
  //returns velocity command to publish in /cmd_vel topic to drive robot
  return cmd;
}

//calculates straight line distance between 2 points
double ControlNode::computeDist(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

//extracts robot's heading angle from quaternion (yaw)
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  //converts ROS into TF2 
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  
  //converts quaternion into 3x3 rotational matrix
  //represents robots orientation in 3D
  tf2::Matrix3x3 m(quat);
  //extracts roll, pitch, and yaw angles but only returns yaw angle since 2D navigation
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
