#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <optional>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    
    //keeps track of current path the robot is following
    nav_msgs::msg::Path::SharedPtr current_path_;

    //where the current robot is 
    nav_msgs::msg::Odometry::SharedPtr odom_;
    
    //lookahead distance (looks ahead of this distance to determine where to aim next)
    static constexpr double ld_ = 0.5;
    static constexpr double goal_tolerance_ = 0.1;
    //base linear velocity (how faast the robot moves forward)
    static constexpr double linear_vel_ = 1.0;
    
    //delta time for the time interval between updates of controller
    static constexpr int dt = 100;
    
    //what you multiply by distance to make sure robot doesn't overshoot
    //robot moves closer, linear_kp will be slower
    static constexpr double linear_kp = 1.0;
    static constexpr double angular_kp = 0.8;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
     
    
    void controlLoop();

    //lookahead point for robot to follow 
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();

    //velocity to move robot towards goal
    geometry_msgs::msg::Twist computeVel(const geometry_msgs::msg::PoseStamped& tgt);
    
    //measures distance to lookahead point 
    double computeDist(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

    //converts quaternion to yaw angle to know which direction robot is facing
    double extractYaw(const geometry_msgs::msg::Quaternion& q);
};

#endif
