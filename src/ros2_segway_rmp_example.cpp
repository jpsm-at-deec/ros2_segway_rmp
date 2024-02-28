#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

#include "segway_interfaces/msg/segwaystatus.hpp"     
#include "segway_interfaces/msg/stamped.hpp"  

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/quaternion.h"

#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

#include "nav_msgs/msg/odometry.hpp"

#include "segwayrmp/segwayrmp.h"

using namespace std::chrono_literals;

static double degrees_to_radians = M_PI / 180.0;
static double radians_to_degrees = 180.0 / M_PI;

std::shared_ptr<rclcpp::Node>  n;

// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{

  rclcpp::Publisher<segway_interfaces::msg::Stamped>::SharedPtr segway_status_pub;    
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_velSubscriber;    
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;  
  bool optionaldebug = true;

  segwayrmp::SegwayRMP * segway_rmp = NULL;
  segwayrmp::InterfaceType interface_type;
  segwayrmp::SegwayRMPType segway_rmp_type;
  segway_interfaces::msg::Stamped sss_msg;  
  rclcpp::Time odometry_reset_start_time;
  rclcpp::Time last_time;
  rclcpp::TimerBase::SharedPtr keep_alive_timer;
  rclcpp::TimerBase::SharedPtr motor_timeout_timer;
  std::string serial_port;
  std::string frame_id;
  std::string odom_frame_id;
  geometry_msgs::msg::TransformStamped odom_trans;
  nav_msgs::msg::Odometry odom_msg;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  boost::mutex m_mutex;
  double initial_integrated_forward_position;
  double initial_integrated_left_wheel_position;
  double initial_integrated_right_wheel_position;
  double initial_integrated_turn_position;
  double odometry_reset_duration;
  double linear_odom_scale;
  double angular_odom_scale;
  double linear_vel;
  double angular_vel;
  double target_linear_vel;
  double target_angular_vel;
  double max_linear_vel; 
  double max_angular_vel;
  double linear_pos_accel_limit;
  double linear_neg_accel_limit;
  double angular_pos_accel_limit; 
  double angular_neg_accel_limit;
  double segway_motor_timeout;
  float last_forward_displacement;
  float last_yaw_displacement;
  float odometry_w;
  float odometry_x;
  float odometry_y;
  bool invert_x;
  bool invert_z;
  bool connected;
  bool broadcast_tf;
  bool reset_odometry;   
  bool first_odometry;
  
  public:
    SegwayRMPNode() : Node("ros2_segway_rmp_node") {        
      if (this->optionaldebug) {
        std::cout << "[wally] SegwayRMPNode class init\n";         
      };
      this->segway_rmp = NULL;
      this->connected = false;
      this->initial_integrated_forward_position = 0.0;
      this->initial_integrated_left_wheel_position = 0.0;
      this->initial_integrated_right_wheel_position = 0.0;
      this->initial_integrated_turn_position = 0.0;
      this->last_forward_displacement = 0.0;
      this->last_yaw_displacement = 0.0;
      this->odometry_w = 0.0;
      this->odometry_x = 0.0;
      this->odometry_y = 0.0;
      this->linear_vel = 0.0;
      this->angular_vel = 0.0;
      this->target_linear_vel = 0.0;
      this->target_angular_vel = 0.0;
      this->max_linear_vel = 0.0; 
      this->max_angular_vel = 0.0;
      this->linear_pos_accel_limit = 0.0;
      this->linear_neg_accel_limit = 0.0;
      this->angular_pos_accel_limit = 0.0;
      this->angular_neg_accel_limit = 0.0;
      this->linear_odom_scale = 1.0;
      this->angular_odom_scale = 1.0;
      this->odometry_reset_duration = 1.0;
      this->reset_odometry = false;
      this->segway_motor_timeout = 0.5;
      this->frame_id = std::string("base_link");
      this->odom_frame_id = std::string("odom");
      this->first_odometry = true;
      this->broadcast_tf = true;
      this->invert_x = false;
      this->invert_z = false;
      this->sss_msg.header.frame_id = this->frame_id;
      this->odom_trans.header.frame_id = this->odom_frame_id;
      this->odom_trans.child_frame_id = this->frame_id;
      this->odom_msg.header.frame_id = this->odom_frame_id;
      this->odom_msg.child_frame_id = this->frame_id;      
      n = rclcpp::Node::make_shared("ros2_segway_rmp_node");
      this->setupROSComms();
      if (this->optionaldebug) { 
        std::cout << "[wally] SegwayRMPNode class init done\n";  
      };
    }
    ~SegwayRMPNode() {
    }
    void setupROSComms() {
      if (this->optionaldebug) {
        std::cout << "[wally] setupROSComms call\n";  
      };
      this->segway_status_pub = n->create_publisher<segway_interfaces::msg::Stamped>("segway_status", 1000);      
      this->cmd_velSubscriber = n->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), std::bind(&SegwayRMPNode::cmd_velCallback, this, std::placeholders::_1));
      this->odom_pub = n->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
      if (this->optionaldebug) {
        std::cout << "[wally] setupROSComms call done\n";  
      };
    }
    void cmd_velCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { 
      if (this->optionaldebug) {
        std::cout << "[wally] cmd_velCallback call\n";  
      }; 
      if (!this->connected) {
            return;
      }
      boost::mutex::scoped_lock lock(m_mutex);
      double x = msg->linear.x, z = msg->angular.z;
      if (this->invert_x) {
          x *= -1;
      }
      if (this->invert_z) {
          z *= -1;
      }
      if (this->max_linear_vel != 0.0) {
        if (abs(x) > this->max_linear_vel) {
          x = (x > 0) ? this->max_linear_vel : -this->max_linear_vel;
        }
      }
      if (this->max_angular_vel != 0.0) {
        if (abs(z) > this->max_angular_vel) {
          z = (z > 0) ? this->max_angular_vel : -this->max_angular_vel;
        }
      }
      this->target_linear_vel = x;
      this->target_angular_vel = z * radians_to_degrees; // Convert to degrees
      this->motor_timeout_timer = n->create_wall_timer(500ms, std::bind(&SegwayRMPNode::motor_timeoutCallback, this));
      if (this->optionaldebug) {
        std::cout << "[wally] cmd_velCallback done call\n";  
      };
    }
    void motor_timeoutCallback(){
      boost::mutex::scoped_lock lock(m_mutex);
      if (this->optionaldebug) {
        std::cout << "[wally] motor_timeoutCallback done call\n";  
      };
      this->target_linear_vel = 0.0;
      this->target_angular_vel = 0.0;
      if (this->optionaldebug) {
        std::cout << "[wally] motor_timeoutCallback done call\n";  
      };
    }    
};

int main(int argc, char * argv[])
{
  std::cout << "main init\n";  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegwayRMPNode>());
  rclcpp::shutdown();
  std::cout << "main done\n";  
  return 0;
}
