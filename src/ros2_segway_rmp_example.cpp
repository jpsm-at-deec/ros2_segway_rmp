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

class SegwayRMPNode;
static SegwayRMPNode * segwayrmp_node_instance;

static double degrees_to_radians = M_PI / 180.0;
static double radians_to_degrees = 180.0 / M_PI;

void handleDebugMessages(const std::string &msg) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_debug"), "%s",msg.c_str());
}
void handleInfoMessages(const std::string &msg) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_info"), "%s",msg.c_str());
}
void handleErrorMessages(const std::string &msg) {
  RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"), "%s",msg.c_str());
}

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss);

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
      this->linear_pos_accel_limit /= 20;
      this->linear_neg_accel_limit /= 20;
      this->angular_pos_accel_limit /= 20;
      this->angular_neg_accel_limit /= 20;   
      n = rclcpp::Node::make_shared("ros2_segway_rmp_node");
      /*--*/
      this->setupROSComms();
      this->setupSegwayRMP();
      this->run();
      /*--*/
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
        std::cout << "[wally] cmd_velCallback call done\n";  
      };
    }
    void motor_timeoutCallback(){
      if (this->optionaldebug) {
        std::cout << "[wally] motor_timeoutCallback call\n";  
      };
      boost::mutex::scoped_lock lock(m_mutex);      
      this->target_linear_vel = 0.0;
      this->target_angular_vel = 0.0;
      if (this->optionaldebug) {
        std::cout << "[wally] motor_timeoutCallback call done\n";  
      };
    }
    void setupSegwayRMP() {
      if (this->optionaldebug) {
        std::cout << "[wally] setupSegwayRMP call\n";  
      };
      std::stringstream ss;
      ss << "Connecting to Segway RMP via ";      
      this->interface_type = segwayrmp::InterfaceType::serial;
      this->segway_rmp_type = segwayrmp::SegwayRMPType::rmp200;
      this->serial_port = "ttyUSB0";
      this->segway_rmp = new segwayrmp::SegwayRMP(this->interface_type, this->segway_rmp_type);      
      ss << "serial on serial port: " << this->serial_port;
      this->segway_rmp->configureSerial(this->serial_port);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp_info"), "%s", ss.str().c_str());
      segwayrmp_node_instance = this;
      this->segway_rmp->setStatusCallback(handleStatusWrapper);
      this->segway_rmp->setLogMsgCallback("rclcpp_debug", handleDebugMessages);
      this->segway_rmp->setLogMsgCallback("rclcpp_info", handleInfoMessages);
      this->segway_rmp->setLogMsgCallback("rclcpp_error", handleErrorMessages);      
      if (this->optionaldebug) {
        std::cout << "[wally] setupSegwayRMP call done\n";  
      };
    }
    void handleStatus(segwayrmp::SegwayStatus::Ptr &ss_ptr) {
      if (this->optionaldebug) {
        std::cout << "[wally] handleStatus call\n";  
      };
      if (!this->connected) {
          return;
      }      
      rclcpp::Time current_time = rclcpp::Clock(RCL_ROS_TIME).now();
      this->sss_msg.header.stamp = current_time;
      segwayrmp::SegwayStatus &ss = *(ss_ptr);
      if (this->reset_odometry) {
        if ((current_time - this->odometry_reset_start_time).seconds() < 0.25) {
          return;
        }
        if (fabs(ss.integrated_forward_position) < 1e-3 &&
            fabs(ss.integrated_turn_position) < 1e-3 &&
            fabs(ss.integrated_left_wheel_position) < 1e-3 &&
            fabs(ss.integrated_right_wheel_position) < 1e-3) {
          this->initial_integrated_forward_position = ss.integrated_forward_position;
          this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
          this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
          this->initial_integrated_turn_position = ss.integrated_turn_position;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp_info"),"Integrators reset by Segway RMP successfully");
          this->reset_odometry = false;
        } else if ((current_time - this->odometry_reset_start_time).seconds() > this->odometry_reset_duration) {
          this->initial_integrated_forward_position = ss.integrated_forward_position;
          this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
          this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
          this->initial_integrated_turn_position = ss.integrated_turn_position; 
          RCLCPP_INFO(rclcpp::get_logger("rclcpp_info"),"Integrator reset by Segway RMP failed. Performing software reset");           
          this->reset_odometry = false;
        } else {
          return; // continue waiting for odometry to be reset
        }
      }
      this->sss_msg.segway.pitch_angle = ss.pitch * degrees_to_radians;
      this->sss_msg.segway.pitch_rate = ss.pitch_rate * degrees_to_radians;
      this->sss_msg.segway.roll_angle = ss.roll * degrees_to_radians;
      this->sss_msg.segway.roll_rate = ss.roll_rate * degrees_to_radians;
      this->sss_msg.segway.left_wheel_velocity = ss.left_wheel_speed;
      this->sss_msg.segway.right_wheel_velocity = ss.right_wheel_speed;
      this->sss_msg.segway.yaw_rate = ss.yaw_rate * degrees_to_radians;
      this->sss_msg.segway.servo_frames = ss.servo_frames;
      this->sss_msg.segway.left_wheel_displacement = ss.integrated_left_wheel_position - this->initial_integrated_left_wheel_position;
      this->sss_msg.segway.right_wheel_displacement = ss.integrated_right_wheel_position - this->initial_integrated_right_wheel_position;
      this->sss_msg.segway.forward_displacement = ss.integrated_forward_position - this->initial_integrated_forward_position;
      this->sss_msg.segway.yaw_displacement = (ss.integrated_turn_position - this->initial_integrated_turn_position) * degrees_to_radians;
      this->sss_msg.segway.left_motor_torque = ss.left_motor_torque;
      this->sss_msg.segway.right_motor_torque = ss.right_motor_torque;
      this->sss_msg.segway.operation_mode = ss.operational_mode;
      this->sss_msg.segway.gain_schedule = ss.controller_gain_schedule;
      this->sss_msg.segway.ui_battery = ss.ui_battery_voltage;
      this->sss_msg.segway.powerbase_battery = ss.powerbase_battery_voltage;
      this->sss_msg.segway.motors_enabled = (bool)(ss.motor_status);

      segway_status_pub->publish(this->sss_msg);

      // Grab the newest Segway data
      float forward_displacement = (ss.integrated_forward_position - this->initial_integrated_forward_position) * this->linear_odom_scale;
      float yaw_displacement = (ss.integrated_turn_position - this->initial_integrated_turn_position) * degrees_to_radians * this->angular_odom_scale;
      float yaw_rate = ss.yaw_rate * degrees_to_radians;

      // Integrate the displacements over time
      // If not the first odometry calculate the delta in displacements
      float vel_x = 0.0;
      float vel_y = 0.0;
      if(!this->first_odometry) {
        float delta_forward_displacement = forward_displacement - this->last_forward_displacement;
        double delta_time = (current_time-this->last_time).seconds();
        // Update accumulated odometries and calculate the x and y components 
        // of velocity
        this->odometry_w = yaw_displacement;
        float delta_odometry_x = delta_forward_displacement * std::cos(this->odometry_w);
        vel_x = delta_odometry_x / delta_time;
        this->odometry_x += delta_odometry_x;
        float delta_odometry_y = delta_forward_displacement * std::sin(this->odometry_w);
        vel_y = delta_odometry_y / delta_time;
        this->odometry_y += delta_odometry_y;
      } else {
        this->first_odometry = false;
      }
      // No matter what update the previouse (last) displacements
      this->last_forward_displacement = forward_displacement;
      this->last_yaw_displacement = yaw_displacement;
      this->last_time = current_time;

      // Create a Quaternion from the yaw displacement      
      geometry_msgs::msg::Quaternion quat = this->createQuaternionMsgFromYaw(yaw_displacement);

      // Publish the Transform odom->base_link
      if (this->broadcast_tf) {
        this->odom_trans.header.stamp = current_time;            
        this->odom_trans.transform.translation.x = this->odometry_x;
        this->odom_trans.transform.translation.y = this->odometry_y;
        this->odom_trans.transform.translation.z = 0.0;
        this->odom_trans.transform.rotation = quat;
            
        //send the transform
        this->odom_broadcaster->sendTransform(this->odom_trans);
      }
      // Publish Odometry
      this->odom_msg.header.stamp = current_time;
      this->odom_msg.pose.pose.position.x = this->odometry_x;
      this->odom_msg.pose.pose.position.y = this->odometry_y;
      this->odom_msg.pose.pose.position.z = 0.0;
      this->odom_msg.pose.pose.orientation = quat;
      this->odom_msg.pose.covariance[0] = 0.00001;
      this->odom_msg.pose.covariance[7] = 0.00001;
      this->odom_msg.pose.covariance[14] = 1000000000000.0;
      this->odom_msg.pose.covariance[21] = 1000000000000.0;
      this->odom_msg.pose.covariance[28] = 1000000000000.0;
      this->odom_msg.pose.covariance[35] = 0.001;        
      this->odom_msg.twist.twist.linear.x = vel_x;
      this->odom_msg.twist.twist.linear.y = vel_y;
      this->odom_msg.twist.twist.angular.z = yaw_rate;
        
      this->odom_pub->publish(this->odom_msg);
      if (this->optionaldebug) {
        std::cout << "[wally] handleStatus call done\n";  
      };
    }
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
    {
      if (this->optionaldebug) {
        std::cout << "[wally] createQuaternionMsgFromYaw call\n";  
      };
      geometry_msgs::msg::Quaternion q;
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, yaw);
      q.x = myQuaternion.getX();
      q.y = myQuaternion.getY();
      q.z = myQuaternion.getZ();
      q.w = myQuaternion.getW();
      if (this->optionaldebug) {
        std::cout << "[wally] createQuaternionMsgFromYaw call done\n";  
      };
      return q;
    }
    void run() {
      if (this->optionaldebug) {
        std::cout << "[wally] run call\n";  
      };
      std::chrono::duration<double> num_minutes(1.0/20.0);      
      this->keep_alive_timer = n->create_wall_timer(500ms, std::bind(&SegwayRMPNode::keepAliveCallback, this));      
      this->odometry_reset_start_time = this->get_clock()->now();
      this->connected = false;
      while (rclcpp::ok()) {
        try {
          this->segway_rmp->connect();
          this->connected = true;
        } catch (std::exception& e) {
          std::string e_msg(e.what());
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"Exception while connecting to the SegwayRMP, check your cables and power buttons.");
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"    %s", e_msg.c_str());
          this->connected = false;
        }
        if (this->spin()) { 
          rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
      }
      if (this->optionaldebug) {
        std::cout << "[wally] run call done\n";  
      };
    }
    void keepAliveCallback() {
      if (this->optionaldebug) {
        std::cout << "[wally] keepAliveCallback call\n";  
      }; 
      if (!this->connected || this->reset_odometry) {
        return;
      }
      if (rclcpp::ok()) {
        boost::mutex::scoped_lock lock(this->m_mutex);
        // Update the linear velocity based on the linear acceleration limits
        if (this->linear_vel < this->target_linear_vel) {
          // Must increase linear speed
          if (this->linear_pos_accel_limit == 0.0 || this->target_linear_vel - this->linear_vel < this->linear_pos_accel_limit) {
            this->linear_vel = this->target_linear_vel;
          } else {
            this->linear_vel += this->linear_pos_accel_limit; 
          }
        } else if (this->linear_vel > this->target_linear_vel) {
          // Must decrease linear speed
          if (this->linear_neg_accel_limit == 0.0 || this->linear_vel - this->target_linear_vel < this->linear_neg_accel_limit) {
                    this->linear_vel = this->target_linear_vel;
          } else {
            this->linear_vel -= this->linear_neg_accel_limit; 
          }
        }
      }
      // Update the angular velocity based on the angular acceleration limits
      if (this->angular_vel < this->target_angular_vel) {
        // Must increase angular speed
        if (this->angular_pos_accel_limit == 0.0 || this->target_angular_vel - this->angular_vel < this->angular_pos_accel_limit) {
          this->angular_vel = this->target_angular_vel;
        } else {
          this->angular_vel += this->angular_pos_accel_limit;
        }
      } else if (this->angular_vel > this->target_angular_vel) {
        // Must decrease angular speed
        if (this->angular_neg_accel_limit == 0.0 || this->angular_vel - this->target_angular_vel < this->angular_neg_accel_limit) {
          this->angular_vel = this->target_angular_vel;
        } else {
          this->angular_vel -= this->angular_neg_accel_limit; 
        }
      }
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_debug"), "Sending move command: linear velocity = %f, angular velocity = %f",  this->linear_vel, this->angular_vel);
      try {
        this->segway_rmp->move(this->linear_vel, this->angular_vel);
      } catch (std::exception& e) {
        std::string e_msg(e.what());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"Error commanding Segway RMP: %s", e_msg.c_str());
        this->connected = false;
        this->disconnect();
      }
      if (this->optionaldebug) {
        std::cout << "[wally] keepAliveCallback call done\n";  
      }; 
    }
    void disconnect() {
      if (this->optionaldebug) {
        std::cout << "[wally] disconnect call \n";  
      }; 
      if (this->segway_rmp != NULL) {
        delete this->segway_rmp;
      }
      this->segway_rmp = NULL;
      if (this->optionaldebug) {
        std::cout << "[wally] disconnect call done\n";  
      }; 
    }
    bool spin() {
      if (this->optionaldebug) {
        std::cout << "[wally] spin call \n";  
      }; 
      if (rclcpp::ok() && this->connected) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_info"),"Segway RMP Ready.");
        while (rclcpp::ok() && this->connected) {
          rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
      }
      if (rclcpp::ok()) { // Error not shutdown
        return true;
      } else {         // Shutdown
        return false;
      }
      if (this->optionaldebug) {
        std::cout << "[wally] spin call done\n";  
      }; 
    }
};

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss) {
  segwayrmp_node_instance->handleStatus(ss);
}

int main(int argc, char * argv[])
{
  std::cout << "main init\n";  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegwayRMPNode>());
  rclcpp::shutdown();
  std::cout << "main done\n";  
  return 0;
}
