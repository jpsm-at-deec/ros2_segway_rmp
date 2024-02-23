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
#include "segwayrmp/segwayrmp.h"
#include "SegwayStatusStamped.h"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"



using namespace std::chrono_literals;

class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;
static double degrees_to_radians = M_PI / 180.0;

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss);

//segway_rmp::SegwayStatusStamped
template<>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = std_msgs::msg::String;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source;
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = source.data;
  }
};

using SegwayAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

//class MinimalPublisher : public rclcpp::Node
//{
//  public:
//    MinimalPublisher()
//    : Node("minimal_publisher"), count_(0)
//    {
      //publisher_ = this->create_publisher<segway_rmp::SegwayStatusStamped>("topic", 10);
//      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

//      timer_ = this->create_wall_timer(
//      500ms, std::bind(&MinimalPublisher::timer_callback, this));
//    }

//  private:
//    void timer_callback()
//    {
//      auto message = std_msgs::msg::String();
      //auto message = segway_rmp::SegwayStatusStamped();

      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//      publisher_->publish(message);
//    }
//    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Publisher<segway_rmp::SegwayStatusStamped>::SharedPtr publisher_;
//    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//    size_t count_;
//};

std::shared_ptr<rclcpp::Node>  n = rclcpp::Node::make_shared("~");
//rclcpp::Publisher<std_msgs::msg::String>::SharedPtr n_pub = n->create_publisher<std_msgs::msg::String>("topic", 10);
//rclcpp::Publisher<segwayrmp::SegwayStatus>::SharedPtr

// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{
  public:

    //rclcpp::Publisher<segwayrmp::SegwayStatus>::SharedPtr segway_status_pub;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr segway_status_pub;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr segway_status_pub = n->create_publisher<std_msgs::msg::String>("segway_status", 1000);
    //rclcpp::Publisher<segway_rmp::SegwayStatusStamped>::SharedPtr segway_status_pub = n->create_publisher<segway_rmp::SegwayStatusStamped>("segway_status", 1000);
    rclcpp::Publisher<SegwayAdaptedType>::SharedPtr segway_status_pub = n->create_publisher<SegwayAdaptedType>("segway_status", 1000);
    
    
    //MinimalPublisher segway_status_pub;
    segwayrmp::SegwayRMP * segway_rmp = NULL;
    segwayrmp::InterfaceType interface_type;
    segwayrmp::SegwayRMPType segway_rmp_type;
    segway_rmp::SegwayStatusStamped sss_msg;
    rclcpp::Time odometry_reset_start_time;
    std::string serial_port;
    double initial_integrated_forward_position;
    double initial_integrated_left_wheel_position;
    double initial_integrated_right_wheel_position;
    double initial_integrated_turn_position;
    double odometry_reset_duration;
    bool connected;
    bool reset_odometry;   
    
    
    /****************************************/
    SegwayRMPNode() : Node("ros2_segway_rmp_node") { 
      //: Node("ros2_segway_rmp_node") 
      //std::shared_ptr<rclcpp::Node>         
      n = rclcpp::Node::make_shared("ros2_segway_rmp_node");
      this->segway_rmp = NULL;
      this->initial_integrated_forward_position = 0.0;
      this->initial_integrated_left_wheel_position = 0.0;
      this->initial_integrated_right_wheel_position = 0.0;
      this->initial_integrated_turn_position = 0.0;
      this->run();
    }
    /*--------------------------------------*/
    
    /****************************************/
    ~SegwayRMPNode() {
      this->disconnect();
    }
    /*--------------------------------------*/

    /****************************************/
    void disconnect() {
      if (this->segway_rmp != NULL) {
        delete this->segway_rmp;
      }
      this->segway_rmp = NULL;
    }
    /*--------------------------------------*/

    /****************************************/
    void run() {
      this->connected = false;
      this->setupSegwayRMP();
    }
    /*--------------------------------------*/

    /****************************************/
    void setupSegwayRMP() {
      std::stringstream ss;
      ss << "Connecting to Segway RMP via ";
      this->interface_type = segwayrmp::InterfaceType::serial;
      this->segway_rmp_type = segwayrmp::SegwayRMPType::rmp200;
      this->serial_port = "ttyUSB0";
      this->segway_rmp = new segwayrmp::SegwayRMP(this->interface_type, this->segway_rmp_type);
      ss << "serial on serial port: " << this->serial_port;
      this->segway_rmp->configureSerial(this->serial_port);

      segwayrmp_node_instance = this;

      this->segway_rmp->setStatusCallback(handleStatusWrapper);
    }
    /*--------------------------------------*/    

    /****************************************/
    void setupROSComms() {
      // Advertise the SegwayStatusStamped
      //rclcpp::Publisher<segwayrmp::SegwayStatus>::SharedPtr publisher = n->advertise<segwayrmp::SegwayStatus>("segway_status", 10);

      //this->segway_status_pub = n->advertise<segway_rmp::SegwayStatusStamped>("segway_status", 1000);
      //this->segway_status_pub = n->advertise<std_msgs::msg::String>("segway_status", 1000);
      //this->segway_status_pub = n->advertise<std_msgs::msg::String>("segway_status", 1000);
      //this->segway_status_pub = n->create_publisher<std_msgs::msg::String>("segway_status", 1000);
      //this->segway_status_pub = n->create_publisher<segway_rmp::SegwayStatusStamped>("segway_status", 1000);
      this->segway_status_pub = n->create_publisher<SegwayAdaptedType>("segway_status", 1000);

      
    }
    /*--------------------------------------*/    

    /****************************************/
    void handleStatus(segwayrmp::SegwayStatus::Ptr &ss_ptr) {
      if (!this->connected)
          return;
      
      rclcpp::Time current_time = rclcpp::Clock(RCL_ROS_TIME).now();
      this->sss_msg.header.stamp = current_time;

      segwayrmp::SegwayStatus &ss = *(ss_ptr);

      if (this->reset_odometry) {
        if ((current_time - this->odometry_reset_start_time).seconds() < 0.25) {
          return; // discard readings for the first 0.25 seconds
        }

        if (fabs(ss.integrated_forward_position) < 1e-3 &&
            fabs(ss.integrated_turn_position) < 1e-3 &&
            fabs(ss.integrated_left_wheel_position) < 1e-3 &&
            fabs(ss.integrated_right_wheel_position) < 1e-3) {
          this->initial_integrated_forward_position = ss.integrated_forward_position;
          this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
          this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
          this->initial_integrated_turn_position = ss.integrated_turn_position;
          this->reset_odometry = false;
        } else if ((current_time - this->odometry_reset_start_time).seconds() > this->odometry_reset_duration) {
          this->initial_integrated_forward_position = ss.integrated_forward_position;
          this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
          this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
          this->initial_integrated_turn_position = ss.integrated_turn_position;            
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

      //segway_status_pub.publish(this->sss_msg);
    }
    /*--------------------------------------*/

};

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss) {
  segwayrmp_node_instance->handleStatus(ss);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegwayRMPNode>());
  rclcpp::shutdown();
  return 0;
}
