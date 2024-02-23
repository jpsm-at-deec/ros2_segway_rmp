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
#include "geometry_msgs/msg/quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>



using namespace std::chrono_literals;

class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;
static double degrees_to_radians = M_PI / 180.0;

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss);

//segway_rmp::SegwayStatusStamped
template<>
struct rclcpp::TypeAdapter<segway_rmp::SegwayStatusStamped, std_msgs::msg::Header>
{
  using is_specialized = std::true_type;
  using custom_type = segway_rmp::SegwayStatusStamped;
  using ros_message_type = std_msgs::msg::Header;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    //destination.data = source.header;
    destination = source.header;
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.header = source; //.data
  }
};

//using SegwayAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;
using SegwayAdaptedType = rclcpp::TypeAdapter<segway_rmp::SegwayStatusStamped, std_msgs::msg::Header>;

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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub = n->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    
    
    //MinimalPublisher segway_status_pub;
    segwayrmp::SegwayRMP * segway_rmp = NULL;
    segwayrmp::InterfaceType interface_type;
    segwayrmp::SegwayRMPType segway_rmp_type;
    segway_rmp::SegwayStatusStamped sss_msg;
    rclcpp::Time odometry_reset_start_time;
    rclcpp::Time last_time;
    rclcpp::TimerBase::SharedPtr keep_alive_timer;
    std::string serial_port;
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
    float last_forward_displacement;
    float last_yaw_displacement;
    float odometry_w;
    float odometry_x;
    float odometry_y;
    bool connected;
    bool broadcast_tf;
    bool reset_odometry;   
    bool first_odometry;
    
    
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
      this->last_forward_displacement = 0.0;
      this->last_yaw_displacement = 0.0;
      this->odometry_w = 0.0;
      this->odometry_x = 0.0;
      this->odometry_y = 0.0;
      this->linear_odom_scale = 1.0;
      this->angular_odom_scale = 1.0;
      this->first_odometry = true;
      this->broadcast_tf = true;
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
      this->setupROSComms();

      std::chrono::duration<double> num_minutes(1.0/20.0);
      //auto thisotherthing = n->create_wall_timer(std::chrono::milliseconds(5), std::bind(&SegwayRMPNode::keepAliveCallback, this));

      this->keep_alive_timer = n->create_wall_timer(500ms, std::bind(&SegwayRMPNode::keepAliveCallback, this));
      //this->keep_alive_timer = 
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
      this->odom_pub = n->create_publisher<nav_msgs::msg::Odometry>("odom", 50);

      
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

      //std::string sss = "wally";
      //segway_status_pub->publish(sss);
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
      //geometry_msgs::msg::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw_displacement);
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
    }
    /*--------------------------------------*/

    /****************************************/
    //struct TimerEvent
    //{
    //    rclcpp::Time last_expected;                     
    //    rclcpp::Time last_real;                         
  
    //    rclcpp::Time current_expected;                  
    //    rclcpp::Time current_real;                      
  
        //struct
        //{
        //    rclcpp::WallDuration last_duration;           
        //} profile;
    //};
    //typedef boost::function<void(const TimerEvent&)> TimerCallback;
    /*--------------------------------------*/

    /****************************************/
    /**
     * This method is called at 20Hz.  Each time it sends a movement
     * command to the Segway RMP.
     */
    void keepAliveCallback() { //keepAliveCallback
      if (!this->connected || this->reset_odometry) {
        return;
      }
      if (rclcpp::ok()) {
        boost::mutex::scoped_lock lock(this->m_mutex);
      }

    }
    /*--------------------------------------*/

    /****************************************/
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
    {
      geometry_msgs::msg::Quaternion q;

      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, yaw);
      q.x = myQuaternion.getX();
      q.y = myQuaternion.getY();
      q.z = myQuaternion.getZ();
      q.w = myQuaternion.getW();
      return q;
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
