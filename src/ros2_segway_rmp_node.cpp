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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

//#include "ros2_segway_rmp/msg/SegwayStatus.hpp"
//#include "ros2_segway_rmp/msg/SegwayStatusStamped.hpp"


using namespace std::chrono_literals;

class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;

static double degrees_to_radians = M_PI / 180.0;
static double radians_to_degrees = 180.0 / M_PI;

void handleDebugMessages(const std::string &msg) {RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_debug"), "%s",msg.c_str());}
void handleInfoMessages(const std::string &msg) {RCLCPP_INFO(rclcpp::get_logger("rclcpp_info"), "%s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"), "%s",msg.c_str());}

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss);

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

//https://github.com/ZhenshengLee/ros2_shm_msgs/blob/42fde5f188dc5a55be54718cbb601539d2823815/intra/intra_int_node.cpp#L15
//https://github.com/ros-acceleration/acceleration_examples/tree/b63b1d8851d8d0eff91f7e9552d893d7ce868c6e/nodes/doublevadd_publisher/src


using SegwayAdaptedType = rclcpp::TypeAdapter<segway_rmp::SegwayStatusStamped, std_msgs::msg::Header>;

std::shared_ptr<rclcpp::Node>  n = rclcpp::Node::make_shared("~");

// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{
  public:

    rclcpp::Publisher<SegwayAdaptedType>::SharedPtr segway_status_pub;
    //rclcpp::Publisher<ros2_segway_rmp::msg::SegwayStatusStamped>::SharedPtr segway_status_pub;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_velSubscriber;    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;    
    
    segwayrmp::SegwayRMP * segway_rmp = NULL;
    segwayrmp::InterfaceType interface_type;
    segwayrmp::SegwayRMPType segway_rmp_type;
    segway_rmp::SegwayStatusStamped sss_msg;
    //ros2_segway_rmp::msg::SegwayStatusStamped sss_msg;
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
        
    /****************************************/
    SegwayRMPNode() : Node("ros2_segway_rmp_node") { 
     
      n = rclcpp::Node::make_shared("ros2_segway_rmp_node");

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

      if (this->getParameters()) {
        return;
      }

      
      this->setupSegwayRMP();
      this->setupROSComms();

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

    }
    /*--------------------------------------*/

    /****************************************/
    int getParameters() {

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

      // Check for valid acceleration limits
      if (this->linear_pos_accel_limit < 0) {
        //ROS_ERROR("Invalid linear positive acceleration limit of %f (must be non-negative).",
        //    this->linear_pos_accel_limit);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"Invalid linear positive acceleration limit of %f (must be non-negative).", this->linear_pos_accel_limit);
        return 1;
      }
      if (this->linear_neg_accel_limit < 0) {
        //ROS_ERROR("Invalid linear negative acceleration limit of %f (must be non-negative).",
        //    this->linear_neg_accel_limit);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"Invalid linear negative acceleration limit of %f (must be non-negative).", this->linear_neg_accel_limit);
        return 1;
      }
      if (this->angular_pos_accel_limit < 0) {
        //ROS_ERROR("Invalid angular positive acceleration limit of %f (must be non-negative).",
        //    this->angular_pos_accel_limit);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"Invalid angular positive acceleration limit of %f (must be non-negative).", this->angular_pos_accel_limit);
        return 1;
      }
      if (this->angular_neg_accel_limit < 0) {
        //ROS_ERROR("Invalid angular negative acceleration limit of %f (must be non-negative).",
        //    this->angular_neg_accel_limit);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"Invalid angular negative acceleration limit of %f (must be non-negative).", this->angular_neg_accel_limit);
        return 1;
      }
      if (this->max_linear_vel < 0) {
        //ROS_ERROR("Invalid max linear velocity limit of %f (must be non-negative).",
        //    this->max_linear_vel);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Invalid max linear velocity limit of %f (must be non-negative).", this->max_linear_vel);
        return 1;
      }
      if (this->max_angular_vel < 0) {
        //ROS_ERROR("Invalid max angular velocity limit of %f (must be non-negative).",
        //    this->max_angular_vel);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Invalid max angular velocity limit of %f (must be non-negative).", this->max_angular_vel);
        return 1;
      }
      // Convert the linear acceleration limits to have units of (m/s^2)/20 since
      // the movement commands are sent to the Segway at 20Hz.
      this->linear_pos_accel_limit /= 20;
      this->linear_neg_accel_limit /= 20;

      // Convert the angular acceleration limits to have units of (deg/s^2)/20 since
      // the movement commands are sent to the Segway at 20Hz.
      this->angular_pos_accel_limit /= 20;
      this->angular_neg_accel_limit /= 20;

      return 0;

    }
    /*--------------------------------------*/

    /****************************************/
    bool spin() {
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

      RCLCPP_INFO(rclcpp::get_logger("rclcpp_info"), "%s", ss.str().c_str());

      segwayrmp_node_instance = this;

      this->segway_rmp->setStatusCallback(handleStatusWrapper);
      this->segway_rmp->setLogMsgCallback("rclcpp_debug", handleDebugMessages);
      this->segway_rmp->setLogMsgCallback("rclcpp_info", handleInfoMessages);
      this->segway_rmp->setLogMsgCallback("rclcpp_error", handleErrorMessages);

    }
    /*--------------------------------------*/    

    /****************************************/
    
    void segwayStatusPubCallback(const segway_rmp::SegwayStatusStamped msg) {
    //void segwayStatusPubCallback(const ros2_segway_rmp::msg::SegwayStatusStamped msg) {

      this->segway_status_pub->publish(msg);

    }
    /*--------------------------------------*/    

    /****************************************/
    void setupROSComms() {

      this->segway_status_pub = n->create_publisher<SegwayAdaptedType>("segway_status", 1000);
      //this->segway_status_pub = n->create_publisher<ros2_segway_rmp::msg::SegwayStatusStamped>("segway_status", 1000);
      
      this->cmd_velSubscriber = n->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), std::bind(&SegwayRMPNode::cmd_velCallback, this, std::placeholders::_1));
      this->odom_pub = n->create_publisher<nav_msgs::msg::Odometry>("odom", 50);

    }
    /*--------------------------------------*/  

    /****************************************/
    void cmd_velCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {  

      if (!this->connected)
            return;
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

    }
    /*--------------------------------------*/  

    void motor_timeoutCallback(){

      boost::mutex::scoped_lock lock(m_mutex);
        //ROS_INFO("Motor command timeout!  Setting target linear and angular velocities to be zero.");
      this->target_linear_vel = 0.0;
      this->target_angular_vel = 0.0;

    }

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
    void keepAliveCallback() { 

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
        //ROS_ERROR("Error commanding Segway RMP: %s", e_msg.c_str());
        //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Error commanding Segway RMP: %s", e_msg.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_error"),"Error commanding Segway RMP: %s", e_msg.c_str());
        this->connected = false;
        this->disconnect();
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
