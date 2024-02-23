#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>


#include "rclcpp/rclcpp.hpp"
#include "segwayrmp/segwayrmp.h"
//#include "ros2_segway_rmp/msg/segway_status.hpp"
//#include "ros2_segway_rmp/msg/segway_status_stamped.hpp"
#include "SegwayStatusStamped.h"
#include "std_msgs/msg/header.hpp"


class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;
static double degrees_to_radians = M_PI / 180.0;

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss);




// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{
  public:

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
      std::shared_ptr<rclcpp::Node> n = rclcpp::Node::make_shared("ros2_segway_rmp_node");
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
