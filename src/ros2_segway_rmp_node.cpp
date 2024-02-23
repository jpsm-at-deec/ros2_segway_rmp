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
    bool connected;
    bool reset_odometry;
    
    
    /****************************************/
    SegwayRMPNode() : Node("ros2_segway_rmp_node") {         
      std::shared_ptr<rclcpp::Node> n = rclcpp::Node::make_shared("ros2_segway_rmp_node");
      this->segway_rmp = NULL;
      this->initial_integrated_forward_position = 0.0;
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

        if (fabs(ss.integrated_forward_position) < 1e-3) {
          this->initial_integrated_forward_position = ss.integrated_forward_position;
          this->reset_odometry = false;
        }

      }
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
