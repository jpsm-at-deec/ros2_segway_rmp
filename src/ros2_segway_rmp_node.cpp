#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>


#include "rclcpp/rclcpp.hpp"
#include "segwayrmp/segwayrmp.h"


// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{
  public:
    segwayrmp::SegwayRMP * segway_rmp = NULL;
    segwayrmp::InterfaceType interface_type;
    segwayrmp::SegwayRMPType segway_rmp_type;

    
    /****************************************/
    SegwayRMPNode() : Node("ros2_segway_rmp_node") {
      this->segway_rmp = NULL;
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
      this->setupSegwayRMP();
    }
    /*--------------------------------------*/

    /****************************************/
    void setupSegwayRMP() {
      this->interface_type = segwayrmp::InterfaceType::serial;
      this->segway_rmp_type = segwayrmp::SegwayRMPType::rmp200;
      this->segway_rmp = new segwayrmp::SegwayRMP(this->interface_type, this->segway_rmp_type);
    }
    /*--------------------------------------*/

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegwayRMPNode>());
  rclcpp::shutdown();
  return 0;
}
