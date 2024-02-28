#include <iostream>
#include "rclcpp/rclcpp.hpp"

// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{
  public:
    SegwayRMPNode() : Node("ros2_segway_rmp_node") { 
    }
    ~SegwayRMPNode() {
    }
};

int main(int argc, char * argv[])
{
  std::cout << "wally\n";  
  return 0;
}
