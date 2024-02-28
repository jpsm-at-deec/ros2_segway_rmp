#include <iostream>
#include "rclcpp/rclcpp.hpp"

// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{
  public:
    SegwayRMPNode() : Node("ros2_segway_rmp_node") { 
      std::cout << "wally\n";  
    }
    ~SegwayRMPNode() {
    }
};

int main(int argc, char * argv[])
{
  std::cout << "wally\n";  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegwayRMPNode>());
  rclcpp::shutdown();
  return 0;
}
