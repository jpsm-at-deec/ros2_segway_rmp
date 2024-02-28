#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

#include "segway_interfaces/msg/segwaystatus.hpp"     
#include "segway_interfaces/msg/stamped.hpp"  

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"


//std::shared_ptr<rclcpp::Node>  n = rclcpp::Node::make_shared("ros2_segway_rmp_node");

// ROS2 Node class
class SegwayRMPNode : public rclcpp::Node{

  rclcpp::Publisher<segway_interfaces::msg::Stamped>::SharedPtr segway_status_pub;    
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_velSubscriber;    
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;  
  bool optionaldebug = true;
  
  public:
    SegwayRMPNode() : Node("ros2_segway_rmp_node") {        
      if (this->optionaldebug) { 
        std::cout << "SegwayRMPNode class init\n";         
      };
      //n = rclcpp::Node::make_shared("ros2_segway_rmp_node");
      this->setupROSComms();
      if (this->optionaldebug) { 
        std::cout << "SegwayRMPNode class init done\n";  
      };
    }
    ~SegwayRMPNode() {
    }
    void setupROSComms() {
      if (this->optionaldebug) {
        std::cout << "setupROSComms call\n";  
      };
      //this->segway_status_pub = n->create_publisher<segway_interfaces::msg::Stamped>("segway_status", 1000);      
      //this->cmd_velSubscriber;// = n->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), std::bind(&SegwayRMPNode::cmd_velCallback, this, std::placeholders::_1));
      //this->odom_pub = n->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
      if (this->optionaldebug) {
        std::cout << "setupROSComms call done\n";  
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
