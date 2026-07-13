#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // Include the AckermannDriveStamped message type from the ackermann_msgs package
using namespace std::chrono_literals; // Use the std::chrono_literals namespace to enable the use of time literals like 500ms

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("talker")
    {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10); // Create a publisher for the "drive" topic with a queue size of 10
        timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::timer_callback, this)); // Create a timer that calls the timer_callback function every 500 milliseconds
        this->declare_parameter("v", 0.0); // Declare a parameter "v" with a default value of 0.0
        this->declare_parameter("d", 0.0); // Declare a parameter "d" with a default value of 0.0
    }
    void timer_callback()
    {
        
        double v = this->get_parameter("v").as_double(); // Get the value of the parameter "v" as a double
        double d = this->get_parameter("d").as_double(); // Get the value of the parameter "d" as a double
        auto message = ackermann_msgs::msg::AckermannDriveStamped(); // Create a new AckermannDriveStamped message
        message.drive.speed = v; // Set the speed of the message to the value of "v"
        message.drive.steering_angle = d; // Set the steering angle of the message to the value of "d"
        publisher_->publish(message); // Publish the message to the "drive" topic
        RCLCPP_INFO(this->get_logger(), "Publishing: speed='%f', steering_angle='%f'", v, d); // Log the published speed and steering angle values
    }
private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_; // Declare a shared pointer for the publisher
  rclcpp::TimerBase::SharedPtr timer_; // Declare a shared pointer for the timer
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 system
  auto nodo = std::make_shared<PublisherNode>(); // Create a shared pointer for the PublisherNode
  rclcpp::spin(nodo); // Spin the node to process callbacks (ctrl + c to stop)
  rclcpp::shutdown(); // Shutdown the ROS 2 system
  return 0;
}
