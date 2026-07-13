#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // Include the AckermannDriveStamped message type from the ackermann_msgs package
using namespace std::chrono_literals; // Use the std::chrono_literals namespace to enable the use of time literals like 500ms

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("relay")
    {
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", 10, std::bind(&SubscriberNode::listener_callback, this, std::placeholders::_1)); // Create a subscription to the "drive" topic with a queue size of 10 and bind the listener_callback function to handle incoming messages
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10); // Create a publisher for the "drive_relay" topic with a queue size of 10
    }
    void listener_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const // Define the listener_callback function that takes a shared pointer to an AckermannDriveStamped message as input
    {
        auto relay_msg = ackermann_msgs::msg::AckermannDriveStamped(); // Create a new AckermannDriveStamped message to relay the received message
        relay_msg.drive.speed = msg->drive.speed * 3; // Set the speed of the relay message to three times the speed of the received message
        relay_msg.drive.steering_angle = msg->drive.steering_angle * 3; // Set the steering angle of the relay message to three times the steering angle of the received message
        publisher_->publish(relay_msg); // Publish the relay message to the "drive_relay" topic
        RCLCPP_INFO(this->get_logger(), "Relaying: speed='%f', steering_angle='%f'", relay_msg.drive.speed, relay_msg.drive.steering_angle); // Log the relayed speed and steering angle values
    }   
private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_; // Declare a shared pointer for the publisher
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_; // Declare a shared pointer for the subscription
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 system
  auto nodo = std::make_shared<SubscriberNode>(); // Create a shared pointer for the SubscriberNode
  rclcpp::spin(nodo); // Spin the node to process callbacks (ctrl + c to stop)
  rclcpp::shutdown(); // Shutdown the ROS 2 system
  return 0; 
}
