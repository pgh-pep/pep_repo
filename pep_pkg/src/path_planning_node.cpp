#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class PathPlanningNode : public rclcpp::Node
{
public:
    PathPlanningNode()
    : Node("path_planning"), count_(0)
    {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "ackermann_cmd", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathPlanningNode::publishAckermannCommand, this));
    }

private:
    void publishAckermannCommand()
    {
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "base_link";
        
        message.drive.steering_angle = 0.0;
        message.drive.speed = 1.0;
        
        publisher_->publish(message);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanningNode>());
  rclcpp::shutdown();
  return 0;
}