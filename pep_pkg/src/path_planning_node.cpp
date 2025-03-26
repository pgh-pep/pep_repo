#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "path_planning/goal_points_generator.cpp"
#include "path_planning/goal_looper.cpp"
#include <array>

class PathPlanningNode : public rclcpp::Node
{
public:
    PathPlanningNode()
    : Node("path_planning"), count_(0)
    {
        // Declare parameters with default values
        this->declare_parameter<double>("top_buoy_x", 0.0);
        this->declare_parameter<double>("top_buoy_y", 0.0);
        this->declare_parameter<double>("bottom_buoy_x", 0.0);
        this->declare_parameter<double>("bottom_buoy_y", 0.0);

        // Get parameter values
        this->get_parameter("top_buoy_x", top_buoy_x_);
        this->get_parameter("top_buoy_y", top_buoy_y_);
        this->get_parameter("bottom_buoy_x", bottom_buoy_x_);
        this->get_parameter("bottom_buoy_y", bottom_buoy_y_);

        RCLCPP_INFO(this->get_logger(), "Top Buoy: (%f, %f)", top_buoy_x_, top_buoy_y_);
        RCLCPP_INFO(this->get_logger(), "Bottom Buoy: (%f, %f)", bottom_buoy_x_, bottom_buoy_y_);

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "ackermann_cmd", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathPlanningNode::publishAckermannCommand, this));

        // Generate goal points based on buoy positions
        std::array<std::array<double,2>, 6> goal_points_ = generateGoalPoints();
        
        // Initialize the GoalLooper with the generated goal points
        this->goal_looper = std::make_shared<GoalLooper>(goal_points_);   
        current_position_ = {top_buoy_x_, top_buoy_y_};
    }

private:
    std::array<std::array<double, 2>, 6> generateGoalPoints()
    {
        GoalPointsGenerator* goal_points_generator_ = new GoalPointsGenerator();

        std::array<std::array<double, 2>, 6> goal_points_ = goal_points_generator_->get_points(top_buoy_x_, top_buoy_y_, bottom_buoy_x_, bottom_buoy_y_);

        RCLCPP_INFO(this->get_logger(), "Generated goal points:");
        for (const auto& point : goal_points_) {
            RCLCPP_INFO(this->get_logger(), "Point: (%f, %f)", point[0], point[1]);
        }
        delete goal_points_generator_;
        return goal_points_;
    }

    void publishAckermannCommand()
    {
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "base_link";

        auto next_goal = this->goal_looper->loopGoals(current_position_);
        RCLCPP_INFO(this->get_logger(), "Current Position: (%f, %f)", current_position_[0], current_position_[1]);
        RCLCPP_INFO(this->get_logger(), "Next Goal: (%f, %f)", next_goal[0], next_goal[1]);
        
        message.drive.steering_angle = 0.0;
        message.drive.speed = 1.0;

        publisher_->publish(message);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GoalLooper> goal_looper;
    double top_buoy_x_;
    double top_buoy_y_;
    double bottom_buoy_x_;
    double bottom_buoy_y_;
    std::array<double, 2> current_position_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanningNode>());
  rclcpp::shutdown();
  return 0;
}