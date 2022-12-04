#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"    // Twist
#include "rclcpp/rclcpp.hpp"              // ROS Core Libraries
#include "sensor_msgs/msg/laser_scan.hpp" // Laser Scan


#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class Obstacle_Avoidance : public rclcpp::Node
{
  public:
    Obstacle_Avoidance()
    : Node("obstacle_avoid"), count_(0)
    {
      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      stop_or_move_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "stop_or_move", 10, std::bind(&Obstacle_Avoidance::stop_or_move_callback, this, _1));

      fwd_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "fwd_distance", 10, std::bind(&Obstacle_Avoidance::fwd_distance_callback, this, _1));

      left_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "left_distance", 10, std::bind(&Obstacle_Avoidance::left_distance_callback, this, _1));

      right_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "right_distance", 10, std::bind(&Obstacle_Avoidance::right_distance_callback, this, _1));

      left_limit_state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "left_limit_state", 10, std::bind(&Obstacle_Avoidance::left_limit_state_callback, this, _1));

      right_limit_state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "right_limit_state", 10, std::bind(&Obstacle_Avoidance::right_limit_state_callback, this, _1));      

      timer_ = this->create_wall_timer(
      500ms, std::bind(&Obstacle_Avoidance::timer_callback, this));
    }


 private:
    void timer_callback()
    {
      auto message = this->calculateVelMsg();
      cmd_vel_publisher_ ->publish(message);
    }

    void stop_or_move_callback(const std_msgs::msg::Int32 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
    }

    void fwd_distance_callback(const std_msgs::msg::Float32 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.data);
    }

        void left_distance_callback(const std_msgs::msg::Float32 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.data);
    }

        void right_distance_callback(const std_msgs::msg::Float32 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.data);
    }

        void left_limit_state_callback(const std_msgs::msg::Int32 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
    }

        void right_limit_state_callback(const std_msgs::msg::Int32 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
    }

    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


  geometry_msgs::msg::Twist calculateVelMsg() {
    auto msg = geometry_msgs::msg::Twist();
    // logic
    //RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);
    //if (distance < 1) {
      // turn around
    msg.linear.x = 0;
    msg.angular.z = 0.3;
    //} else {
      // go straight ahead
    //  msg.linear.x = 0.3;
    //  msg.angular.z = 0;
    //}
    return msg;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stop_or_move_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fwd_distance_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_distance_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_distance_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_limit_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_limit_state_subscription_;
  
  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Obstacle_Avoidance>());
  rclcpp::shutdown();
  return 0;

}
// check following website for escription
//https://robohub.org/exploring-ros2-with-a-wheeled-robot-4-obstacle-avoidance/