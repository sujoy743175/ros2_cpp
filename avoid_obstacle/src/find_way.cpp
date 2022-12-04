#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"    // Twist
#include "rclcpp/rclcpp.hpp"              // ROS Core Libraries
#include "sensor_msgs/msg/laser_scan.hpp" // Laser Scan
#include <unistd.h>// for delay



#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
unsigned sleep(unsigned seconds);
int fwd_distance_threshould;
int lft_distance_threshould;
int rt_distance_threshould;
int left_limit_state;
int right_limit_state;
int stop_or_move;
int fdw_limit = 25;
int side_limit = 15;
//int wheelState = 0;

class Obstacle_Avoidance : public rclcpp::Node
{
  public:
    Obstacle_Avoidance()
    : Node("obstacle_avoid"), count_(0)
    {
      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

      stop_or_move_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "stop_or_move", 1, std::bind(&Obstacle_Avoidance::stop_or_move_callback, this, _1));
      

      fwd_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "fwd_distance", 1, std::bind(&Obstacle_Avoidance::fwd_distance_callback, this, _1));
      

      left_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "left_distance", 1, std::bind(&Obstacle_Avoidance::left_distance_callback, this, _1));
      

      right_distance_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "right_distance", 1, std::bind(&Obstacle_Avoidance::right_distance_callback, this, _1));
      

      left_limit_state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "left_limit_state", 1, std::bind(&Obstacle_Avoidance::left_limit_state_callback, this, _1));
      

      right_limit_state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "right_limit_state", 1, std::bind(&Obstacle_Avoidance::right_limit_state_callback, this, _1));
          

    timer_ = this->create_wall_timer(
      100ms, std::bind(&Obstacle_Avoidance::timer_callback, this));
    }


 private:
    void timer_callback()
    {
      auto message = this->calculateVelMsg();
      //cmd_vel_publisher_ ->publish(message);
    }

    void stop_or_move_callback(const std_msgs::msg::Int32 & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Stop or Move: '%d'", msg.data);
      stop_or_move = msg.data;
    }

    void fwd_distance_callback(const std_msgs::msg::Float32 & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Distance forward: '%f'", msg.data);
      fwd_distance_threshould = msg.data;
    }

        void left_distance_callback(const std_msgs::msg::Float32 & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Distance Left: '%f'", msg.data);
      lft_distance_threshould = msg.data;
    }

        void right_distance_callback(const std_msgs::msg::Float32 & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Distance Right: '%f'", msg.data);
      rt_distance_threshould = msg.data;
    }

        void left_limit_state_callback(const std_msgs::msg::Int32 & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Left Limit State '%d'", msg.data);
      left_limit_state = msg.data;
    }

        void right_limit_state_callback(const std_msgs::msg::Int32 & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Right Limit State: '%d'", msg.data);
      right_limit_state = msg.data;  
    }
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stop_or_move_subscription_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fwd_distance_subscription_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_distance_subscription_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_distance_subscription_;
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_limit_state_subscription_;
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_limit_state_subscription_;
  
  

    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


    geometry_msgs::msg::Twist forward() {
      using namespace std::chrono_literals;
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = 0.2;
      msg.angular.z = 0.0;      
      RCLCPP_INFO(this->get_logger(), "Move Forward");      
      cmd_vel_publisher_ ->publish(msg);
    }
 
    geometry_msgs::msg::Twist reverse() {
      using namespace std::chrono_literals;
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = -0.2;
      msg.angular.z = 0.0;      
      RCLCPP_INFO(this->get_logger(), "Go Reverse");
      cmd_vel_publisher_ ->publish(msg);
      sleep(.5);            
    }
    geometry_msgs::msg::Twist Stop() {
      using namespace std::chrono_literals;
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;      
      RCLCPP_INFO(this->get_logger(), "Stop");
      cmd_vel_publisher_ ->publish(msg);
      sleep(.2);          
    }
    geometry_msgs::msg::Twist left() {
      using namespace std::chrono_literals;
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = 0.0;
      msg.angular.z = 0.2;      
      RCLCPP_INFO(this->get_logger(), "Turn Left");
      cmd_vel_publisher_ ->publish(msg);
      sleep(.5);            
    }
    geometry_msgs::msg::Twist right() {
      using namespace std::chrono_literals;
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = 0.0;
      msg.angular.z = -0.2;      
      RCLCPP_INFO(this->get_logger(), "Turn Right");
      cmd_vel_publisher_ ->publish(msg);      sleep(.5);
      
    }

  geometry_msgs::msg::Twist calculateVelMsg() {

    // logic

    if (lft_distance_threshould == 0) {
      lft_distance_threshould = side_limit;
    }
   
    if (rt_distance_threshould == 0) {
      rt_distance_threshould = side_limit;
    }

    if (fwd_distance_threshould == 0) {
      fwd_distance_threshould = fdw_limit;
    }
        
    if (fwd_distance_threshould >= fdw_limit && lft_distance_threshould >= side_limit && 
    rt_distance_threshould >= side_limit && left_limit_state != 0 
    && right_limit_state !=0 /*&& wheelState != 0 && wheelState != 10*/) {
      forward();
    }

    /*else if (wheelState == 0) {
      Stop();
      reverse();
      Stop();
      right();
      Stop();
    }

    else if (wheelState == 10) {            
      Stop();
      reverse();
      Stop();
      right();
      Stop();
    }*/


    else if (left_limit_state ==0) {           
      Stop();
      reverse();
      Stop();
      right();
      Stop();
    }
             
        
    else if (right_limit_state ==0) {           
      Stop();
      reverse();
      Stop();
      left();
      Stop();
    }

    else if (fwd_distance_threshould < fdw_limit  && lft_distance_threshould>rt_distance_threshould) {;            
      Stop();
      reverse();
      Stop();
      left();
      Stop();
    }
              

    else if (fwd_distance_threshould < fdw_limit  && lft_distance_threshould<rt_distance_threshould) {
      Stop();
      reverse();
      Stop();
      right();
      Stop();
    }
            

    else if (fwd_distance_threshould < fdw_limit && lft_distance_threshould==rt_distance_threshould) {
      Stop();
      reverse();
      Stop();
      left();
      Stop();
    }
            

    else if (fwd_distance_threshould >= fdw_limit && lft_distance_threshould<side_limit 
    && rt_distance_threshould>side_limit) {
      Stop();
      reverse();
      Stop();
      right();
      Stop();
    }
            

    else if (fwd_distance_threshould >= fdw_limit && rt_distance_threshould<side_limit 
    && lft_distance_threshould>side_limit) {
      Stop();
      reverse();
      Stop();
      left();
      Stop();
    }           

            

    else if (fwd_distance_threshould >= fdw_limit && rt_distance_threshould<side_limit 
    && lft_distance_threshould<side_limit) {
      Stop();
      reverse();
      Stop();
      left();
      Stop();
    }

        
   /* else:
            pass*/



    //RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);
    //if (distance < 1) {
      // turn around
    //msg.linear.x = 0;
    //msg.angular.z = 0.3;
    //} else {
      // go straight ahead
    //  msg.linear.x = 0.3;
    //  msg.angular.z = 0;
    //}
    //return msg;
  }
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