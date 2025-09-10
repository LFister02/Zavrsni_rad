#include "tello_joy_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include "std_msgs/msg/bool.hpp"

namespace tello_joy
{

  TelloJoyNode::TelloJoyNode(const rclcpp::NodeOptions &options) :
    Node("tello_joy", options)
  {
    using std::placeholders::_1;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&TelloJoyNode::joy_callback, this, _1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    tello_client_ = create_client<tello_msgs::srv::TelloAction>("tello_action");
    stop_tracker_pub_ = create_publisher<std_msgs::msg::Bool>("stop_tracker", 10); # Dodan publisher tipa Bool za praćenje statusa pritisnute tipke

    bool enable_joy_input = true;

    (void) joy_sub_;
  }

  TelloJoyNode::~TelloJoyNode()
  {}


  void TelloJoyNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    // Praćenje statusa tipke A, svakim pritiskom se mijenja stanje
    static bool last_A_state = false;
    bool current_A_state = joy_msg->buttons[joy_button_input];

    if (current_A_state && !last_A_state) {
      enable_joy_input = !enable_joy_input;
      RCLCPP_INFO(this->get_logger(), "Joystick input %s", enable_joy_input ? "enabled" : "disabled");
    }

    last_A_state = current_A_state;

    if (joy_msg->buttons[joy_button_takeoff_]) {
      auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
      request->cmd = "takeoff";
      tello_client_->async_send_request(request);
      
    } else if (joy_msg->buttons[joy_button_land_]) {
      auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
      request->cmd = "land";
      tello_client_->async_send_request(request);

    } else if(enable_joy_input){
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = joy_msg->axes[joy_axis_throttle_];
      twist_msg.linear.y = joy_msg->axes[joy_axis_strafe_];
      twist_msg.linear.z = joy_msg->axes[joy_axis_vertical_];
      twist_msg.angular.z = joy_msg->axes[joy_axis_yaw_];
      cmd_vel_pub_->publish(twist_msg);
      
      
      //  Pritiskom tipke X šalje se signal za zaustavljanje drona, signal prima čvor follower
    } else if(joy_msg->buttons[joy_button_stop_]){
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      stop_tracker_pub_->publish(stop_msg);
      RCLCPP_INFO(this->get_logger(), "Stop signal sent to follower!");
    }
  }

} // namespace tello_joy

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tello_joy::TelloJoyNode)
