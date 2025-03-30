#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace cyberbot_controller
{

const int JOINT_LEFT_FRONT = 0;
const int JOINT_LEFT_REAR = 1;
const int JOINT_RIGHT_FRONT = 2;
const int JOINT_RIGHT_REAR = 3;

const int PASSIVE_LEFT_FRONT = 4;
const int PASSIVE_LEFT_REAR = 5;
const int PASSIVE_RIGHT_FRONT = 6;
const int PASSIVE_RIGHT_REAR = 7;

class CyberbotController : public controller_interface::ControllerInterface
{
public:
  CyberbotController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::LoanedCommandInterface> joint_command_interfaces_;

    std::vector<std::string> joint_names_ = {
    "joint_left_front",
    "joint_left_rear",
    "joint_right_front",
    "joint_right_rear",
    "passive_left_front",
    "passive_left_rear",
    "passive_right_front",
    "passive_right_rear",    
    };


  //rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_subscriber_;
  double target_height_ = 200.0;

  void height_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  std::vector<double> compute_joint_positions_from_height(double height);
};

}  // namespace cyberbot_controller
