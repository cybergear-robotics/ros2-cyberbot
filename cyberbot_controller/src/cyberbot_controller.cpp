#include "cyberbot_controller/cyberbot_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <math.h>

namespace cyberbot_controller
{

CyberbotController::CyberbotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn CyberbotController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "CyberbotController initialized.");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CyberbotController::command_interface_configuration() const
{
  std::vector<std::string> conf;
  for (const auto & name : joint_names_) {
    conf.push_back(name + "/position");
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    conf
  };
}

controller_interface::InterfaceConfiguration CyberbotController::state_interface_configuration() const
{
  std::vector<std::string> conf;
  for (const auto & name : joint_names_) {
    conf.push_back(name + "/position");
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    conf
  };
}

controller_interface::CallbackReturn CyberbotController::on_configure(const rclcpp_lifecycle::State &)
{
  height_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/height", 10,
    std::bind(&CyberbotController::height_callback, this, std::placeholders::_1));
  //cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
  //"/cmd_vel", 10,
  //std::bind(&CyberbotController::cmd_vel_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CyberbotController::on_activate(const rclcpp_lifecycle::State &)
{
  joint_command_interfaces_ = std::move(command_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CyberbotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  joint_command_interfaces_.clear();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CyberbotController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<double> joint_positions = compute_joint_positions_from_height(target_height_);

  if (joint_positions.size() != joint_command_interfaces_.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "Computed %ld joint positions, but have %ld interfaces.",
                joint_positions.size(), joint_command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_command_interfaces_.size(); ++i) {
    joint_command_interfaces_[i].set_value(joint_positions[i]);
  }

  return controller_interface::return_type::OK;
}

void CyberbotController::height_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  target_height_ = msg->data;
}

void CyberbotController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  target_height_ += msg->linear.z;
}

std::vector<double> CyberbotController::compute_joint_positions_from_height(double h)
{
  std::vector<double> joint_positions(joint_names_.size(), 0.0);

  /*
   *   A     a
   *    *----+----*
   *  b |\   |    |  e
   *  B * \f |h   *
   *    \  \ |    /
   *      \ \|  /  d
   *   c    \|/
   *         *
   *         C
  */
  double a = 100.0; // cm
  double b = 120.0; // cm
  double c = 180.0; // cm
  double a_2 = a / 2;

  double b_angle_offset = 19.471 * M_PI / 180;
  
  // hypotenuse (diagonale) berechnen
  double f = sqrt(a_2*a_2 + h*h);
  // unterer Teil-Winkel berechnen
  double C1_inside = asin(a_2/f);
  double A1_inside = M_PI * 0.5 - C1_inside;

  // Berechnung der Winkel mit dem Kosinussatz
  double B = acos((b*b + c*c - f*f) / (2 * b * c));
  //double C2_inside = acos((f*f + c*c - b*b) / (2 * f * b));
  double A2_inside = acos((f*f + b*b - c*c) / (2 * f * b));
  double A_inside = A1_inside + A2_inside;
  //double C_inside = C1_inside + C2_inside;

  joint_positions[JOINT_RIGHT_REAR] = A_inside - M_PI;
  joint_positions[JOINT_RIGHT_FRONT] = M_PI - A_inside;
  joint_positions[JOINT_LEFT_FRONT] = A_inside - M_PI;
  joint_positions[JOINT_LEFT_REAR] =  M_PI - A_inside;

  joint_positions[PASSIVE_RIGHT_REAR] = -B + b_angle_offset;
  joint_positions[PASSIVE_RIGHT_FRONT] = B - b_angle_offset;
  
  joint_positions[PASSIVE_LEFT_FRONT] = -B + b_angle_offset;
  joint_positions[PASSIVE_LEFT_REAR] =  B - b_angle_offset;

  return joint_positions;
}





}  // namespace cyberbot_controller

PLUGINLIB_EXPORT_CLASS(cyberbot_controller::CyberbotController, controller_interface::ControllerInterface)
