#include "cirtesub_controllers/stabilize_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace cirtesub_controllers
{

controller_interface::CallbackReturn StabilizeController::on_init()
{
  try {
    auto_declare<std::string>(
      "feedforward_topic", "stabilize_controller/feedforward");
    auto_declare<std::string>(
      "navigator_topic", "/cirtesub/navigator/navigation");
    auto_declare<std::string>(
      "setpoint_topic", "stabilize/set_point");
    auto_declare<std::string>(
      "body_force_controller_name", "body_force_controller");

    auto_declare<double>("kp_roll", 0.0);
    auto_declare<double>("ki_roll", 0.0);
    auto_declare<double>("kd_roll", 0.0);

    auto_declare<double>("kp_pitch", 0.0);
    auto_declare<double>("ki_pitch", 0.0);
    auto_declare<double>("kd_pitch", 0.0);

    auto_declare<double>("kp_yaw", 0.0);
    auto_declare<double>("ki_yaw", 0.0);
    auto_declare<double>("kd_yaw", 0.0);

    auto_declare<double>("yaw_command_threshold", 1e-3);
  } catch (const std::exception &) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StabilizeController::command_interface_configuration() const
{
  const std::string prefix = body_force_controller_name_;

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      prefix + "/force.x",
      prefix + "/force.y",
      prefix + "/force.z",
      prefix + "/torque.x",
      prefix + "/torque.y",
      prefix + "/torque.z"
    }
  };
}

controller_interface::InterfaceConfiguration
StabilizeController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn StabilizeController::on_configure(
  const rclcpp_lifecycle::State &)
{
  feedforward_topic_ = get_node()->get_parameter("feedforward_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  body_force_controller_name_ =
    get_node()->get_parameter("body_force_controller_name").as_string();

  kp_roll_ = get_node()->get_parameter("kp_roll").as_double();
  ki_roll_ = get_node()->get_parameter("ki_roll").as_double();
  kd_roll_ = get_node()->get_parameter("kd_roll").as_double();

  kp_pitch_ = get_node()->get_parameter("kp_pitch").as_double();
  ki_pitch_ = get_node()->get_parameter("ki_pitch").as_double();
  kd_pitch_ = get_node()->get_parameter("kd_pitch").as_double();

  kp_yaw_ = get_node()->get_parameter("kp_yaw").as_double();
  ki_yaw_ = get_node()->get_parameter("ki_yaw").as_double();
  kd_yaw_ = get_node()->get_parameter("kd_yaw").as_double();

  yaw_command_threshold_ =
    std::max(0.0, get_node()->get_parameter("yaw_command_threshold").as_double());

  feedforward_sub_ = get_node()->create_subscription<TwistMsg>(
    feedforward_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const TwistMsg::SharedPtr msg)
    {
      feedforward_buffer_.writeFromNonRT(msg);
    });

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  setpoint_pub_ = get_node()->create_publisher<Vector3Msg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS());
  setpoint_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<Vector3Msg>>(setpoint_pub_);

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&StabilizeController::parametersCallback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StabilizeController::on_activate(
  const rclcpp_lifecycle::State &)
{
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};
  yaw_setpoint_ = 0.0;
  yaw_setpoint_initialized_ = false;
  yaw_feedforward_active_ = false;
  first_update_ = true;

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StabilizeController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};
  yaw_setpoint_ = 0.0;
  yaw_setpoint_initialized_ = false;
  yaw_feedforward_active_ = false;
  first_update_ = true;

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult StabilizeController::parametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (name == "kp_roll") {
      kp_roll_ = param.as_double();
    } else if (name == "ki_roll") {
      ki_roll_ = param.as_double();
      roll_pid_.integral = 0.0;
    } else if (name == "kd_roll") {
      kd_roll_ = param.as_double();
    } else if (name == "kp_pitch") {
      kp_pitch_ = param.as_double();
    } else if (name == "ki_pitch") {
      ki_pitch_ = param.as_double();
      pitch_pid_.integral = 0.0;
    } else if (name == "kd_pitch") {
      kd_pitch_ = param.as_double();
    } else if (name == "kp_yaw") {
      kp_yaw_ = param.as_double();
    } else if (name == "ki_yaw") {
      ki_yaw_ = param.as_double();
      yaw_pid_.integral = 0.0;
    } else if (name == "kd_yaw") {
      kd_yaw_ = param.as_double();
    } else if (name == "yaw_command_threshold") {
      yaw_command_threshold_ = std::max(0.0, param.as_double());
    }
  }

  return result;
}

double StabilizeController::computePid(
  double error,
  double dt,
  double kp,
  double ki,
  double kd,
  AxisPidState & state)
{
  state.integral += error * dt;

  double derivative = 0.0;
  if (!first_update_ && dt > 0.0) {
    derivative = (error - state.previous_error) / dt;
  }

  state.previous_error = error;

  return kp * error + ki * state.integral + kd * derivative;
}

double StabilizeController::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

controller_interface::return_type StabilizeController::update(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  auto feedforward_msg = feedforward_buffer_.readFromRT();
  auto navigator_msg = navigator_buffer_.readFromRT();

  if (!navigator_msg || !(*navigator_msg)) {
    return controller_interface::return_type::OK;
  }

  if (command_interfaces_.size() != 6) {
    return controller_interface::return_type::ERROR;
  }

  const auto & orientation = (*navigator_msg)->position.orientation;
  tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  if (!yaw_setpoint_initialized_) {
    yaw_setpoint_ = yaw;
    yaw_setpoint_initialized_ = true;
  }

  const double force_x = (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->linear.x : 0.0;
  const double force_y = (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->linear.y : 0.0;
  const double force_z = (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->linear.z : 0.0;
  const double torque_ff_x = (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->angular.x : 0.0;
  const double torque_ff_y = (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->angular.y : 0.0;
  const double yaw_feedforward = (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->angular.z : 0.0;
  const bool has_yaw_feedforward = std::abs(yaw_feedforward) > yaw_command_threshold_;

  if (has_yaw_feedforward) {
    yaw_setpoint_ = yaw;
    yaw_pid_.integral = 0.0;
  } else if (yaw_feedforward_active_) {
    yaw_setpoint_ = yaw;
    yaw_pid_.integral = 0.0;

    if (setpoint_rt_pub_ && setpoint_rt_pub_->trylock()) {
      setpoint_rt_pub_->msg_.x = 0.0;
      setpoint_rt_pub_->msg_.y = 0.0;
      setpoint_rt_pub_->msg_.z = yaw_setpoint_;
      setpoint_rt_pub_->unlockAndPublish();
    }
  }

  const double dt = period.seconds();

  const double roll_error = wrapAngle(0.0 - roll);
  const double pitch_error = wrapAngle(0.0 - pitch);
  const double yaw_error = wrapAngle(yaw_setpoint_ - yaw);

  const double torque_x =
    torque_ff_x +
    computePid(roll_error, dt, kp_roll_, ki_roll_, kd_roll_, roll_pid_);

  const double torque_y =
    torque_ff_y +
    computePid(pitch_error, dt, kp_pitch_, ki_pitch_, kd_pitch_, pitch_pid_);

  const double torque_z =
    yaw_feedforward +
    computePid(yaw_error, dt, kp_yaw_, ki_yaw_, kd_yaw_, yaw_pid_);

  command_interfaces_[0].set_value(force_x);
  command_interfaces_[1].set_value(force_y);
  command_interfaces_[2].set_value(force_z);
  command_interfaces_[3].set_value(torque_x);
  command_interfaces_[4].set_value(torque_y);
  command_interfaces_[5].set_value(torque_z);

  yaw_feedforward_active_ = has_yaw_feedforward;
  first_update_ = false;

  return controller_interface::return_type::OK;
}

}  // namespace cirtesub_controllers

PLUGINLIB_EXPORT_CLASS(
  cirtesub_controllers::StabilizeController,
  controller_interface::ControllerInterface)
