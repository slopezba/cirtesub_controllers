#include "cirtesub_controllers/depth_hold_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace cirtesub_controllers
{

controller_interface::CallbackReturn DepthHoldController::on_init()
{
  try {
    auto_declare<std::string>(
      "feedforward_topic", "depth_hold_controller/feedforward");
    auto_declare<std::string>(
      "navigator_topic", "/cirtesub/navigator/navigation");
    auto_declare<std::string>(
      "setpoint_topic", "depth_hold_controller/set_point");
    auto_declare<std::string>(
      "enable_roll_pitch_service_name", "depth_hold_controller/enable_roll_pitch");
    auto_declare<std::string>(
      "disable_roll_pitch_service_name", "depth_hold_controller/disable_roll_pitch");
    auto_declare<std::string>(
      "body_force_controller_name", "body_force_controller");

    auto_declare<bool>("allow_roll_pitch", false);
    auto_declare<double>("feedforward_gain_x", 1.0);
    auto_declare<double>("feedforward_gain_y", 1.0);
    auto_declare<double>("feedforward_gain_z", 1.0);
    auto_declare<double>("feedforward_gain_roll", 1.0);
    auto_declare<double>("feedforward_gain_pitch", 1.0);
    auto_declare<double>("feedforward_gain_yaw", 1.0);
    auto_declare<double>("kp_roll", 0.0);
    auto_declare<double>("ki_roll", 0.0);
    auto_declare<double>("kd_roll", 0.0);

    auto_declare<double>("kp_pitch", 0.0);
    auto_declare<double>("ki_pitch", 0.0);
    auto_declare<double>("kd_pitch", 0.0);

    auto_declare<double>("kp_yaw", 0.0);
    auto_declare<double>("ki_yaw", 0.0);
    auto_declare<double>("kd_yaw", 0.0);

    auto_declare<double>("kp_depth", 0.0);
    auto_declare<double>("ki_depth", 0.0);
    auto_declare<double>("kd_depth", 0.0);

    auto_declare<double>("command_threshold", 1e-3);
    auto_declare<double>("depth_command_threshold", 1e-3);
  } catch (const std::exception &) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DepthHoldController::command_interface_configuration() const
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
DepthHoldController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn DepthHoldController::on_configure(
  const rclcpp_lifecycle::State &)
{
  feedforward_topic_ = get_node()->get_parameter("feedforward_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  enable_roll_pitch_service_name_ =
    get_node()->get_parameter("enable_roll_pitch_service_name").as_string();
  disable_roll_pitch_service_name_ =
    get_node()->get_parameter("disable_roll_pitch_service_name").as_string();
  body_force_controller_name_ =
    get_node()->get_parameter("body_force_controller_name").as_string();
  allow_roll_pitch_ = get_node()->get_parameter("allow_roll_pitch").as_bool();
  feedforward_gain_x_ = get_node()->get_parameter("feedforward_gain_x").as_double();
  feedforward_gain_y_ = get_node()->get_parameter("feedforward_gain_y").as_double();
  feedforward_gain_z_ = get_node()->get_parameter("feedforward_gain_z").as_double();
  feedforward_gain_roll_ = get_node()->get_parameter("feedforward_gain_roll").as_double();
  feedforward_gain_pitch_ = get_node()->get_parameter("feedforward_gain_pitch").as_double();
  feedforward_gain_yaw_ = get_node()->get_parameter("feedforward_gain_yaw").as_double();

  kp_roll_ = get_node()->get_parameter("kp_roll").as_double();
  ki_roll_ = get_node()->get_parameter("ki_roll").as_double();
  kd_roll_ = get_node()->get_parameter("kd_roll").as_double();

  kp_pitch_ = get_node()->get_parameter("kp_pitch").as_double();
  ki_pitch_ = get_node()->get_parameter("ki_pitch").as_double();
  kd_pitch_ = get_node()->get_parameter("kd_pitch").as_double();

  kp_yaw_ = get_node()->get_parameter("kp_yaw").as_double();
  ki_yaw_ = get_node()->get_parameter("ki_yaw").as_double();
  kd_yaw_ = get_node()->get_parameter("kd_yaw").as_double();

  kp_depth_ = get_node()->get_parameter("kp_depth").as_double();
  ki_depth_ = get_node()->get_parameter("ki_depth").as_double();
  kd_depth_ = get_node()->get_parameter("kd_depth").as_double();

  command_threshold_ =
    std::max(0.0, get_node()->get_parameter("command_threshold").as_double());
  depth_command_threshold_ =
    std::max(0.0, get_node()->get_parameter("depth_command_threshold").as_double());

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

  enable_roll_pitch_srv_ = get_node()->create_service<TriggerSrv>(
    enable_roll_pitch_service_name_,
    [this](
      const std::shared_ptr<TriggerSrv::Request>,
      std::shared_ptr<TriggerSrv::Response> response)
    {
      setRollPitchEnabled(true, true);
      response->success = true;
      response->message = "Roll and pitch setpoints will be zeroed and unlocked in update.";
    });

  disable_roll_pitch_srv_ = get_node()->create_service<TriggerSrv>(
    disable_roll_pitch_service_name_,
    [this](
      const std::shared_ptr<TriggerSrv::Request>,
      std::shared_ptr<TriggerSrv::Response> response)
    {
      setRollPitchEnabled(false, true);
      response->success = true;
      response->message = "Roll and pitch setpoints will be zeroed and locked in update.";
    });

  setpoint_pub_ = get_node()->create_publisher<SetPointMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS());
  setpoint_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<SetPointMsg>>(setpoint_pub_);
  allow_roll_pitch_buffer_.writeFromNonRT(allow_roll_pitch_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Depth hold feedforward gains loaded: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
    feedforward_gain_x_,
    feedforward_gain_y_,
    feedforward_gain_z_,
    feedforward_gain_roll_,
    feedforward_gain_pitch_,
    feedforward_gain_yaw_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Depth hold PID gains loaded: roll(kp=%.3f ki=%.3f kd=%.3f) pitch(kp=%.3f ki=%.3f kd=%.3f) yaw(kp=%.3f ki=%.3f kd=%.3f) depth(kp=%.3f ki=%.3f kd=%.3f)",
    kp_roll_,
    ki_roll_,
    kd_roll_,
    kp_pitch_,
    ki_pitch_,
    kd_pitch_,
    kp_yaw_,
    ki_yaw_,
    kd_yaw_,
    kp_depth_,
    ki_depth_,
    kd_depth_);

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&DepthHoldController::parametersCallback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DepthHoldController::on_activate(
  const rclcpp_lifecycle::State &)
{
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};
  depth_pid_ = AxisPidState{};
  roll_setpoint_ = 0.0;
  pitch_setpoint_ = 0.0;
  yaw_setpoint_ = 0.0;
  depth_setpoint_ = 0.0;
  roll_setpoint_initialized_ = false;
  pitch_setpoint_initialized_ = false;
  yaw_setpoint_initialized_ = false;
  depth_setpoint_initialized_ = false;
  roll_feedforward_active_ = false;
  pitch_feedforward_active_ = false;
  yaw_feedforward_active_ = false;
  depth_feedforward_active_ = false;
  zero_roll_pitch_requested_.store(false);
  allow_roll_pitch_buffer_.writeFromNonRT(allow_roll_pitch_);
  first_update_ = true;

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DepthHoldController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};
  depth_pid_ = AxisPidState{};
  roll_setpoint_ = 0.0;
  pitch_setpoint_ = 0.0;
  yaw_setpoint_ = 0.0;
  depth_setpoint_ = 0.0;
  roll_setpoint_initialized_ = false;
  pitch_setpoint_initialized_ = false;
  yaw_setpoint_initialized_ = false;
  depth_setpoint_initialized_ = false;
  roll_feedforward_active_ = false;
  pitch_feedforward_active_ = false;
  yaw_feedforward_active_ = false;
  depth_feedforward_active_ = false;
  zero_roll_pitch_requested_.store(false);
  allow_roll_pitch_buffer_.writeFromNonRT(allow_roll_pitch_);
  first_update_ = true;

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult DepthHoldController::parametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  bool feedforward_gains_updated = false;

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (name == "kp_roll") {
      kp_roll_ = param.as_double();
    } else if (name == "allow_roll_pitch") {
      setRollPitchEnabled(param.as_bool(), !param.as_bool());
    } else if (name == "feedforward_gain_x") {
      feedforward_gain_x_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_y") {
      feedforward_gain_y_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_z") {
      feedforward_gain_z_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_roll") {
      feedforward_gain_roll_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_pitch") {
      feedforward_gain_pitch_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_yaw") {
      feedforward_gain_yaw_ = param.as_double();
      feedforward_gains_updated = true;
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
    } else if (name == "kp_depth") {
      kp_depth_ = param.as_double();
    } else if (name == "ki_depth") {
      ki_depth_ = param.as_double();
      depth_pid_.integral = 0.0;
    } else if (name == "kd_depth") {
      kd_depth_ = param.as_double();
    } else if (name == "command_threshold") {
      command_threshold_ = std::max(0.0, param.as_double());
    } else if (name == "depth_command_threshold") {
      depth_command_threshold_ = std::max(0.0, param.as_double());
    }
  }

  if (feedforward_gains_updated) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Depth hold feedforward gains updated: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
      feedforward_gain_x_,
      feedforward_gain_y_,
      feedforward_gain_z_,
      feedforward_gain_roll_,
      feedforward_gain_pitch_,
      feedforward_gain_yaw_);
  }

  return result;
}

double DepthHoldController::computePid(
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

double DepthHoldController::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void DepthHoldController::setRollPitchEnabled(bool enabled, bool request_zero_setpoint)
{
  allow_roll_pitch_ = enabled;
  allow_roll_pitch_buffer_.writeFromNonRT(enabled);
  if (request_zero_setpoint) {
    zero_roll_pitch_requested_.store(true);
  }
}

void DepthHoldController::publishSetpoint()
{
  if (setpoint_rt_pub_ && setpoint_rt_pub_->trylock()) {
    setpoint_rt_pub_->msg_.position.x = std::numeric_limits<double>::quiet_NaN();
    setpoint_rt_pub_->msg_.position.y = std::numeric_limits<double>::quiet_NaN();
    setpoint_rt_pub_->msg_.position.z = depth_setpoint_;
    setpoint_rt_pub_->msg_.rpy.x = roll_setpoint_;
    setpoint_rt_pub_->msg_.rpy.y = pitch_setpoint_;
    setpoint_rt_pub_->msg_.rpy.z = yaw_setpoint_;
    setpoint_rt_pub_->unlockAndPublish();
  }
}

controller_interface::return_type DepthHoldController::update(
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

  const double depth = (*navigator_msg)->position.position.z;

  if (
    !roll_setpoint_initialized_ ||
    !pitch_setpoint_initialized_ ||
    !yaw_setpoint_initialized_ ||
    !depth_setpoint_initialized_)
  {
    roll_setpoint_ = roll;
    pitch_setpoint_ = pitch;
    yaw_setpoint_ = yaw;
    depth_setpoint_ = depth;
    roll_setpoint_initialized_ = true;
    pitch_setpoint_initialized_ = true;
    yaw_setpoint_initialized_ = true;
    depth_setpoint_initialized_ = true;
    publishSetpoint();
  }

  const double force_x =
    (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->linear.x * feedforward_gain_x_ : 0.0;
  const double force_y =
    (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->linear.y * feedforward_gain_y_ : 0.0;
  const double force_ff_z =
    (feedforward_msg && *feedforward_msg) ? (*feedforward_msg)->linear.z * feedforward_gain_z_ : 0.0;
  const double torque_ff_x =
    (feedforward_msg && *feedforward_msg) ?
    (*feedforward_msg)->angular.x * feedforward_gain_roll_ : 0.0;
  const double torque_ff_y =
    (feedforward_msg && *feedforward_msg) ?
    (*feedforward_msg)->angular.y * feedforward_gain_pitch_ : 0.0;
  const double yaw_feedforward =
    (feedforward_msg && *feedforward_msg) ?
    (*feedforward_msg)->angular.z * feedforward_gain_yaw_ : 0.0;
  const bool * allow_roll_pitch_ptr = allow_roll_pitch_buffer_.readFromRT();
  const bool allow_roll_pitch = allow_roll_pitch_ptr ? *allow_roll_pitch_ptr : false;
  const bool zero_roll_pitch_requested =
    zero_roll_pitch_requested_.exchange(false);
  const bool has_roll_feedforward = std::abs(torque_ff_x) > command_threshold_;
  const bool has_pitch_feedforward = std::abs(torque_ff_y) > command_threshold_;
  const bool has_yaw_feedforward = std::abs(yaw_feedforward) > command_threshold_;
  const bool has_depth_feedforward = std::abs(force_ff_z) > depth_command_threshold_;
  bool publish_setpoint =
    zero_roll_pitch_requested ||
    (!has_roll_feedforward && roll_feedforward_active_) ||
    (!has_pitch_feedforward && pitch_feedforward_active_) ||
    (!has_yaw_feedforward && yaw_feedforward_active_) ||
    (!has_depth_feedforward && depth_feedforward_active_);

  if (zero_roll_pitch_requested || !allow_roll_pitch) {
    roll_setpoint_ = 0.0;
    pitch_setpoint_ = 0.0;
  }

  if (allow_roll_pitch && !zero_roll_pitch_requested && has_roll_feedforward) {
    roll_setpoint_ = roll;
    roll_pid_.integral = 0.0;
  } else if (allow_roll_pitch && !zero_roll_pitch_requested && roll_feedforward_active_) {
    roll_setpoint_ = roll;
    roll_pid_.integral = 0.0;
  }

  if (allow_roll_pitch && !zero_roll_pitch_requested && has_pitch_feedforward) {
    pitch_setpoint_ = pitch;
    pitch_pid_.integral = 0.0;
  } else if (allow_roll_pitch && !zero_roll_pitch_requested && pitch_feedforward_active_) {
    pitch_setpoint_ = pitch;
    pitch_pid_.integral = 0.0;
  }

  if (has_yaw_feedforward) {
    yaw_setpoint_ = yaw;
    yaw_pid_.integral = 0.0;
  } else if (yaw_feedforward_active_) {
    yaw_setpoint_ = yaw;
    yaw_pid_.integral = 0.0;
  }

  if (has_depth_feedforward) {
    depth_setpoint_ = depth;
    depth_pid_.integral = 0.0;
  } else if (depth_feedforward_active_) {
    depth_setpoint_ = depth;
    depth_pid_.integral = 0.0;
  }

  if (publish_setpoint) {
    publishSetpoint();
  }

  const double dt = period.seconds();

  const double roll_error = wrapAngle(roll_setpoint_ - roll);
  const double pitch_error = wrapAngle(pitch_setpoint_ - pitch);
  const double yaw_error = wrapAngle(yaw_setpoint_ - yaw);
  const double depth_error = depth_setpoint_ - depth;

  const double force_z =
    force_ff_z +
    computePid(depth_error, dt, kp_depth_, ki_depth_, kd_depth_, depth_pid_);

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

  roll_feedforward_active_ = allow_roll_pitch && has_roll_feedforward;
  pitch_feedforward_active_ = allow_roll_pitch && has_pitch_feedforward;
  yaw_feedforward_active_ = has_yaw_feedforward;
  depth_feedforward_active_ = has_depth_feedforward;
  first_update_ = false;

  return controller_interface::return_type::OK;
}

}  // namespace cirtesub_controllers

PLUGINLIB_EXPORT_CLASS(
  cirtesub_controllers::DepthHoldController,
  controller_interface::ControllerInterface)
