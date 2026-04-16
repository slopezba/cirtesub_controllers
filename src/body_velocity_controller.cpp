#include "cirtesub_controllers/body_velocity_controller.hpp"

#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace cirtesub_controllers
{

controller_interface::CallbackReturn BodyVelocityController::on_init()
{
  try {
    auto_declare<std::string>(
      "setpoint_topic", "/body_velocity_controller/setpoint");
    auto_declare<std::string>(
      "navigator_topic", "/cirtesub/navigator/navigation");
    auto_declare<std::string>(
      "body_force_controller_name", "body_force_controller");

    auto_declare<double>("kp_x", 0.0);
    auto_declare<double>("ki_x", 0.0);
    auto_declare<double>("kd_x", 0.0);

    auto_declare<double>("kp_y", 0.0);
    auto_declare<double>("ki_y", 0.0);
    auto_declare<double>("kd_y", 0.0);

    auto_declare<double>("kp_z", 0.0);
    auto_declare<double>("ki_z", 0.0);
    auto_declare<double>("kd_z", 0.0);

    auto_declare<double>("kp_roll", 0.0);
    auto_declare<double>("ki_roll", 0.0);
    auto_declare<double>("kd_roll", 0.0);

    auto_declare<double>("kp_pitch", 0.0);
    auto_declare<double>("ki_pitch", 0.0);
    auto_declare<double>("kd_pitch", 0.0);

    auto_declare<double>("kp_yaw", 0.0);
    auto_declare<double>("ki_yaw", 0.0);
    auto_declare<double>("kd_yaw", 0.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BodyVelocityController::command_interface_configuration() const
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
BodyVelocityController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn BodyVelocityController::on_configure(
  const rclcpp_lifecycle::State &)
{
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  body_force_controller_name_ =
    get_node()->get_parameter("body_force_controller_name").as_string();

  kp_x_ = get_node()->get_parameter("kp_x").as_double();
  ki_x_ = get_node()->get_parameter("ki_x").as_double();
  kd_x_ = get_node()->get_parameter("kd_x").as_double();

  kp_y_ = get_node()->get_parameter("kp_y").as_double();
  ki_y_ = get_node()->get_parameter("ki_y").as_double();
  kd_y_ = get_node()->get_parameter("kd_y").as_double();

  kp_z_ = get_node()->get_parameter("kp_z").as_double();
  ki_z_ = get_node()->get_parameter("ki_z").as_double();
  kd_z_ = get_node()->get_parameter("kd_z").as_double();

  kp_roll_ = get_node()->get_parameter("kp_roll").as_double();
  ki_roll_ = get_node()->get_parameter("ki_roll").as_double();
  kd_roll_ = get_node()->get_parameter("kd_roll").as_double();

  kp_pitch_ = get_node()->get_parameter("kp_pitch").as_double();
  ki_pitch_ = get_node()->get_parameter("ki_pitch").as_double();
  kd_pitch_ = get_node()->get_parameter("kd_pitch").as_double();

  kp_yaw_ = get_node()->get_parameter("kp_yaw").as_double();
  ki_yaw_ = get_node()->get_parameter("ki_yaw").as_double();
  kd_yaw_ = get_node()->get_parameter("kd_yaw").as_double();

  setpoint_sub_ = get_node()->create_subscription<TwistMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const TwistMsg::SharedPtr msg)
    {
      setpoint_buffer_.writeFromNonRT(msg);
    });

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&BodyVelocityController::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Configured BodyVelocityController");
  RCLCPP_INFO(get_node()->get_logger(), "setpoint topic: %s", setpoint_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "navigator topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "body_force_controller_name: %s",
    body_force_controller_name_.c_str());
  logGains("Body velocity gains loaded");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyVelocityController::on_activate(
  const rclcpp_lifecycle::State &)
{
  x_pid_ = AxisPidState{};
  y_pid_ = AxisPidState{};
  z_pid_ = AxisPidState{};
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  logGains("Body velocity controller activated with gains");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyVelocityController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  x_pid_ = AxisPidState{};
  y_pid_ = AxisPidState{};
  z_pid_ = AxisPidState{};
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult BodyVelocityController::parametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (name == "kp_x") {
      kp_x_ = param.as_double();
    } else if (name == "ki_x") {
      ki_x_ = param.as_double();
      x_pid_.integral = 0.0;
    } else if (name == "kd_x") {
      kd_x_ = param.as_double();
    } else if (name == "kp_y") {
      kp_y_ = param.as_double();
    } else if (name == "ki_y") {
      ki_y_ = param.as_double();
      y_pid_.integral = 0.0;
    } else if (name == "kd_y") {
      kd_y_ = param.as_double();
    } else if (name == "kp_z") {
      kp_z_ = param.as_double();
    } else if (name == "ki_z") {
      ki_z_ = param.as_double();
      z_pid_.integral = 0.0;
    } else if (name == "kd_z") {
      kd_z_ = param.as_double();
    } else if (name == "kp_roll") {
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
    }
  }

  logGains("Body velocity gains updated");

  return result;
}

double BodyVelocityController::computePid(
  double error,
  double measured_acceleration,
  double dt,
  double kp,
  double ki,
  double kd,
  AxisPidState & state)
{
  state.integral += error * dt;
  const double derivative = -measured_acceleration;
  return kp * error + ki * state.integral + kd * derivative;
}

void BodyVelocityController::logGains(const std::string & context) const
{
  RCLCPP_INFO(
    get_node()->get_logger(),
    "%s: x(kp=%.3f ki=%.3f kd=%.3f) y(kp=%.3f ki=%.3f kd=%.3f) z(kp=%.3f ki=%.3f kd=%.3f) roll(kp=%.3f ki=%.3f kd=%.3f) pitch(kp=%.3f ki=%.3f kd=%.3f) yaw(kp=%.3f ki=%.3f kd=%.3f)",
    context.c_str(),
    kp_x_,
    ki_x_,
    kd_x_,
    kp_y_,
    ki_y_,
    kd_y_,
    kp_z_,
    ki_z_,
    kd_z_,
    kp_roll_,
    ki_roll_,
    kd_roll_,
    kp_pitch_,
    ki_pitch_,
    kd_pitch_,
    kp_yaw_,
    ki_yaw_,
    kd_yaw_);
}

controller_interface::return_type BodyVelocityController::update(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  auto setpoint_msg = setpoint_buffer_.readFromRT();
  auto navigator_msg = navigator_buffer_.readFromRT();

  if (!navigator_msg || !(*navigator_msg)) {
    return controller_interface::return_type::OK;
  }

  if (command_interfaces_.size() != 6) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Expected 6 command interfaces, got %zu",
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  TwistMsg desired_velocity;
  if (setpoint_msg && *setpoint_msg) {
    desired_velocity = *(*setpoint_msg);
  }

  const double dt = period.seconds();

  const double force_x = computePid(
    desired_velocity.linear.x - (*navigator_msg)->body_velocity.linear.x,
    (*navigator_msg)->body_acceleration.linear.x,
    dt,
    kp_x_,
    ki_x_,
    kd_x_,
    x_pid_);

  const double force_y = computePid(
    desired_velocity.linear.y - (*navigator_msg)->body_velocity.linear.y,
    (*navigator_msg)->body_acceleration.linear.y,
    dt,
    kp_y_,
    ki_y_,
    kd_y_,
    y_pid_);

  const double force_z = computePid(
    desired_velocity.linear.z - (*navigator_msg)->body_velocity.linear.z,
    (*navigator_msg)->body_acceleration.linear.z,
    dt,
    kp_z_,
    ki_z_,
    kd_z_,
    z_pid_);

  const double torque_x = computePid(
    desired_velocity.angular.x - (*navigator_msg)->body_velocity.angular.x,
    (*navigator_msg)->body_acceleration.angular.x,
    dt,
    kp_roll_,
    ki_roll_,
    kd_roll_,
    roll_pid_);

  const double torque_y = computePid(
    desired_velocity.angular.y - (*navigator_msg)->body_velocity.angular.y,
    (*navigator_msg)->body_acceleration.angular.y,
    dt,
    kp_pitch_,
    ki_pitch_,
    kd_pitch_,
    pitch_pid_);

  const double torque_z = computePid(
    desired_velocity.angular.z - (*navigator_msg)->body_velocity.angular.z,
    (*navigator_msg)->body_acceleration.angular.z,
    dt,
    kp_yaw_,
    ki_yaw_,
    kd_yaw_,
    yaw_pid_);

  command_interfaces_[0].set_value(force_x);
  command_interfaces_[1].set_value(force_y);
  command_interfaces_[2].set_value(force_z);
  command_interfaces_[3].set_value(torque_x);
  command_interfaces_[4].set_value(torque_y);
  command_interfaces_[5].set_value(torque_z);

  return controller_interface::return_type::OK;
}

}  // namespace cirtesub_controllers

PLUGINLIB_EXPORT_CLASS(
  cirtesub_controllers::BodyVelocityController,
  controller_interface::ControllerInterface)
