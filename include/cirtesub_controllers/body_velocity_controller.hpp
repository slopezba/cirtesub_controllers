#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace cirtesub_controllers
{

class BodyVelocityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using NavigatorMsg = sura_msgs::msg::Navigator;

  struct AxisPidState
  {
    double integral{0.0};
  };

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & params);

  double computePid(
    double error,
    double measured_acceleration,
    double dt,
    double kp,
    double ki,
    double kd,
    AxisPidState & state);

  void logGains(const std::string & context) const;

  rclcpp::Subscription<TwistMsg>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> setpoint_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string setpoint_topic_;
  std::string navigator_topic_;
  std::string body_force_controller_name_;

  double kp_x_{0.0};
  double ki_x_{0.0};
  double kd_x_{0.0};

  double kp_y_{0.0};
  double ki_y_{0.0};
  double kd_y_{0.0};

  double kp_z_{0.0};
  double ki_z_{0.0};
  double kd_z_{0.0};

  double kp_roll_{0.0};
  double ki_roll_{0.0};
  double kd_roll_{0.0};

  double kp_pitch_{0.0};
  double ki_pitch_{0.0};
  double kd_pitch_{0.0};

  double kp_yaw_{0.0};
  double ki_yaw_{0.0};
  double kd_yaw_{0.0};

  AxisPidState x_pid_;
  AxisPidState y_pid_;
  AxisPidState z_pid_;
  AxisPidState roll_pid_;
  AxisPidState pitch_pid_;
  AxisPidState yaw_pid_;
};

}  // namespace cirtesub_controllers
