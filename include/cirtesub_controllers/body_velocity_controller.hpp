#pragma once

#include <memory>
#include <limits>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace cirtesub_controllers
{

class BodyVelocityController : public controller_interface::ChainableControllerInterface
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

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using WrenchMsg = geometry_msgs::msg::Wrench;
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
    double antiwindup,
    AxisPidState & state);

  void logGains(const std::string & context) const;

  rclcpp::Subscription<TwistMsg>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr feedforward_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<WrenchMsg>> feedforward_rt_pub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> setpoint_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string setpoint_topic_;
  std::string navigator_topic_;
  std::string feedforward_topic_;
  std::string body_force_controller_name_;
  std::vector<std::string> reference_interface_names_;

  double kp_x_{0.0};
  double ki_x_{0.0};
  double kd_x_{0.0};
  double antiwindup_x_{0.0};

  double kp_y_{0.0};
  double ki_y_{0.0};
  double kd_y_{0.0};
  double antiwindup_y_{0.0};

  double kp_z_{0.0};
  double ki_z_{0.0};
  double kd_z_{0.0};
  double antiwindup_z_{0.0};

  double kp_roll_{0.0};
  double ki_roll_{0.0};
  double kd_roll_{0.0};
  double antiwindup_roll_{0.0};

  double kp_pitch_{0.0};
  double ki_pitch_{0.0};
  double kd_pitch_{0.0};
  double antiwindup_pitch_{0.0};

  double kp_yaw_{0.0};
  double ki_yaw_{0.0};
  double kd_yaw_{0.0};
  double antiwindup_yaw_{0.0};

  AxisPidState x_pid_;
  AxisPidState y_pid_;
  AxisPidState z_pid_;
  AxisPidState roll_pid_;
  AxisPidState pitch_pid_;
  AxisPidState yaw_pid_;
};

}  // namespace cirtesub_controllers
