#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace cirtesub_controllers
{

class StabilizeController : public controller_interface::ControllerInterface
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
  using Vector3Msg = geometry_msgs::msg::Vector3;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using TriggerSrv = std_srvs::srv::Trigger;

  struct AxisPidState
  {
    double integral{0.0};
    double previous_error{0.0};
  };

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & params);

  double computePid(
    double error,
    double dt,
    double kp,
    double ki,
    double kd,
    AxisPidState & state);

  static double wrapAngle(double angle);
  void publishSetpoint();
  void setRollPitchEnabled(bool enabled, bool request_zero_setpoint);

  rclcpp::Subscription<TwistMsg>::SharedPtr feedforward_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Service<TriggerSrv>::SharedPtr enable_roll_pitch_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr disable_roll_pitch_srv_;
  rclcpp::Publisher<Vector3Msg>::SharedPtr setpoint_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Vector3Msg>> setpoint_rt_pub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> feedforward_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<bool> allow_roll_pitch_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string feedforward_topic_;
  std::string navigator_topic_;
  std::string setpoint_topic_;
  std::string enable_roll_pitch_service_name_;
  std::string disable_roll_pitch_service_name_;
  std::string body_force_controller_name_;

  double kp_roll_{0.0};
  double ki_roll_{0.0};
  double kd_roll_{0.0};

  double kp_pitch_{0.0};
  double ki_pitch_{0.0};
  double kd_pitch_{0.0};

  double kp_yaw_{0.0};
  double ki_yaw_{0.0};
  double kd_yaw_{0.0};

  bool allow_roll_pitch_{false};
  double feedforward_gain_x_{1.0};
  double feedforward_gain_y_{1.0};
  double feedforward_gain_z_{1.0};
  double yaw_command_threshold_{1e-3};
  double roll_setpoint_{0.0};
  double pitch_setpoint_{0.0};
  double yaw_setpoint_{0.0};
  bool roll_setpoint_initialized_{false};
  bool pitch_setpoint_initialized_{false};
  bool yaw_setpoint_initialized_{false};
  bool roll_feedforward_active_{false};
  bool pitch_feedforward_active_{false};
  bool yaw_feedforward_active_{false};
  std::atomic<bool> zero_roll_pitch_requested_{false};

  AxisPidState roll_pid_;
  AxisPidState pitch_pid_;
  AxisPidState yaw_pid_;
  bool first_update_{true};
};

}  // namespace cirtesub_controllers
