#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace cirtesub_controllers
{

class PositionHoldController : public controller_interface::ChainableControllerInterface
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
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using TwistMsg = geometry_msgs::msg::Twist;
  using NavigatorMsg = sura_msgs::msg::Navigator;

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
    double antiwindup,
    AxisPidState & state);

  static double wrapAngle(double angle);
  void resetPidStates();
  void setSetpointFromNavigator(const NavigatorMsg & navigator_msg);
  void applyExternalSetpoint(const PoseStampedMsg & setpoint_msg);
  void publishCurrentSetpoint(const rclcpp::Time & stamp);
  void updateReferenceInterfacesFromSetpoint();

  rclcpp::Subscription<PoseStampedMsg>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<TwistMsg>::SharedPtr feedforward_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr setpoint_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<PoseStampedMsg>> setpoint_rt_pub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseStampedMsg>> setpoint_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> feedforward_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string setpoint_topic_;
  std::string feedforward_topic_;
  std::string navigator_topic_;
  std::string body_velocity_controller_name_;
  std::string setpoint_frame_id_;
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

  double linear_feedforward_threshold_{1e-3};
  double angular_feedforward_threshold_{1e-3};
  double feedforward_timeout_{0.25};

  AxisPidState x_pid_;
  AxisPidState y_pid_;
  AxisPidState z_pid_;
  AxisPidState roll_pid_;
  AxisPidState pitch_pid_;
  AxisPidState yaw_pid_;

  PoseStampedMsg current_setpoint_;
  TwistMsg current_feedforward_;
  bool setpoint_initialized_{false};
  bool feedforward_active_{false};
  std::atomic<bool> new_setpoint_requested_{false};
  std::atomic<int64_t> last_feedforward_time_ns_{0};
};

}  // namespace cirtesub_controllers
