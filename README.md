# cirtesub_controllers

Custom ROS 2 chainable controllers for the CIRTESUB vehicle.

This package currently provides five controllers:

- `body_force_controller`
- `body_velocity_controller`
- `position_hold_controller`
- `stabilize_controller`
- `depth_hold_controller`

All five plugins inherit from `controller_interface::ChainableControllerInterface`, so each one can run in:

- `topic mode`: it reads commands from ROS topics
- `chained mode`: it receives commands through exported reference interfaces from another controller

## Overview

| Controller | Purpose | Input | Output | Chained |
| --- | --- | --- | --- | --- |
| `body_force_controller` | Convert a body wrench into individual thruster efforts | `geometry_msgs/msg/Wrench` or chained wrench references | Thruster `effort` commands | Yes |
| `body_velocity_controller` | Track body-frame linear/angular velocity setpoints | `geometry_msgs/msg/Twist` + `/cirtesub/navigator/navigation` | `geometry_msgs/msg/Wrench` feedforward for the next stage | Yes |
| `position_hold_controller` | Hold a 6-DoF pose and generate body velocity commands | `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/Twist` + `/cirtesub/navigator/navigation` | Chained body velocity commands | Yes |
| `stabilize_controller` | Hold roll, pitch, yaw while passing translational feedforward | `geometry_msgs/msg/Wrench` + `/cirtesub/navigator/navigation` | Chained body wrench commands | Yes |
| `depth_hold_controller` | Hold depth, yaw, and optionally roll/pitch while passing XY feedforward | `geometry_msgs/msg/Wrench` + `/cirtesub/navigator/navigation` | Chained body wrench commands | Yes |

## Common Concepts

### Topic mode vs chained mode

Every controller exports reference interfaces. In `topic mode`, the controller fills those references from a ROS topic. In `chained mode`, another controller writes directly into the exported references.

### Navigator dependency

All controllers except `body_force_controller` depend on:

- `/cirtesub/navigator/navigation`

They use it as the measured state for their control loops.

### Default controller names

The default chain names configured in [`cirtesub_bringup/config/ros2_control_params.yaml`](/home/cirtesu/cirtesub_ws/src/cirtesub_bringup/config/ros2_control_params.yaml) are:

- `body_force_controller`
- `body_velocity_controller`
- `position_hold_controller`
- `stabilize_controller`
- `depth_hold_controller`

Those names matter because chained interfaces are exported as `<controller_name>/<interface_name>`.

## Controller Details

### `body_force_controller`

Purpose:
Convert a desired body wrench into individual thruster efforts using a thruster allocation matrix built from `robot_description`.

How it works:

- Parses `<ros2_control>` joints from `robot_description`
- Selects joints with an `effort` command interface
- Builds the thruster allocation matrix from each thruster pose relative to `base_link`
- Computes thruster commands with the pseudoinverse of the allocation matrix

Default parameters:

- `input_topic`: `/body_force/command`
- `base_link`: `base_link`

Topic input in topic mode:

- `/body_force/command`
- Type: `geometry_msgs/msg/Wrench`

Consumed command interfaces:

- Thruster joint efforts discovered from `robot_description`
- Format: `<thruster_joint>/effort`

Exported chained reference interfaces:

- `body_force_controller/force.x`
- `body_force_controller/force.y`
- `body_force_controller/force.z`
- `body_force_controller/torque.x`
- `body_force_controller/torque.y`
- `body_force_controller/torque.z`

Publishes:

- None

Services:

- None

What it receives and what it outputs:

- Receives a body wrench in the vehicle frame
- Outputs one `effort` command per thruster

Typical use:

- Final allocation stage in every control chain
- Can also be used standalone for direct wrench control

### `body_velocity_controller`

Purpose:
Track body-frame linear and angular velocity setpoints and generate a body wrench request for `body_force_controller`.

How it works:

- Reads desired body velocities
- Reads measured body velocities and accelerations from the navigator
- Runs six PID loops:
  - `x`, `y`, `z`
  - `roll`, `pitch`, `yaw`
- Publishes the resulting wrench as feedforward for the next controller

Default parameters:

- `setpoint_topic`: `/body_velocity_controller/setpoint`
- `navigator_topic`: `/cirtesub/navigator/navigation`
- `feedforward_topic`: `/depth_hold_controller/feedforward`
- `body_force_controller_name`: `body_force_controller`

Topic input in topic mode:

- `/body_velocity_controller/setpoint`
- Type: `geometry_msgs/msg/Twist`

State input:

- `/cirtesub/navigator/navigation`
- Type: navigator message used by the project
- Consumes:
  - `body_velocity.linear`
  - `body_velocity.angular`
  - `body_acceleration.linear`
  - `body_acceleration.angular`

Exported chained reference interfaces:

- `body_velocity_controller/linear.x`
- `body_velocity_controller/linear.y`
- `body_velocity_controller/linear.z`
- `body_velocity_controller/angular.x`
- `body_velocity_controller/angular.y`
- `body_velocity_controller/angular.z`

Publishes:

- `/depth_hold_controller/feedforward` by default
- Type: `geometry_msgs/msg/Wrench`

Services:

- None

What it receives and what it outputs:

- Receives desired body-frame linear/angular velocities
- Outputs a body wrench feedforward for a wrench-level controller

Typical use:

- Standalone velocity loop feeding `body_force_controller`
- Middle stage of a pose-hold chain: `position_hold_controller -> body_velocity_controller -> body_force_controller`
- In the current bringup config, its feedforward topic points to `/depth_hold_controller/feedforward`, so it is intended to feed `depth_hold_controller` when both are used together

### `position_hold_controller`

Purpose:
Hold a full 6-DoF pose and generate body-frame velocity commands for `body_velocity_controller`.

How it works:

- Stores a pose setpoint
- If no setpoint has been received yet, it captures the current navigator pose as the hold setpoint
- Converts world-frame position error into the body frame
- Computes orientation error from quaternion difference
- Runs six PID loops to generate body-frame linear/angular velocity commands
- Accepts external feedforward velocities that temporarily move the hold setpoint with the vehicle

Default parameters:

- `setpoint_topic`: `/position_hold_controller/setpoint`
- `feedforward_topic`: `/position_hold_controller/feedforward`
- `navigator_topic`: `/cirtesub/navigator/navigation`
- `body_velocity_controller_name`: `body_velocity_controller`
- `setpoint_frame_id`: `world_ned`

Topic inputs in topic mode:

- `/position_hold_controller/setpoint`
- Type: `geometry_msgs/msg/PoseStamped`
- `/position_hold_controller/feedforward`
- Type: `geometry_msgs/msg/Twist`

State input:

- `/cirtesub/navigator/navigation`

Exported chained reference interfaces:

- `position_hold_controller/position.x`
- `position_hold_controller/position.y`
- `position_hold_controller/position.z`
- `position_hold_controller/orientation.roll`
- `position_hold_controller/orientation.pitch`
- `position_hold_controller/orientation.yaw`

Consumed command interfaces from the next stage:

- `body_velocity_controller/linear.x`
- `body_velocity_controller/linear.y`
- `body_velocity_controller/linear.z`
- `body_velocity_controller/angular.x`
- `body_velocity_controller/angular.y`
- `body_velocity_controller/angular.z`

Publishes:

- `/position_hold_controller/setpoint`
- Type: `geometry_msgs/msg/PoseStamped`
- This is also republished internally so the current held pose can be monitored

Services:

- None

What it receives and what it outputs:

- Receives a desired pose and optional feedforward body velocities
- Outputs body-frame linear/angular velocity commands

Typical use:

- Outer loop for station keeping or waypoint hold
- Designed to chain into `body_velocity_controller`

Important behavior:

- Feedforward is considered valid only while it is fresh (`feedforward_timeout`)
- When feedforward is active, the controller updates the hold setpoint to the current pose, effectively following operator motion without accumulating position error

### `stabilize_controller`

Purpose:
Hold attitude while allowing translational wrench feedforward to pass through.

In practice it is an attitude stabilizer with yaw hold and optional roll/pitch hold release.

How it works:

- Reads a wrench feedforward
- Passes `force.x`, `force.y`, `force.z` directly after applying feedforward gains
- Uses navigator attitude as feedback
- Holds `roll`, `pitch`, and `yaw` setpoints with PID control
- When commanded feedforward appears on roll, pitch, or yaw, it snaps the corresponding setpoint to the current measured attitude to avoid fighting the operator
- Can lock roll/pitch to zero or allow them to float

Default parameters:

- `feedforward_topic`: `/stabilize_controller/feedforward`
- `navigator_topic`: `/cirtesub/navigator/navigation`
- `setpoint_topic`: `/stabilize_controller/set_point`
- `body_force_controller_name`: `body_force_controller`
- `enable_roll_pitch_service_name`: `/stabilize_controller/enable_roll_pitch`
- `disable_roll_pitch_service_name`: `/stabilize_controller/disable_roll_pitch`
- `allow_roll_pitch`: `false`

Topic input in topic mode:

- `/stabilize_controller/feedforward`
- Type: `geometry_msgs/msg/Wrench`

State input:

- `/cirtesub/navigator/navigation`

Exported chained reference interfaces:

- `stabilize_controller/force.x`
- `stabilize_controller/force.y`
- `stabilize_controller/force.z`
- `stabilize_controller/torque.x`
- `stabilize_controller/torque.y`
- `stabilize_controller/torque.z`

Consumed command interfaces from the next stage:

- `body_force_controller/force.x`
- `body_force_controller/force.y`
- `body_force_controller/force.z`
- `body_force_controller/torque.x`
- `body_force_controller/torque.y`
- `body_force_controller/torque.z`

Publishes:

- `/stabilize_controller/set_point`
- Type: `geometry_msgs/msg/Vector3`
- Fields:
  - `x = roll_setpoint`
  - `y = pitch_setpoint`
  - `z = yaw_setpoint`

Services:

- `/stabilize_controller/enable_roll_pitch`
- `/stabilize_controller/disable_roll_pitch`
- Type: `std_srvs/srv/Trigger`

Service behavior:

- `enable_roll_pitch`: unlocks roll and pitch hold on the next update
- `disable_roll_pitch`: zeros roll and pitch setpoints and locks them on the next update

What it receives and what it outputs:

- Receives a feedforward wrench
- Outputs a stabilized wrench with PID-generated torques added

Typical use:

- Teleoperation with yaw hold
- Intermediate wrench-level stabilizer before `body_force_controller`

### `depth_hold_controller`

Purpose:
Hold depth and yaw, and optionally roll/pitch, while allowing XY wrench feedforward to pass through.

How it works:

- Reads a wrench feedforward
- Passes `force.x` and `force.y` through after feedforward gains
- Uses `force.z` as depth feedforward
- Uses navigator pose to estimate roll, pitch, yaw, and depth
- Holds `depth`, `yaw`, and optionally `roll`/`pitch` with PID control
- Just like `stabilize_controller`, it resets setpoints to the current measured state when feedforward becomes active

Default parameters:

- `feedforward_topic`: `/depth_hold_controller/feedforward`
- `navigator_topic`: `/cirtesub/navigator/navigation`
- `setpoint_topic`: `/depth_hold_controller/set_point`
- `body_force_controller_name`: `body_force_controller`
- `enable_roll_pitch_service_name`: `/depth_hold_controller/enable_roll_pitch`
- `disable_roll_pitch_service_name`: `/depth_hold_controller/disable_roll_pitch`
- `allow_roll_pitch`: `false`

Topic input in topic mode:

- `/depth_hold_controller/feedforward`
- Type: `geometry_msgs/msg/Wrench`

State input:

- `/cirtesub/navigator/navigation`

Exported chained reference interfaces:

- `depth_hold_controller/force.x`
- `depth_hold_controller/force.y`
- `depth_hold_controller/force.z`
- `depth_hold_controller/torque.x`
- `depth_hold_controller/torque.y`
- `depth_hold_controller/torque.z`

Consumed command interfaces from the next stage:

- `body_force_controller/force.x`
- `body_force_controller/force.y`
- `body_force_controller/force.z`
- `body_force_controller/torque.x`
- `body_force_controller/torque.y`
- `body_force_controller/torque.z`

Publishes:

- `/depth_hold_controller/set_point`
- Custom setpoint message used by the project
- Content:
  - `position.z = depth_setpoint`
  - `rpy.x = roll_setpoint`
  - `rpy.y = pitch_setpoint`
  - `rpy.z = yaw_setpoint`
  - `position.x` and `position.y` are published as `NaN`

Services:

- `/depth_hold_controller/enable_roll_pitch`
- `/depth_hold_controller/disable_roll_pitch`
- Type: `std_srvs/srv/Trigger`

Service behavior:

- `enable_roll_pitch`: unlocks roll and pitch hold on the next update
- `disable_roll_pitch`: zeros roll and pitch setpoints and locks them on the next update

What it receives and what it outputs:

- Receives a feedforward wrench
- Outputs a stabilized wrench with depth and attitude corrections

Typical use:

- Depth hold during teleoperation
- Wrench-level stabilization stage before `body_force_controller`

## Typical Chains

### 1. Direct wrench control

```text
ROS topic (Wrench)
  -> body_force_controller
  -> thruster effort commands
```

Topic used:

- `/body_force/command`

### 2. Velocity control

```text
ROS topic (Twist)
  -> body_velocity_controller
  -> Wrench feedforward
  -> body_force_controller
  -> thruster effort commands
```

This is the logical chain implemented by the controllers, although the default bringup config currently points `body_velocity_controller.feedforward_topic` to `/depth_hold_controller/feedforward`.

It can also be extended like this:

```text
ROS topic (Twist)
  -> body_velocity_controller
  -> stabilize_controller or depth_hold_controller
  -> body_force_controller
  -> thruster effort commands
```

### 3. Position hold

```text
Pose hold
  -> position_hold_controller
  -> body velocity command interfaces
  -> body_velocity_controller
  -> Wrench feedforward
  -> body_force_controller
  -> thruster effort commands
```

This chain can also insert a wrench-level stabilizer before allocation:

```text
Pose hold
  -> position_hold_controller
  -> body_velocity_controller
  -> stabilize_controller or depth_hold_controller
  -> body_force_controller
  -> thruster effort commands
```

### 4. Attitude stabilization

```text
Wrench feedforward
  -> stabilize_controller
  -> body wrench command interfaces
  -> body_force_controller
  -> thruster effort commands
```

### 5. Depth hold

```text
Wrench feedforward
  -> depth_hold_controller
  -> body wrench command interfaces
  -> body_force_controller
  -> thruster effort commands
```

## Notes

- `thruster_test_controller` is configured in bringup but does not belong to this package.
- `body_force_controller` is chainable even though it can also be used directly from a topic.
- `position_hold_controller` exports pose references, but its downstream consumed interfaces are velocity references on `body_velocity_controller`.
- `stabilize_controller` and `depth_hold_controller` both operate as wrench-level controllers that add closed-loop corrections on top of feedforward commands.
