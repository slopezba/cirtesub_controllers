# Controllers Explained

This document is a guided explanation of how the controllers in `cirtesub_controllers` work internally.

It is written for someone who is starting with ROS 2 Control and wants to understand:

- how a ROS 2 controller plugin is structured
- why there is a `.hpp`, a `.cpp`, and a plugin XML file
- how `controller_manager` discovers the controllers
- what each function in these controllers is doing
- what parts are friendly to realtime execution and what parts are risky

The goal is not only to say what the code does, but why it was written that way and what practical effect it has on the robot.

## File Map

These are the key files in this package:

- [`package.xml`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/package.xml)
- [`CMakeLists.txt`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/CMakeLists.txt)
- [`cirtesub_controllers_plugins.xml`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/cirtesub_controllers_plugins.xml)
- [`include/cirtesub_controllers/body_force_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/body_force_controller.hpp)
- [`include/cirtesub_controllers/body_velocity_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/body_velocity_controller.hpp)
- [`include/cirtesub_controllers/stabilize_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/stabilize_controller.hpp)
- [`include/cirtesub_controllers/depth_hold_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/depth_hold_controller.hpp)
- [`include/cirtesub_controllers/position_hold_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/position_hold_controller.hpp)
- [`src/body_force_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/body_force_controller.cpp)
- [`src/body_velocity_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/body_velocity_controller.cpp)
- [`src/stabilize_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/stabilize_controller.cpp)
- [`src/depth_hold_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/depth_hold_controller.cpp)
- [`src/position_hold_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/position_hold_controller.cpp)

The general rule is:

- `.hpp`: declares the class, members, and function signatures
- `.cpp`: implements the behavior
- `plugins.xml`: tells `pluginlib` which classes are available as controller plugins
- `CMakeLists.txt`: builds the shared library and exports the plugin description
- `package.xml`: declares ROS dependencies

## ROS 2 Control: The Big Picture

Before we go controller by controller, it helps to understand the architecture around them.

### What is `ros2_control`?

`ros2_control` is the framework that sits between:

- the robot hardware or simulator
- the control algorithms
- the rest of the ROS graph

In simple terms:

- hardware exposes interfaces such as position, velocity, or effort
- controllers read some state and produce commands
- `controller_manager` calls the controllers periodically

### What does `controller_manager` do?

`controller_manager` is the runtime that:

- loads controller plugins
- configures them
- activates and deactivates them
- calls the update loop at a fixed rate

When a controller is active, `controller_manager` repeatedly calls its update functions.

### Why these controllers inherit from `ChainableControllerInterface`

All controllers in this package inherit from:

```cpp
controller_interface::ChainableControllerInterface
```

This is more powerful than a basic controller because it allows two modes:

- topic mode
- chained mode

In topic mode:

- the controller receives commands from ROS topics

In chained mode:

- another controller writes directly into exported reference interfaces

That makes it possible to create control chains such as:

```text
position_hold -> body_velocity -> depth_hold -> body_force
```

instead of having every controller talk only through topics.

## Common Structure Shared by All Controllers

Even though the controllers do different jobs, they follow the same overall skeleton.

### 1. Class declaration in the header

Every controller declares a class like:

```cpp
class BodyVelocityController : public controller_interface::ChainableControllerInterface
```

What this means in practice:

- the class is a ROS 2 Control controller plugin
- `controller_manager` can load it dynamically
- the class must implement the lifecycle and update methods expected by ROS 2 Control

### 2. Lifecycle methods

All controllers implement these functions:

- `on_init()`
- `on_configure(...)`
- `on_activate(...)`
- `on_deactivate(...)`

These are lifecycle hooks.

Think of them like this:

- `on_init()`: declare parameters and set up static defaults
- `on_configure()`: read parameters, create subscriptions, publishers, or services
- `on_activate()`: reset runtime state before control starts
- `on_deactivate()`: stop commanding and reset internal state

### 3. Interface description methods

All controllers implement:

- `command_interface_configuration() const`
- `state_interface_configuration() const`

These tell ROS 2 Control what hardware or controller interfaces the controller wants to use.

For example:

- `body_force_controller` requests real thruster `effort` command interfaces
- `position_hold_controller` requests command interfaces exported by `body_velocity_controller`

### 4. Chainable-specific methods

All controllers also implement:

- `on_export_reference_interfaces()`
- `on_set_chained_mode(bool chained_mode)`
- `update_reference_from_subscribers()`
- `update_and_write_commands(...)`

These are the heart of the chainable design.

The mental model is:

- `on_export_reference_interfaces()`
  - creates software command interfaces other controllers can write into
- `on_set_chained_mode(...)`
  - tells the controller whether it should accept direct chained input
- `update_reference_from_subscribers()`
  - copies topic data into the reference interfaces
- `update_and_write_commands(...)`
  - performs the actual control computation and writes the output commands

### 5. Private helpers

Each controller then adds helper functions such as:

- PID functions
- angle wrapping
- setpoint publishing
- URDF parsing
- matrix building

Those helpers are not part of the ROS 2 Control API. They are only internal implementation details.

## The Control Loop Pattern

A useful way to understand these controllers is this repeated pattern:

1. Receive input from a ROS topic or another controller
2. Copy that input into internal reference values
3. Read current state from navigator or hardware
4. Compute an output command
5. Write the command to the next stage

Depending on the controller, the output may be:

- a wrench
- a velocity command
- direct thruster effort

### Who Actually Calls the Update Methods

One of the most important details to understand is that your controller class does not call
`update_and_write_commands(...)` by itself.

The real call chain is:

```text
ros2_control_node
  -> controller_manager
    -> controller->update(time, period)
```

For a chainable controller, the important detail is that `controller_manager` does not call
`update_and_write_commands(...)` directly.
It calls:

```cpp
update(time, period)
```

through the generic controller interface.

In a normal controller, that `update(...)` method is usually implemented directly by the controller.
But in a chainable controller, the situation is different.

`ChainableControllerInterface` already implements:

```cpp
return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) final;
```

The `final` keyword is very important here because it means:

- the chainable base class owns the common update flow
- child controllers do not override `update(...)`
- child controllers instead implement the specific pieces used by that flow

Those pieces are:

- `update_reference_from_subscribers()`
- `update_and_write_commands(...)`

So the real flow for a chainable controller is:

```text
controller_manager
  -> controller->update(time, period)
      -> ChainableControllerInterface::update(time, period)
```

And from there the base class decides what to do next.

### Topic Mode Flow

If the controller is **not** in chained mode, the chainable base class first updates the internal
reference values from ROS subscribers and then runs the control law:

```text
controller_manager.update(...)
  -> controller->update(...)
      -> ChainableControllerInterface::update(...)
          -> update_reference_from_subscribers()
          -> update_and_write_commands(...)
```

This means:

- ROS topic callbacks store incoming data in a realtime buffer
- `update_reference_from_subscribers()` copies that data into `reference_interfaces_`
- `update_and_write_commands(...)` uses those references to compute the output

Example with `BodyVelocityController`:

```text
/body_velocity_controller/setpoint (Twist)
  -> subscriber callback
  -> setpoint_buffer_
  -> update_reference_from_subscribers()
  -> reference_interfaces_
  -> update_and_write_commands()
  -> publish Wrench
```

### Chained Mode Flow

If the controller **is** in chained mode, another controller is already writing directly into the
exported reference interfaces.

So in that case, the base class does not need to update references from ROS subscribers first:

```text
controller_manager.update(...)
  -> controller->update(...)
      -> ChainableControllerInterface::update(...)
          -> update_and_write_commands(...)
```

In other words:

- topic mode: references come from subscribers
- chained mode: references come from another controller

But in both cases:

- `update_and_write_commands(...)` is the place where the real control computation happens

Example:

```text
PositionHoldController
  -> writes into body_velocity_controller/linear.x ...
  -> BodyVelocityController reference_interfaces_
  -> BodyVelocityController::update_and_write_commands(...)
```

### Where `on_set_chained_mode(...)` Fits

The `controller_manager` is also the component that manages whether a controller is currently being
used in a chain.

When the chain topology changes, it switches the controller between:

- topic mode
- chained mode

That is what eventually triggers:

```cpp
on_set_chained_mode(bool chained_mode)
```

So this method is not part of the periodic update loop itself.
Instead, it updates the controller's operating mode so that the next calls to
`ChainableControllerInterface::update(...)` follow the correct path.

### Summary of the Real Control Loop

For a normal controller:

```text
controller_manager -> controller.update(...)
```

For a chainable controller:

```text
controller_manager
  -> controller.update(...)
      -> ChainableControllerInterface::update(...)
          -> if not in chained mode:
               update_reference_from_subscribers()
          -> update_and_write_commands(...)
```

That is why your controller classes do not implement `update(...)` directly.
They implement the pieces that the chainable base class plugs into the shared update flow.

## How ROS 2 Control Discovers These Controllers

This part is important because many beginners see the controller class and think it is enough. It is not.

Three things are needed for plugin discovery:

1. the class must be compiled into a shared library
2. the class must be exported with `PLUGINLIB_EXPORT_CLASS`
3. the package must provide a plugin description XML

### `package.xml`

[`package.xml`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/package.xml) declares dependencies such as:

- `controller_interface`
- `hardware_interface`
- `pluginlib`
- `realtime_tools`
- `rclcpp`
- `rclcpp_lifecycle`
- `geometry_msgs`
- `std_srvs`
- `urdf`
- `tf2`
- `sura_msgs`

Why this matters:

- without these dependencies, the code may compile incorrectly or not compile at all
- more importantly, plugin consumers expect the package metadata to be correct

### `CMakeLists.txt`

[`CMakeLists.txt`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/CMakeLists.txt) does several important things.

First, it builds the shared library:

```cpp
add_library(${PROJECT_NAME} SHARED
  src/body_force_controller.cpp
  src/body_velocity_controller.cpp
  src/depth_hold_controller.cpp
  src/position_hold_controller.cpp
  src/stabilize_controller.cpp
)
```

Why this matters:

- controllers are loaded as plugins at runtime
- a shared library is required for that

Then it exports the plugin description:

```cpp
pluginlib_export_plugin_description_file(
  controller_interface
  cirtesub_controllers_plugins.xml
)
```

This tells pluginlib that this package provides controller plugins described in that XML file.

### `cirtesub_controllers_plugins.xml`

[`cirtesub_controllers_plugins.xml`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/cirtesub_controllers_plugins.xml) maps a plugin name to a concrete C++ class.

Example:

```xml
<class
  name="cirtesub_controllers/BodyForceController"
  type="cirtesub_controllers::BodyForceController"
  base_class_type="controller_interface::ChainableControllerInterface">
```

This means:

- the controller type string used in YAML is `cirtesub_controllers/BodyForceController`
- the real C++ class is `cirtesub_controllers::BodyForceController`
- it derives from `ChainableControllerInterface`

### `PLUGINLIB_EXPORT_CLASS(...)`

At the bottom of every `.cpp` file there is a line like:

```cpp
PLUGINLIB_EXPORT_CLASS(
  cirtesub_controllers::BodyForceController,
  controller_interface::ChainableControllerInterface)
```

This is what finally registers the compiled class with pluginlib.

Without this line:

- the class would compile
- the library would exist
- but pluginlib would not be able to instantiate it

## Realtime Basics for These Controllers

Before discussing each controller, we need one practical idea.

### What does realtime mean here?

In ROS 2 Control, the update loop often runs in a timing-sensitive thread.

That means we try to avoid inside `update_and_write_commands(...)`:

- dynamic memory allocation
- locking mutexes
- blocking on ROS communication
- creating publishers or subscriptions
- heavy logging
- expensive computations with variable runtime

In short:

- setup code is fine in `on_configure()`
- timing-critical code should stay lean in `update_and_write_commands()`

### Patterns in this package that are good for realtime

These controllers already use several good patterns:

- `realtime_tools::RealtimeBuffer`
  - topic callbacks run in normal ROS threads
  - the update loop reads the latest data without blocking
- `realtime_tools::RealtimePublisher`
  - publishing from the update loop is done in a safer way than with a normal publisher
- `std::atomic`
  - small flags such as `new_setpoint_requested_` or `zero_roll_pitch_requested_` are passed safely between threads without a heavy mutex
- reset and setup work is mostly kept in lifecycle callbacks

### Patterns in this package that are risky for realtime

There are also some important caveats in the current code.

1. `body_force_controller` computes the pseudoinverse inside the update loop.

Why this is risky:

- it uses dynamic-size Eigen matrices
- it runs SVD every cycle
- it can allocate memory and take variable time

In practice:

- this is one of the biggest realtime concerns in the package
- ideally the pseudoinverse should be computed once during configuration if the thruster matrix does not change

2. Parameter callbacks write directly into variables used by the update loop.

Why this is risky:

- callbacks happen in non-realtime ROS threads
- update runs in the control loop
- values like `kp_x_` or `feedforward_gain_x_` are plain `double`
- there is no `RealtimeBuffer` or explicit synchronization for most of them

What can happen:

- data races are possible in principle
- on many platforms a `double` write may appear to work most of the time, but it is still not a strict realtime-safe design

3. Logging in update paths should be treated carefully.

Example:

- `BodyForceController::update_and_write_commands()` may emit `RCLCPP_ERROR_THROTTLE(...)`

Why this is risky:

- logging can involve formatting and internal locking
- it is usually avoided in strict realtime loops

4. Dynamic-size Eigen containers are used in control paths.

This is especially visible in:

- `BodyForceController`

Dynamic containers are convenient, but they are less predictable for realtime than fixed-size stack objects.

5. Normal ROS subscriptions and services are created in `on_configure()`, which is correct, but their callbacks still modify state that update consumes.

This is acceptable when coupled with `RealtimeBuffer` or atomics.
It is less safe when directly mutating shared controller state.

## Common Function-by-Function Template

To make the per-controller sections easier to follow, here is what the common functions mean in plain language.

### `on_init()`

Typical job:

- declare parameters and defaults

Why it exists:

- ROS 2 parameters should be declared before being used
- defaults make the controller usable even if YAML is incomplete

Realtime note:

- not realtime critical
- safe place for declarations

### `command_interface_configuration() const`

Typical job:

- request the interfaces this controller wants to command

Examples:

- real hardware efforts
- chained reference interfaces from another controller
- no interfaces if the controller only publishes to a topic

Realtime note:

- not in the fast loop

### `state_interface_configuration() const`

Typical job:

- request hardware state interfaces if needed

In this package:

- most controllers return `NONE`
- they do not read hardware state interfaces directly
- instead they consume navigator messages

Design implication:

- the control state comes from a ROS topic, not from `state_interfaces_`

### `on_configure(...)`

Typical job:

- read parameters
- create publishers
- create subscriptions
- create services
- initialize exported reference interface names
- install dynamic parameter callbacks

A simple beginner way to think about it is:

- "register topics and services, read the configuration, and prepare the controller"

Realtime note:

- this is not realtime critical
- creating subscriptions, publishers, and services here is normal and correct

### `on_activate(...)`

Typical job:

- reset integrators
- clear stale setpoints
- zero output commands
- initialize runtime flags

Why it matters:

- activation should start from a known clean state

Realtime note:

- not in the normal loop

### `on_deactivate(...)`

Typical job:

- stop publishing commands
- clear runtime state
- zero command interfaces if appropriate

Why it matters:

- prevents stale commands from remaining active after deactivation

Realtime note:

- not in the normal loop

### `on_export_reference_interfaces()`

Typical job:

- create software interfaces that another controller can write to

Why it matters:

- this is the key mechanism behind chaining

Example:

If a controller exports:

```text
body_force_controller/force.x
```

then another controller can target that interface directly instead of publishing to a topic.

Realtime note:

- setup-time operation, not loop-critical

### `on_set_chained_mode(bool chained_mode)`

Typical job:

- react when the controller switches between topic mode and chained mode

In this package it usually:

- logs the mode change
- clears reference interfaces when returning to topic mode

Why clearing matters:

- old chained commands should not survive when the controller stops being chained

### `update_reference_from_subscribers()`

Typical job:

- read the latest topic message from a `RealtimeBuffer`
- copy its values into the internal `reference_interfaces_`

Why it exists:

- it bridges ROS topic inputs into the same reference mechanism used by chained mode

Realtime note:

- usually fine
- reading from `RealtimeBuffer` is one of the standard safe patterns

### `update_and_write_commands(...)`

Typical job:

- compute the controller output
- write to command interfaces or realtime publishers

This is the most important function in every controller.

Realtime note:

- this is the main timing-sensitive path
- if a controller has a realtime problem, it is usually here

## Controller 1: `BodyForceController`

Relevant files:

- [`include/cirtesub_controllers/body_force_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/body_force_controller.hpp)
- [`src/body_force_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/body_force_controller.cpp)

### What this controller is really for

This is the allocation layer.

Its job is:

- receive a desired body wrench
- translate that wrench into one effort command per thruster

It is the closest controller to the hardware.

If the upstream controller says:

- "apply this force and this torque to the robot body"

then `BodyForceController` answers:

- "these are the individual thruster efforts needed to approximate that wrench"

### Header structure

The header shows the main design:

- one ROS topic input: `body_force_sub_`
- one realtime buffer: `rt_buffer_ptr_`
- URDF parsing helpers
- thruster model data:
  - `base_link_`
  - `thruster_joints_`
  - `thruster_allocation_matrix_`

This tells us immediately that the controller is split into two halves:

- configuration-time geometry extraction
- runtime force allocation

### `urdfPoseToEigen(...)`

Purpose:

- convert a URDF pose into an Eigen transform

Why it exists:

- URDF gives joint transforms in URDF types
- the controller uses Eigen for matrix math

What it does:

- creates a quaternion from the URDF rotation
- normalizes it
- creates an `Eigen::Isometry3d`
- copies rotation and translation

Why it matters physically:

- every thruster has a position and direction
- to compute torque contribution you need both

Realtime note:

- not part of the fast loop
- safe

### `extractThrusterJointsFromRos2Control(...)`

Purpose:

- parse `robot_description`
- find all joints under `<ros2_control>` that expose an `effort` command interface

Why this is clever:

- the controller does not hardcode thruster names
- it discovers them from the robot description

What it does:

- parses the XML with TinyXML2
- iterates over `<ros2_control>` blocks
- iterates over `<joint>` elements
- checks whether the joint has a `command_interface` named `effort`
- stores the joint names

Why it matters:

- the allocation controller becomes more generic
- if the URDF changes, the controller can adapt without recompiling

Realtime note:

- safe because it runs during setup
- would be bad if it ran during update, but it does not

### `resolveBaseLink(...)`

Purpose:

- determine which link should be used as the reference frame for the allocation matrix

Behavior:

- if the configured `base_link` exists, use it
- otherwise fall back to the URDF root link

Why it matters:

- all wrench computations are frame-dependent
- if the base frame is wrong, force and torque mapping will be wrong too

### `jointPoseInBase(...)`

Purpose:

- compute a joint pose relative to the chosen base link

How it works:

- starts from the joint
- walks up the URDF tree through parent links
- accumulates transforms until it reaches `base_link`

Why it matters:

- each thruster contributes:
  - a force direction
  - a moment arm

Without the joint pose in the base frame, you cannot build the allocation matrix.

### `buildThrusterAllocationMatrix(...)`

Purpose:

- build the matrix that maps thruster forces into a body wrench

This is one of the most important functions in the whole package.

Physical idea:

Each thruster contributes:

- a force direction `d`
- a torque contribution `m = r x d`

where:

- `r` is the thruster position relative to the base
- `d` is the thrust direction

The controller stores these contributions in a matrix `B`.

Interpretation:

- top 3 rows: force contribution
- bottom 3 rows: torque contribution

Why it matters:

- if `u` is the vector of thruster forces, then the body wrench is approximately:

```text
tau = B * u
```

To go the other way, the controller later uses a pseudoinverse.

Realtime note:

- safe because it runs in configuration
- this is exactly the kind of expensive math you want to do once, not every cycle

### `pseudoInverse(...)`

Purpose:

- compute the Moore-Penrose pseudoinverse with SVD

Why it is needed:

- the allocation matrix is usually not square
- even if it were square, it may not be invertible
- the pseudoinverse gives a least-squares solution

Realtime note:

- the function itself is mathematically fine
- but calling it inside the update loop is expensive and not ideal for realtime

### `on_init()`

Purpose:

- declare parameters:
  - `input_topic`
  - `base_link`

Why it matters:

- this controller can work in topic mode
- it also needs to know which base link to use for allocation

### `command_interface_configuration() const`

Purpose:

- request the real thruster effort interfaces

What is special here:

- it reads `robot_description`
- extracts thruster joint names
- returns interface names like:

```text
<joint_name>/effort
```

Why this is important:

- this is the point where the controller tells ROS 2 Control:
  - "I want to command the thrusters directly"

Realtime note:

- not a loop concern

### `state_interface_configuration() const`

Purpose:

- this controller returns `NONE`

Meaning:

- it does not read hardware state interfaces
- it only writes commands

### `on_configure(...)`

Purpose:

- read parameters
- read and parse `robot_description`
- discover thrusters
- resolve base link
- build the allocation matrix
- create the subscription to the wrench command topic
- define exported reference interface names

The subscription creation looks like this conceptually:

- subscribe to `/body_force/command`
- when a message arrives, store it into a realtime buffer

Why this pattern is good:

- the ROS callback is non-realtime
- the update loop later reads from `RealtimeBuffer`

This is a classic and good separation between ROS callbacks and the control loop.

### `on_activate(...)`

Purpose:

- clear the realtime buffer
- reset exported references to `NaN`

Why `NaN` is used:

- it means "no command currently set"
- later the controller can treat `NaN` as zero or ignore it

### `on_deactivate(...)`

Purpose:

- clear the realtime buffer
- set every thruster command interface to zero

Why it matters:

- deactivation should leave the hardware in a safe state

### `on_export_reference_interfaces()`

Purpose:

- export these software references:
  - `force.x`
  - `force.y`
  - `force.z`
  - `torque.x`
  - `torque.y`
  - `torque.z`

Practical meaning:

- another controller can command this controller by writing a wrench directly into these reference slots

### `on_set_chained_mode(bool chained_mode)`

Purpose:

- just logs whether the controller is in chained or topic mode

Design note:

- unlike other controllers, this one does not clear references when switching modes
- it only logs the mode

### `update_reference_from_subscribers()`

Purpose:

- read the last `Wrench` received on the ROS topic
- copy it into the six reference interfaces

This is the bridge from ROS topic commands into the chainable reference storage.

### `update_and_write_commands(...)`

This is the true allocation step.

What it does:

1. Builds a desired 6x1 wrench vector from `reference_interfaces_`
2. Uses the pseudoinverse of `B`
3. Computes:

```text
thruster_forces = B_pinv * desired_wrench
```

4. Writes each resulting value to the corresponding thruster command interface

Why this is the core of the controller:

- it converts body-level intention into actuator-level commands

Realtime notes:

- reading six doubles and writing command interfaces is fine
- recomputing `pseudoInverse(thruster_allocation_matrix_)` every update is not ideal
- using dynamic `Eigen::MatrixXd` and SVD in the loop is one of the main realtime weaknesses of this package

Practical risk:

- jitter in control timing
- unnecessary CPU usage

If you wanted to improve this first:

- compute and store `B_pinv` during `on_configure()`
- only multiply during update

## Controller 2: `BodyVelocityController`

Relevant files:

- [`include/cirtesub_controllers/body_velocity_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/body_velocity_controller.hpp)
- [`src/body_velocity_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/body_velocity_controller.cpp)

### What this controller is really for

This controller closes the loop on body velocities.

It receives desired:

- surge, sway, heave velocities
- roll, pitch, yaw rates

and turns them into a wrench request.

In practical terms:

- if you want the robot to move at a target body velocity
- this controller decides what force and torque should be applied

### Header structure

Main components:

- `setpoint_sub_`
- `navigator_sub_`
- `feedforward_pub_`
- realtime buffers for setpoint and navigator
- a realtime publisher for the output wrench
- PID gains and PID states for six axes

This tells us:

- input is velocity command plus state estimate
- output is not hardware effort yet, but a wrench

### `AxisPidState`

Contains:

- `integral`

Interesting detail:

- there is no stored previous error here
- derivative is based on measured acceleration instead

That is a common control idea:

- if you want a derivative term on velocity error, and the setpoint derivative is ignored, then:
  - derivative of error is approximately negative measured acceleration

### `on_init()`

Purpose:

- declare all topics
- declare downstream controller name
- declare six sets of PID gains and antiwindup limits

Why it matters:

- this controller is highly parameterized
- almost all its behavior comes from the configured gains

### `command_interface_configuration() const`

Returns:

- `NONE`

Why?

- this controller does not write directly into hardware or another controller's command interfaces
- its output is published on a topic as a `Wrench`

This is an important design point:

- even though the class is chainable and exports references, its output is topic-based

### `state_interface_configuration() const`

Also returns:

- `NONE`

Because:

- it reads state from `/cirtesub/navigator/navigation`
- not from ROS 2 hardware state interfaces

### `on_configure(...)`

This is where the controller becomes operational.

It does several jobs.

First:

- reads topic names
- reads the downstream controller name
- reads all PID gains

Second:

- creates the setpoint subscription
- creates the navigator subscription
- creates the wrench publisher and wraps it with a `RealtimePublisher`

Third:

- defines the exported chained references:
  - `linear.x`
  - `linear.y`
  - `linear.z`
  - `angular.x`
  - `angular.y`
  - `angular.z`

Fourth:

- installs a parameter callback so gains can be updated at runtime

Simple interpretation:

- "register topics, register output, create the internal chainable references, and load the controller tuning"

Realtime notes:

- good to create all ROS entities here
- the parameter callback itself is not realtime safe with respect to shared doubles, but configuration code here is fine

### `on_activate(...)`

Purpose:

- reset all PID integrators
- clear reference interfaces to `NaN`

Why it matters:

- activation should not inherit stale integrator values from the last run

### `on_deactivate(...)`

Purpose:

- reset PIDs
- clear references
- publish a zero wrench if possible

Why publishing zero helps:

- downstream consumers will not keep seeing the last nonzero wrench forever

Realtime note:

- this is not the main loop, so the zero publish here is fine

### `parametersCallback(...)`

Purpose:

- allow changing gains while the controller is running

What it does:

- checks each parameter name
- updates the corresponding gain
- resets integrators when `ki` changes

Why resetting the integrator is sensible:

- changing integral gain with old stored integral can create a jump in control output

Realtime caution:

- this callback writes plain doubles and PID state directly
- update reads those same values in the control loop
- this is convenient but not a strict realtime-safe synchronization strategy

### `computePid(...)`

Purpose:

- compute a PID control action using:
  - current error
  - measured acceleration
  - gains
  - antiwindup limit

What it does:

1. integrates error
2. clamps integral if antiwindup is enabled
3. sets derivative term to `-measured_acceleration`
4. returns:

```text
kp * error + ki * integral + kd * derivative
```

Why this makes sense here:

- the controlled variable is velocity
- measured acceleration is a practical estimate of the derivative of velocity

### `logGains(...)`

Purpose:

- print the current gains

This is not control logic, just observability.

### `on_export_reference_interfaces()`

Exports:

- `linear.x`
- `linear.y`
- `linear.z`
- `angular.x`
- `angular.y`
- `angular.z`

Meaning:

- another controller can inject a body velocity setpoint directly

This is how `position_hold_controller` can chain into it.

### `on_set_chained_mode(bool chained_mode)`

Behavior:

- logs the mode
- if going back to topic mode, clears references to `NaN`

Why clearing is useful:

- prevents stale chained commands from remaining active

### `update_reference_from_subscribers()`

Purpose:

- read the last `Twist` setpoint from the realtime buffer
- copy it into the internal references

This is a very standard bridge function.

### `update_and_write_commands(...)`

This is the actual control law.

What it does:

1. Reads the latest navigator state
2. Uses `period.seconds()` as `dt`
3. Converts `NaN` references into zero setpoints
4. For each axis computes a force or torque using `computePid(...)`
5. Publishes the resulting wrench through the realtime publisher

Axis mapping:

- linear velocity errors produce:
  - `force.x`
  - `force.y`
  - `force.z`
- angular velocity errors produce:
  - `torque.x`
  - `torque.y`
  - `torque.z`

Important design point:

- this controller does not write hardware commands
- it produces an intermediate wrench command for the next layer

Realtime notes:

- good use of realtime buffers
- good use of realtime publisher
- math is lightweight and mostly stack-based
- the main realtime concern is unsynchronized runtime gain updates from the parameter callback

## Controller 3: `PositionHoldController`

Relevant files:

- [`include/cirtesub_controllers/position_hold_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/position_hold_controller.hpp)
- [`src/position_hold_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/position_hold_controller.cpp)

### What this controller is really for

This is the outer-loop pose controller.

Its job is:

- hold a target pose
- compute the body-frame velocity commands needed to reduce pose error

This means:

- it does not command thrusters directly
- it does not command wrench directly
- it commands desired body velocities for `body_velocity_controller`

### Header structure

Main elements:

- subscriptions for:
  - pose setpoint
  - feedforward twist
  - navigator state
- publisher for current setpoint
- realtime buffers
- PID gains and states
- persistent state:
  - `current_setpoint_`
  - `current_feedforward_`
  - flags for initialization and freshness

This header already reveals a very practical behavior:

- the controller can both hold a pose and temporarily follow operator feedforward

### `AxisPidState`

Contains:

- `integral`
- `previous_error`

Here derivative is computed from error difference, unlike `BodyVelocityController`.

Why:

- this controller works on pose error, not velocity error
- so estimating derivative from the error history is more natural

### `on_init()`

Declares:

- topics
- downstream controller name
- default frame id
- PID gains for all six pose axes
- feedforward thresholds
- feedforward timeout

Why timeout matters:

- feedforward messages should not remain valid forever
- old commands should expire

### `command_interface_configuration() const`

This controller requests command interfaces from:

- `body_velocity_controller`

Specifically:

- `linear.x`
- `linear.y`
- `linear.z`
- `angular.x`
- `angular.y`
- `angular.z`

This is an excellent example of chaining.

Meaning:

- the output of `PositionHoldController` is written directly into the input references of `BodyVelocityController`

### `state_interface_configuration() const`

Returns:

- `NONE`

because state comes from the navigator topic.

### `on_configure(...)`

This function does a lot of setup.

1. Read parameters

- topic names
- downstream controller name
- setpoint frame id
- gains
- thresholds
- timeout

2. Create the setpoint subscription

Interesting detail:

- `ignore_local_publications = true`

Why this matters:

- this controller republishes its own setpoint for monitoring
- without ignoring local publications, it could read back its own published setpoint and treat it like an external command

That line is a small but very important practical detail.

3. Create feedforward and navigator subscriptions

4. Create the realtime setpoint publisher

5. Define exported references:

- `position.x`
- `position.y`
- `position.z`
- `orientation.roll`
- `orientation.pitch`
- `orientation.yaw`

6. Register the dynamic parameter callback

### `on_activate(...)`

Purpose:

- reset PIDs
- clear current setpoint and feedforward
- mark setpoint as uninitialized
- clear flags
- zero command interfaces

Why this is good:

- when activated, the controller starts from a known clean condition

### `on_deactivate(...)`

Similar purpose:

- reset internal state
- zero commands

### `parametersCallback(...)`

Same general role as in `BodyVelocityController`:

- update gains and thresholds at runtime
- reset integrators when needed

Realtime caution:

- same concern about direct shared variable mutation

### `computePid(...)`

Purpose:

- compute a PID output from pose error

Behavior:

- uses a safe `dt`
- integrates error
- clamps integral with antiwindup if configured
- computes derivative from:

```text
(error - previous_error) / dt
```

This is a conventional discrete PID implementation.

### `wrapAngle(...)`

Purpose:

- normalize angle errors to a principal range

Why it matters:

- orientation errors should take the shortest path
- without wrapping, yaw errors near `pi` and `-pi` can behave badly

### `resetPidStates()`

Purpose:

- clear all PID accumulators and previous errors

This is used when:

- activating
- deactivating
- changing the setpoint logic due to feedforward transitions

### `setSetpointFromNavigator(...)`

Purpose:

- copy the current navigator pose as the hold setpoint

Why this is useful:

- if no external setpoint has been sent yet, the controller can still hold the current pose

Practical effect:

- "freeze here"

### `applyExternalSetpoint(...)`

Purpose:

- accept a new external `PoseStamped`
- store it as the active setpoint
- ensure the frame id is valid
- reset PID states
- update exported references

Why reset PID here:

- a new target pose changes the error suddenly
- carrying the old integrator would be undesirable

### `publishCurrentSetpoint(...)`

Purpose:

- publish the current held setpoint using a realtime publisher

Why it exists:

- debugging
- visualization
- operator awareness

Realtime note:

- good pattern, because it uses `RealtimePublisher`

### `updateReferenceInterfacesFromSetpoint()`

Purpose:

- convert `current_setpoint_` into the six exported references

Important detail:

- orientation is converted from quaternion to roll, pitch, yaw

This means the chained interface is not quaternion-based.
It is a six-value pose command in mixed form:

- position xyz
- orientation rpy

### `on_export_reference_interfaces()`

Exports:

- `position.x`
- `position.y`
- `position.z`
- `orientation.roll`
- `orientation.pitch`
- `orientation.yaw`

These references describe what pose this controller is currently trying to hold.

### `on_set_chained_mode(bool chained_mode)`

Behavior:

- logs mode changes
- clears references on topic mode transition

### `update_reference_from_subscribers()`

Interesting detail:

- this function does not copy pose setpoint topic data into references directly
- instead it reads the latest feedforward twist into `current_feedforward_`

Why?

- the external pose setpoint is handled separately in `update_and_write_commands(...)`
- because the controller needs extra logic around freshness, latching, and setpoint replacement

This is a good example that `update_reference_from_subscribers()` does not always have to mean:

- "copy the main setpoint topic into references"

### `update_and_write_commands(...)`

This function is the heart of the pose controller.

What it does conceptually:

1. Read navigator state
2. If no setpoint exists yet, use current pose as hold setpoint
3. If a new external setpoint was requested, apply it
4. Check whether feedforward is fresh enough
5. If feedforward is active:
   - refresh the hold setpoint from current pose
   - reset PIDs
6. Compute position error in the world frame
7. Rotate position error into the body frame
8. Compute orientation error from quaternion difference
9. Run six PID loops
10. Add feedforward twist
11. Write commands into `body_velocity_controller` interfaces

Why rotating into the body frame matters:

- downstream `BodyVelocityController` expects body-frame velocity targets
- world-frame position error must therefore be transformed

This is a very important conceptual step.

Practical example:

- if the robot is facing east and the target is north
- the error in world coordinates is not directly the same as the desired body surge/sway command

Realtime notes:

- mostly good:
  - stack math
  - realtime buffers
  - atomics
  - realtime publisher
- moderate caution:
  - quaternion and tf2 operations still cost CPU, but are usually acceptable
  - parameter updates are still not synchronized in a strict realtime way

## Controller 4: `StabilizeController`

Relevant files:

- [`include/cirtesub_controllers/stabilize_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/stabilize_controller.hpp)
- [`src/stabilize_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/stabilize_controller.cpp)

### What this controller is really for

This controller is a wrench-level attitude stabilizer.

It takes a feedforward wrench and adds feedback torques so that the robot maintains:

- yaw always
- roll and pitch optionally

while still allowing translational forces to pass through.

You can think of it as:

- "keep me level and pointed correctly, but let translational commands pass"

### Header structure

Main components:

- subscriptions:
  - feedforward wrench
  - navigator
- services:
  - enable roll/pitch
  - disable roll/pitch
- publisher:
  - current setpoint as `Vector3`
- realtime buffers for wrench, navigator, and roll/pitch mode
- atomic flag `zero_roll_pitch_requested_`
- PID states and stored setpoints

This header shows a controller with more operational behavior than the previous ones.

It is not just a mathematical loop.
It also exposes operator-facing services.

### `AxisPidState`

Contains:

- `integral`
- `previous_error`

This is a regular PID state for angle control.

### `on_init()`

Declares:

- feedforward topic
- navigator topic
- setpoint topic
- service names
- downstream controller name
- roll/pitch enable mode
- feedforward gains
- thresholds
- PID gains

The parameters clearly reveal the controller philosophy:

- feedforward is allowed
- feedback is added only as needed
- small commands are ignored below thresholds

### `command_interface_configuration() const`

This controller requests command interfaces exported by:

- `body_force_controller`

Specifically:

- `force.x`
- `force.y`
- `force.z`
- `torque.x`
- `torque.y`
- `torque.z`

Meaning:

- `StabilizeController` sits directly upstream of `BodyForceController`

### `state_interface_configuration() const`

Returns:

- `NONE`

because state comes from the navigator topic.

### `on_configure(...)`

This is where the controller wires all communications.

It:

- reads parameters
- creates the feedforward subscription
- creates the navigator subscription
- creates the two services
- creates the realtime publisher for setpoints
- defines exported reference interface names
- installs the dynamic parameter callback

This is the function where you can summarize part of the code as:

- "register topics and services"

But it is useful to be more precise.

The services specifically let an operator or higher-level node choose whether roll and pitch should be:

- actively allowed to follow current orientation
- forced back toward zero

This is very practical for underwater vehicles:

- sometimes you want the robot level
- sometimes you want to allow roll or pitch while maneuvering

Realtime note:

- creating services and publishers here is correct

### `on_activate(...)`

Purpose:

- clear all PID states
- initialize setpoints and flags
- zero command interfaces

Important detail:

- `first_update_ = true`

This is used so the derivative term does not use bogus history on the first cycle.

### `on_deactivate(...)`

Purpose:

- clear the same runtime state
- zero outputs

### `parametersCallback(...)`

Purpose:

- update gains, thresholds, feedforward gains, and roll/pitch mode at runtime

Interesting detail:

- if `allow_roll_pitch` changes, it calls `setRollPitchEnabled(...)`

That means parameter updates affect behavior, not just numbers.

Realtime caution:

- same shared-variable concern as before

### `computePid(...)`

Purpose:

- compute angle PID using error derivative from error history

### `wrapAngle(...)`

Purpose:

- keep angle errors bounded

This is essential for yaw, and still useful for roll and pitch.

### `setRollPitchEnabled(...)`

Purpose:

- update whether roll/pitch are allowed
- push the mode into a realtime buffer
- optionally request that roll and pitch setpoints be zeroed on the next update

Why the `request_zero_setpoint` flag exists:

- service callbacks are not in the control loop
- the actual state transition should happen safely inside update

This is a nice design pattern:

- callbacks request
- update applies

### `publishSetpoint()`

Purpose:

- publish current roll, pitch, yaw setpoints as `Vector3`

Why it helps:

- operators can see what the controller is trying to hold

### `on_export_reference_interfaces()`

Exports the six wrench references for potential upstream chaining.

### `on_set_chained_mode(bool chained_mode)`

Logs mode change and clears references in topic mode.

### `update_reference_from_subscribers()`

Purpose:

- take the latest feedforward wrench from the realtime buffer
- copy it into reference interfaces

This means:

- topic input and chained input ultimately share the same internal representation

### `update_and_write_commands(...)`

This is the core stabilization logic.

What it does:

1. Read navigator orientation
2. Convert quaternion to roll, pitch, yaw
3. Initialize setpoints on first use
4. Read feedforward wrench from reference interfaces
5. Apply feedforward gains
6. Decide whether roll/pitch/yaw feedforward is active by comparing against thresholds
7. Update setpoints depending on:
   - whether roll/pitch are enabled
   - whether zeroing was requested
   - whether feedforward has become active
8. Compute roll, pitch, yaw errors
9. Run PID on those errors
10. Combine:
   - feedforward forces
   - feedforward torques
   - corrective torques
11. Write the final wrench into `body_force_controller` command interfaces

One subtle but important behavior:

When feedforward on an axis becomes active, the controller often sets the hold setpoint to the current measured angle.

Why this is smart:

- suppose the operator commands a yaw rotation
- if the old yaw hold setpoint stayed fixed, the controller would fight the operator
- instead it re-centers the hold point to the current state

That behavior makes the controller feel much more natural in teleoperation.

Realtime notes:

- good:
  - use of realtime buffers
  - atomic flag for zero-roll-pitch request
  - realtime publisher for setpoint
- caution:
  - dynamic parameter callback still writes shared plain variables
  - no obvious blocking operations in update, which is good

## Controller 5: `DepthHoldController`

Relevant files:

- [`include/cirtesub_controllers/depth_hold_controller.hpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/include/cirtesub_controllers/depth_hold_controller.hpp)
- [`src/depth_hold_controller.cpp`](/home/cirtesu/cirtesub_ws/src/cirtesub_controllers/src/depth_hold_controller.cpp)

### What this controller is really for

This controller is very similar in spirit to `StabilizeController`, but it adds depth control.

It takes a feedforward wrench and adds feedback control for:

- depth
- yaw
- optionally roll and pitch

You can think of it as:

- "let the pilot command planar motion, but automatically hold depth and attitude"

### Header structure

Very similar to `StabilizeController`, with two key additions:

- custom published setpoint type:
  - `sura_msgs::msg::AuvControllerSetPoint`
- extra PID state and gains for depth

### `on_init()`

Declares:

- topics
- service names
- downstream controller name
- roll/pitch mode
- feedforward gains
- PID gains for roll, pitch, yaw, and depth
- thresholds

Important threshold split:

- `command_threshold`
  - for angular-related feedforward activity
- `depth_command_threshold`
  - specifically for vertical force feedforward activity

This is sensible because depth control often needs different sensitivity from attitude control.

### `command_interface_configuration() const`

Requests:

- the six wrench interfaces exported by `body_force_controller`

So this controller is also a wrench-level controller upstream of allocation.

### `state_interface_configuration() const`

Returns:

- `NONE`

### `on_configure(...)`

This function:

- reads all parameters
- registers feedforward and navigator subscriptions
- registers the two roll/pitch services
- creates the realtime setpoint publisher
- initializes reference interface names
- installs the parameter callback

This is again the place where "register topics and services" is an accurate high-level summary.

### `on_activate(...)`

Resets:

- PID states
- roll, pitch, yaw, depth setpoints
- feedforward-active flags
- zero-roll-pitch request flag
- references
- command interfaces

This is the correct kind of housekeeping for a controller with internal state.

### `on_deactivate(...)`

Does the corresponding cleanup.

### `parametersCallback(...)`

Same general idea:

- update gains and thresholds at runtime
- reset integrators as needed
- allow parameter-driven roll/pitch mode changes

### `computePid(...)`

Standard error-history PID for angles and depth.

### `wrapAngle(...)`

Used for roll, pitch, yaw error normalization.

### `setRollPitchEnabled(...)`

Same role as in `StabilizeController`.

### `publishSetpoint()`

Publishes:

- `position.z`
- `rpy.x`
- `rpy.y`
- `rpy.z`

with `position.x` and `position.y` set to `NaN`.

Why this is useful:

- the setpoint message carries only the dimensions this controller actually owns
- `NaN` makes it explicit that x and y are not meaningful here

### `on_export_reference_interfaces()`

Exports six wrench references for chaining.

### `on_set_chained_mode(bool chained_mode)`

Logs mode change and clears references in topic mode.

### `update_reference_from_subscribers()`

Copies the latest feedforward wrench from the realtime buffer into the six reference interfaces.

### `update_and_write_commands(...)`

This is the main depth-hold algorithm.

What it does:

1. Read navigator state
2. Extract roll, pitch, yaw from the vehicle quaternion
3. Read current depth from `position.position.z`
4. Initialize setpoints the first time
5. Read feedforward wrench references
6. Apply feedforward gains
7. Determine whether roll, pitch, yaw, and depth feedforward are active based on thresholds
8. Update setpoints:
   - if feedforward is active on an axis, follow the current state
   - if feedforward just stopped, refresh published setpoint
   - if roll/pitch are disabled, set their targets to zero
9. Compute errors:
   - depth error
   - roll error
   - pitch error
   - yaw error
10. Run PID
11. Combine feedforward and feedback
12. Write the final wrench to `body_force_controller`

Why this controller is useful in practice:

- the operator can command forward and lateral motion
- the controller keeps the robot at depth automatically
- yaw can also stay stabilized

This is often one of the most useful operational modes for an underwater robot.

Realtime notes:

- the overall structure is good:
  - buffers for ROS callbacks
  - realtime publisher
  - atomics for one-shot requests
- the same parameter-sharing caution still applies

## Comparing the Controllers

It helps to classify them by level.

### Hardware-level allocator

- `BodyForceController`

Output:

- thruster efforts

### Wrench-level controllers

- `BodyVelocityController`
- `StabilizeController`
- `DepthHoldController`

Output:

- body wrench

### Velocity-level controller

- `PositionHoldController`

Output:

- body velocity commands

This layered design is good because each controller has a clear job:

- position controller decides how fast to move
- velocity controller decides how much force is needed
- wrench stabilizers add attitude or depth behavior
- allocator maps wrench to thrusters

## Realtime Review by Function Type

This section summarizes the realtime aspect in a more practical way.

### Safe or mostly safe setup code

These are generally fine outside the main loop:

- declaring parameters in `on_init()`
- creating subscriptions in `on_configure()`
- creating publishers in `on_configure()`
- creating services in `on_configure()`
- parsing URDF in configuration
- building matrices in configuration

### Good realtime patterns already used

- `RealtimeBuffer` for topic handoff
- `RealtimePublisher` for publishing from update
- atomics for one-way state flags
- `NaN` or zero reset on activate/deactivate

### Main realtime risks in current code

1. `BodyForceController::update_and_write_commands(...)`
   - computes pseudoinverse every cycle
   - uses dynamic Eigen math in the loop

2. Parameter callbacks in multiple controllers
   - directly modify gains and sometimes PID states
   - update loop reads them concurrently

3. Logging from control paths
   - should be minimized in strict realtime designs

### Practical improvements if you wanted to harden this package

1. Precompute the pseudoinverse in `BodyForceController::on_configure()`

2. Move runtime-tunable gains into a `RealtimeBuffer` or a double-buffered parameter struct

3. Keep update loops free of logging except for very rare emergency cases

4. Prefer fixed-size Eigen types in the fast loop where dimensions are known

5. Review whether all published debug setpoints really need to come from inside the update loop

## How to Read a Controller Like This in the Future

If you open a new ROS 2 Control controller and want to understand it quickly, this order works well:

1. Read the header first
   - identify inputs, outputs, state, and helper functions

2. Find `on_init()` and `on_configure()`
   - this tells you topics, services, parameters, and wiring

3. Find `command_interface_configuration()`
   - this tells you what the controller actually commands

4. Find `update_and_write_commands(...)`
   - this is the real control law

5. Check `on_export_reference_interfaces()`
   - this tells you how it participates in chaining

6. Look for `RealtimeBuffer`, `RealtimePublisher`, atomics, logs, and dynamic allocations
   - this tells you whether the design respects realtime constraints

## Final Takeaway

These controllers are well structured from a ROS 2 Control perspective:

- they follow the expected lifecycle
- they use chainable interfaces consistently
- they separate ROS callbacks from update with `RealtimeBuffer`
- they form a clean layered control stack

The most important conceptual idea in the package is this:

- controllers are stacked by abstraction level
- each controller converts one kind of intent into a lower-level command

In very simple terms:

- `PositionHoldController`
  - "where should I be?"
- `BodyVelocityController`
  - "how fast should I move in body frame?"
- `StabilizeController` or `DepthHoldController`
  - "how do I keep attitude/depth under control while moving?"
- `BodyForceController`
  - "how much should each thruster push?"

The main technical caveat is realtime robustness:

- most controllers use good patterns
- but `BodyForceController` is doing too much expensive math inside the update loop
- and runtime gain updates are not fully protected against concurrent access

If you understand those two points, you already understand a large part of the real engineering tradeoff in this package:

- the architecture is clean
- the control intent is clear
- but there is still room to harden the implementation for stricter realtime behavior
