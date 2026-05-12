#include "cirtesub_controllers/state_observer_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace cirtesub_controllers
{
namespace
{

using Vector6 = StateObserverModel::Vector6;

TEST(StateObserverModel, PredictAccelerationUsesDiagonalParameters)
{
  StateObserverModel model;
  Vector6 rigid = Vector6::Ones();
  Vector6 inertia = Vector6::Constant(2.0);
  Vector6 linear = Vector6::Constant(1.0);
  Vector6 quadratic = Vector6::Zero();
  Vector6 static_wrench = Vector6::Zero();

  model.setRigidBodyInertia(rigid);
  model.setEffectiveInertia(inertia);
  model.setLinearDamping(linear);
  model.setQuadraticDamping(quadratic);
  model.setStaticWrench(static_wrench);

  Vector6 wrench = Vector6::Zero();
  Vector6 velocity = Vector6::Zero();
  Vector6 restoring = Vector6::Zero();
  wrench(0) = 5.0;
  velocity(0) = 1.0;

  const Vector6 acceleration = model.predictAcceleration(wrench, velocity, restoring);

  EXPECT_NEAR(acceleration(0), 2.0, 1e-9);
}

TEST(StateObserverModel, PredictAccelerationSubtractsLearnedStaticWrench)
{
  StateObserverModel model;
  Vector6 inertia = Vector6::Constant(2.0);
  Vector6 static_wrench = Vector6::Zero();
  static_wrench(3) = -8.0;

  model.setEffectiveInertia(inertia);
  model.setLinearDamping(Vector6::Zero());
  model.setQuadraticDamping(Vector6::Zero());
  model.setStaticWrench(static_wrench);

  Vector6 wrench = Vector6::Zero();
  Vector6 velocity = Vector6::Zero();
  Vector6 restoring = Vector6::Zero();
  wrench(3) = -8.0;

  const Vector6 acceleration = model.predictAcceleration(wrench, velocity, restoring);

  EXPECT_NEAR(acceleration(3), 0.0, 1e-9);
}

TEST(StateObserverModel, RecursiveLeastSquaresConvergesForSyntheticAxis)
{
  StateObserverModel model;
  Vector6 rigid = Vector6::Zero();
  Vector6 inertia = Vector6::Constant(1.0);
  Vector6 linear = Vector6::Zero();
  Vector6 quadratic = Vector6::Zero();

  model.setRigidBodyInertia(rigid);
  model.setEffectiveInertia(inertia);
  model.setLinearDamping(linear);
  model.setQuadraticDamping(quadratic);
  model.resetEstimator(1e4);

  constexpr double true_inertia = 3.0;
  constexpr double true_linear = 2.0;
  constexpr double true_quadratic = 4.0;
  constexpr double true_static_wrench = -0.75;

  for (int sample = 0; sample < 1000; ++sample) {
    const double velocity = -1.0 + 2.0 * static_cast<double>(sample % 41) / 40.0;
    const double acceleration = -0.5 + static_cast<double>(sample % 29) / 29.0;
    const double wrench =
      true_inertia * acceleration +
      true_linear * velocity +
      true_quadratic * velocity * std::abs(velocity) +
      true_static_wrench;
    model.updateAxis(0, wrench, velocity, acceleration, 0.0, 0.995, 1e-6);
  }

  const Vector6 learned_inertia = model.effectiveInertia();
  const Vector6 learned_linear = model.linearDamping();
  const Vector6 learned_quadratic = model.quadraticDamping();
  const Vector6 learned_static_wrench = model.staticWrench();

  EXPECT_NEAR(learned_inertia(0), true_inertia, 5e-2);
  EXPECT_NEAR(learned_linear(0), true_linear, 5e-2);
  EXPECT_NEAR(learned_quadratic(0), true_quadratic, 5e-2);
  EXPECT_NEAR(learned_static_wrench(0), true_static_wrench, 5e-2);
}

TEST(StateObserverModel, RecursiveLeastSquaresLearnsStaticWrenchAtRest)
{
  StateObserverModel model;
  model.setEffectiveInertia(Vector6::Constant(1.0));
  model.setLinearDamping(Vector6::Zero());
  model.setQuadraticDamping(Vector6::Zero());
  model.setStaticWrench(Vector6::Zero());
  model.resetEstimator(1e4);

  constexpr double true_static_wrench = -8.0;

  for (int sample = 0; sample < 1000; ++sample) {
    model.updateAxis(3, true_static_wrench, 0.0, 0.0, 0.0, 0.995, 1e-6);
  }

  EXPECT_NEAR(model.staticWrench()(3), true_static_wrench, 5e-2);
  EXPECT_NEAR(model.effectiveInertia()(3), 1.0, 1e-9);
  EXPECT_NEAR(model.linearDamping()(3), 0.0, 1e-9);
  EXPECT_NEAR(model.quadraticDamping()(3), 0.0, 1e-9);
}

TEST(StateObserverModel, LimitsKeepModelPhysicallyUsable)
{
  StateObserverModel model;
  StateObserverModel::Limits limits;
  limits.min_effective_inertia = 0.5;
  limits.max_effective_inertia = 10.0;
  limits.min_linear_damping = 0.1;
  limits.max_linear_damping = 20.0;
  limits.min_quadratic_damping = 0.2;
  limits.max_quadratic_damping = 30.0;
  limits.min_static_wrench = -5.0;
  limits.max_static_wrench = 5.0;

  model.setLimits(limits);
  model.setEffectiveInertia(Vector6::Constant(-1.0));
  model.setLinearDamping(Vector6::Constant(-2.0));
  model.setQuadraticDamping(Vector6::Constant(-3.0));
  model.setStaticWrench(Vector6::Constant(-10.0));

  EXPECT_DOUBLE_EQ(model.effectiveInertia()(0), limits.min_effective_inertia);
  EXPECT_DOUBLE_EQ(model.linearDamping()(0), limits.min_linear_damping);
  EXPECT_DOUBLE_EQ(model.quadraticDamping()(0), limits.min_quadratic_damping);
  EXPECT_DOUBLE_EQ(model.staticWrench()(0), limits.min_static_wrench);
}

}  // namespace
}  // namespace cirtesub_controllers
