// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// romea
#include <cmath>
#include <romea_core_control/command/FollowTrajectoryDesbosGenericPredictive.hpp>

// local
#include "romea_core_path_following/lateral_control/desbos_generic_predictive.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

LateralControlDesbosGenericPredictive<SkidSteeringCommand>::LateralControlDesbosGenericPredictive(
  double sampling_period,
  double /*wheelbase*/,
  const MobileBaseInertia & /*inertia*/,
  const Parameters & parameters)
: gains(parameters.gains),
  lateral_control_(
    sampling_period,
    {
      parameters.gains.kp,
      parameters.gains.kd,
      parameters.gains.ks,
      parameters.alpha,
      parameters.a0,
      parameters.a1,
      parameters.b1,
      parameters.b2,
      parameters.adaptive_gains,
      parameters.lmpc,
      parameters.horizon,
      parameters.model_order,
    }),
  sampling_period_(sampling_period)
{
}

SkidSteeringCommand LateralControlDesbosGenericPredictive<SkidSteeringCommand>::compute_command(
  const SetPoint & set_point,
  const CommandLimits &  /*command_limits*/,
  const PathFrenetPose2D & frenet_pose,
  const PathPosture2D & path_posture,
  const double & future_path_curvature,
  const OdometryMeasure & odometry_measure,
  const SkidSlidingParameters &  /*slidings*/)
{
  auto cur_gains = gains.load();
  lateral_control_.set_gains(cur_gains.kp, cur_gains.kd, cur_gains.ks);

  double omega_d = NAN;
  double theta_consigne = NAN;
  double tau = NAN;
  double curv_pred = NAN;
  // double osc_amp = NAN;

  double angular_speed = lateral_control_.compute_angular_speed(
    frenet_pose.lateralDeviation,
    frenet_pose.courseDeviation,
    path_posture.curvature,
    future_path_curvature,
    odometry_measure.longitudinalSpeed,
    set_point.linear_speed,
    odometry_measure.angularSpeed,
    0,  // lateral_slip (generic sliding)
    0,  // angular_slip (generic sliding)
    0,  // courbe 1, unknown (TODO: identify it)
    0,  // courbe 2, unknown (TODO: identify it)
    0,  // lambda, unknown (TODO: identify it)
    omega_d,
    theta_consigne,
    curv_pred,
    tau,
    sampling_period_);

  return {set_point.linear_speed, angular_speed};
}

void LateralControlDesbosGenericPredictive<SkidSteeringCommand>::log(SimpleFileLogger & logger)
{
  logger.addEntry("target_course", target_course_);
}

}  // namespace romea::core::path_following
