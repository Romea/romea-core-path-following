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

#include <romea_core_control/command/FollowTrajectoryDesbosGeneric.hpp>

// local
#include "romea_core_path_following/lateral_control/desbos_generic.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

LateralControlDesbosGeneric<SkidSteeringCommand>::LateralControlDesbosGeneric(
  double /*sample_period*/,
  double /*wheelbase*/,
  const MobileBaseInertia & /*inertia*/,
  const Parameters & parameters)
: gains(parameters.gains),
  lateral_control_(
    {
      parameters.gains.kp,
      parameters.gains.kd,
      parameters.gains.ks,
      parameters.adaptive_gains,
    })
{
}

SkidSteeringCommand LateralControlDesbosGeneric<SkidSteeringCommand>::compute_command(
  const SetPoint & set_point,
  const CommandLimits & command_limits,
  const PathFrenetPose2D & frenet_pose,
  const PathPosture2D & path_posture,
  const double & future_path_curvature,
  const OdometryMeasure & odometry_measure,
  const SkidSlidingParameters & slidings)
{
  auto cur_gains = gains.load();
  lateral_control_.set_gains(cur_gains.kp, cur_gains.kd, cur_gains.ks);

  double theta_error = NAN;
  double tau = NAN;
  double osc_eta = NAN;
  double osc_amp = NAN;

  double angular_speed = lateral_control_.compute_angular_speed(
    frenet_pose.lateralDeviation,
    frenet_pose.courseDeviation,
    command_limits.angularSpeed.upper(),
    path_posture.curvature,
    future_path_curvature,
    odometry_measure.longitudinalSpeed,
    set_point.linear_speed,
    0,  // lateral_slip (generic sliding)
    0,  // angular_slip (generic sliding)
    slidings.linear_speed_disturbance,
    slidings.slip_angle,
    slidings.angular_speed_disturbance,
    target_course_,
    theta_error,
    tau,
    osc_eta,
    osc_amp);

  return {set_point.linear_speed, angular_speed};
}

void LateralControlDesbosGeneric<SkidSteeringCommand>::log(SimpleFileLogger & logger)
{
  logger.addEntry("target_course", target_course_);
}

}  // namespace romea::core::path_following
