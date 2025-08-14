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

// std
#include <algorithm>
#include <cstdlib>
#include <iostream>

// local
#include "romea_core_path_following/longitudinal_control/curvature_transition.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

//-----------------------------------------------------------------------------
template<class CommandType>
LongitudinalControlCurvatureTransition<CommandType>::LongitudinalControlCurvatureTransition(
  const Parameters & parameters)
: params_(parameters)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
double LongitudinalControlCurvatureTransition<CommandType>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D & frenet_pose,
  const PathPosture2D & path_posture,
  double future_curvature,
  const OdometryMeasure & /*odometry_measure*/,
  const Twist2D & /*filtered_twist*/)
{
  double desired_linear_speed = setpoint.linear_speed;
  const double current_curvature = path_posture.curvature;

  // parameters
  // y_max: Maximal admissible error
  // tau: settling time on angular speed
  // n: parameters for settling convergence ratio between lat and ang dynamics >3
  // d: Settling distance in m for lateral deviation (given by control law gains)
  const double y_max = params_.lateral_error_max;
  const double tau = params_.settling_time;
  const double d = params_.settling_distance;
  const double n = params_.convergence_ratio;

  // Max speed computation after initialization on path (curvarture transition)
  double max_speed = std::sqrt(y_max / (tau * std::fabs(future_curvature - current_curvature)));

  // Max speed computation for initial error
  double max_speed2 =
    (d / (3 * tau)) *
    ((n + 1) / n + 0.3333 * std::log(y_max / std::abs(frenet_pose.lateralDeviation)));

  // apply constraints to linear_speed
  desired_linear_speed = std::min(max_speed, desired_linear_speed);
  desired_linear_speed = std::min(max_speed2, desired_linear_speed);
  desired_linear_speed = std::max(params_.minimal_linear_speed, desired_linear_speed);

  // temporary disable speed control
  return desired_linear_speed;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void LongitudinalControlCurvatureTransition<CommandType>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
void LongitudinalControlCurvatureTransition<CommandType>::reset()
{
}

template class LongitudinalControlCurvatureTransition<OneAxleSteeringCommand>;
template class LongitudinalControlCurvatureTransition<TwoAxleSteeringCommand>;
template class LongitudinalControlCurvatureTransition<SkidSteeringCommand>;

}  // namespace romea::core::path_following
