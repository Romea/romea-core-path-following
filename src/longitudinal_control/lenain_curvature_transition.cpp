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
#include "romea_core_path_following/longitudinal_control/lenain_curvature_transition.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

//-----------------------------------------------------------------------------
template<class CommandType>
LongitudinalControlLenainCurvatureTransition<
  CommandType>::LongitudinalControlLenainCurvatureTransition(const Parameters & parameters)
: minimal_linear_speed_(parameters.minimal_linear_speed)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
double LongitudinalControlLenainCurvatureTransition<CommandType>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D & frenet_pose,
  const PathPosture2D & path_posture,
  double future_curvature,
  const OdometryMeasure & /*odometry_measure*/,
  const Twist2D & /*filtered_twist*/)
{
  double desired_linear_speed = setpoint.linear_speed;
  // parameters
  double Ymax = 0.20;  //Maximal admissible error
  double YmaxAbs = 5;  // Maximal possible error before stopping algo
  double Tau = 1.0;    // settling time on angular speed
  double current_curvature = path_posture.curvature;
  double v_max = 2.5;

  // Max speed computation after initialization on path (curvarture transition)
  double Max_Speed = sqrt(Ymax / (Tau * fabs(future_curvature - current_curvature)));

  desired_linear_speed = std::min(Max_Speed, desired_linear_speed);

  // Max speed computation for initial error
  double Max_Speed2 = v_max * cos(fabs(2 * frenet_pose.courseDeviation)) *
                      cos(fabs(frenet_pose.lateralDeviation / YmaxAbs));

  desired_linear_speed = std::min(Max_Speed2, desired_linear_speed);

  return desired_linear_speed;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void LongitudinalControlLenainCurvatureTransition<CommandType>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
void LongitudinalControlLenainCurvatureTransition<CommandType>::reset()
{
}


template class LongitudinalControlLenainCurvatureTransition<OneAxleSteeringCommand>;
template class LongitudinalControlLenainCurvatureTransition<TwoAxleSteeringCommand>;
template class LongitudinalControlLenainCurvatureTransition<SkidSteeringCommand>;

}  // namespace romea::core::path_following
