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
#include "romea_core_path_following/longitudinal_control/constant.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

//-----------------------------------------------------------------------------
LongitudinalControlConstant<OneAxleSteeringCommand>::LongitudinalControlConstant(
  const Parameters & parameters)
: minimal_linear_speed_(parameters.minimal_linear_speed)
{
}

//-----------------------------------------------------------------------------
double LongitudinalControlConstant<OneAxleSteeringCommand>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D &  /*frenet_pose*/,
  const PathPosture2D & /*path_posture*/,
  double  /*future_curvature*/,
  const OdometryMeasure &  /*odometry_measure*/,
  const Twist2D & /*filtered_twist*/)
{
  return setpoint.linear_speed;
}

//-----------------------------------------------------------------------------
void LongitudinalControlConstant<OneAxleSteeringCommand>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void LongitudinalControlConstant<OneAxleSteeringCommand>::reset()
{
}

//-----------------------------------------------------------------------------
LongitudinalControlConstant<SkidSteeringCommand>::LongitudinalControlConstant(
  const Parameters & parameters)
: minimal_linear_speed_(parameters.minimal_linear_speed)
{
}

//-----------------------------------------------------------------------------
double LongitudinalControlConstant<SkidSteeringCommand>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D & /*frenet_pose*/,
  const PathPosture2D & /*path_posture*/,
  double  /*future_curvature*/,
  const OdometryMeasure & /*odometry_measure*/,
  const Twist2D & /*filtered_twist*/)
{
  return setpoint.linear_speed;
}

//-----------------------------------------------------------------------------
void LongitudinalControlConstant<SkidSteeringCommand>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void LongitudinalControlConstant<SkidSteeringCommand>::reset()
{
}

//-----------------------------------------------------------------------------
LongitudinalControlConstant<TwoAxleSteeringCommand>::LongitudinalControlConstant(
  const Parameters & parameters)
: minimal_linear_speed_(parameters.minimal_linear_speed)
{
}

//-----------------------------------------------------------------------------
double LongitudinalControlConstant<TwoAxleSteeringCommand>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D & /*frenet_pose*/,
  const PathPosture2D & /*path_posture*/,
  double  /*future_curvature*/,
  const OdometryMeasure & /*odometry_measure*/,
  const Twist2D & /*filtered_twist*/)
{
  return setpoint.linear_speed;
}

//-----------------------------------------------------------------------------
void LongitudinalControlConstant<TwoAxleSteeringCommand>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void LongitudinalControlConstant<TwoAxleSteeringCommand>::reset()
{
}

}  // namespace romea::core::path_following
