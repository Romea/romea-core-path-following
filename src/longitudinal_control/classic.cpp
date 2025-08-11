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
#include "romea_core_path_following/longitudinal_control/classic.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

//-----------------------------------------------------------------------------
LongitudinalControlClassic<OneAxleSteeringCommand>::LongitudinalControlClassic(
  const Parameters & parameters)
: minimal_linear_speed_(parameters.minimal_linear_speed)
{
}

//-----------------------------------------------------------------------------
double LongitudinalControlClassic<OneAxleSteeringCommand>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D & frenet_pose,
  const PathPosture2D & path_posture,
  const OdometryMeasure & odometry_measure,
  const Twist2D & /*filtered_twist*/)
{
  double desired_linear_speed = setpoint.linear_speed;
  // parameters
  double Ymax=0.20; //Maximal admissible error
  double YmaxAbs=5; // Maximal possible error before stopping algo
  double Tau=1.0;  // settling time on angular speed
  double current_curvature = path_posture.curvature;
  double future_curvature = 0.0;
  double v_max=2.5;

  // Max speed computation after initialization on path (curvarture transition)
  double Max_Speed = sqrt(Ymax/(Tau*fabs(future_curvature-current_curvature)));

  

  if (Max_Speed<desired_linear_speed) 
    desired_linear_speed  = Max_Speed;


  // Max speed computation for initial error 
  double Max_Speed2=v_max*cos(fabs(2*frenet_pose.courseDeviation))*cos(fabs(frenet_pose.lateralDeviation/YmaxAbs));

  if (Max_Speed2<desired_linear_speed) 
    desired_linear_speed  = Max_Speed2;
  // temporary disable speed control
  return desired_linear_speed;



  // if (std::abs(desired_linear_speed) < 1e-3) {
  //   return desired_linear_speed;
  // }
  //
  // double lat_error = frenet_pose.lateralDeviation - setpoint.lateral_deviation;
  // double ang_error = frenet_pose.courseDeviation - setpoint.course_deviation;
  // double steering_angle = get_front_steering_angle(odometry_measure);
  //
  // double linear_speed_command = std::abs(setpoint.linear_speed);
  // linear_speed_command -= 2 * lat_error * lat_error;
  // linear_speed_command -= 10 * ang_error * ang_error;
  //
  // if (fabs(steering_angle) < 10 / 180. * M_PI) {
  //   linear_speed_command -= 1 * std::abs(steering_angle);
  // }
  //
  // linear_speed_command = std::max(linear_speed_command, minimal_linear_speed_);
  //
  // return std::copysign(linear_speed_command, desired_linear_speed);
}

//-----------------------------------------------------------------------------
void LongitudinalControlClassic<OneAxleSteeringCommand>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void LongitudinalControlClassic<OneAxleSteeringCommand>::reset()
{
}

//-----------------------------------------------------------------------------
LongitudinalControlClassic<SkidSteeringCommand>::LongitudinalControlClassic(
  const Parameters & parameters)
: minimal_linear_speed_(parameters.minimal_linear_speed)
{
}

//-----------------------------------------------------------------------------
double LongitudinalControlClassic<SkidSteeringCommand>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D & /*frenet_pose*/,
  const PathPosture2D & /*path_posture*/,
  const OdometryMeasure & /*odometry_measure*/,
  const Twist2D & /*filtered_twist*/)
{
  return setpoint.linear_speed;
}

//-----------------------------------------------------------------------------
void LongitudinalControlClassic<SkidSteeringCommand>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void LongitudinalControlClassic<SkidSteeringCommand>::reset()
{
}

//-----------------------------------------------------------------------------
LongitudinalControlClassic<TwoAxleSteeringCommand>::LongitudinalControlClassic(
  const Parameters & parameters)
: minimal_linear_speed_(parameters.minimal_linear_speed)
{
}

//-----------------------------------------------------------------------------
double LongitudinalControlClassic<TwoAxleSteeringCommand>::compute_linear_speed(
  const SetPoint & setpoint,
  const PathFrenetPose2D & /*frenet_pose*/,
  const PathPosture2D & /*path_posture*/,
  const OdometryMeasure & /*odometry_measure*/,
  const Twist2D & /*filtered_twist*/)
{
  return setpoint.linear_speed;
}

//-----------------------------------------------------------------------------
void LongitudinalControlClassic<TwoAxleSteeringCommand>::log(SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void LongitudinalControlClassic<TwoAxleSteeringCommand>::reset()
{
}

} // namespace romea::core::path_following


