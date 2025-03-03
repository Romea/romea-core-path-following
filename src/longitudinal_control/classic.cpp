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

#include "romea_core_path_following/utils.hpp"
#include "romea_core_path_following/longitudinal_control/classic.hpp"

namespace romea
{
namespace core
{
namespace path_following
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
  const PathPosture2D & /*path_posture*/,
  const OdometryMeasure & odometry_measure,
  const Twist2D & /*filtered_twist*/)
{
  // return setPoint.linearSpeed;
  double desiredLinearSpeed = setpoint.linear_speed;
  double desiredLateralDeviation = setpoint.lateral_deviation;

  const double & EcartLat = frenet_pose.lateralDeviation;
  const double & EcartAng = frenet_pose.courseDeviation;
  const double & steering = get_front_steering_angle(odometry_measure);

  double flag = 1;
  if (fabs(steering) < 10 / 180. * M_PI) {flag = 0;}

  double linearSpeedCommand =
    fabs(desiredLinearSpeed) -
    2 * (EcartLat - desiredLateralDeviation) * (EcartLat - desiredLateralDeviation) -
    10 * EcartAng * EcartAng - 1 * flag * fabs(steering);

  if (linearSpeedCommand < minimal_linear_speed_) {
    linearSpeedCommand = minimal_linear_speed_;
  }

  return std::copysign(linearSpeedCommand, desiredLinearSpeed);
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
void LongitudinalControlClassic<SkidSteeringCommand>::log(
  SimpleFileLogger & /*logger*/)
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

}  // namespace path_following
}  // namespace core
}  // namespace romea
