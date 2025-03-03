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

namespace romea
{
namespace core
{
namespace path_following
{

//-----------------------------------------------------------------------------
double get_front_steering_angle(const TwoAxleSteeringCommand & command)
{
  return command.frontSteeringAngle;
}

//-----------------------------------------------------------------------------
double get_front_steering_angle(const OneAxleSteeringCommand & command)
{
  return command.steeringAngle;
}

//-----------------------------------------------------------------------------
double get_rear_steering_angle(const TwoAxleSteeringCommand & command)
{
  return command.rearSteeringAngle;
}

//-----------------------------------------------------------------------------
double get_rear_steering_angle(const OneAxleSteeringCommand & /*command*/)
{
  return 0;
}

//-----------------------------------------------------------------------------
double get_maximal_angular_speed(const SkidSteeringCommandLimits & limits)
{
  return limits.angularSpeed.upper();
}

//-----------------------------------------------------------------------------
double get_maximal_front_steering_angle(const TwoAxleSteeringCommandLimits & limits)
{
  return limits.frontSteeringAngle.upper();
}

//-----------------------------------------------------------------------------
double get_maximal_front_steering_angle(const OneAxleSteeringCommandLimits & limits)
{
  return limits.steeringAngle.upper();
}

//-----------------------------------------------------------------------------
double get_maximal_rear_steering_angle(const TwoAxleSteeringCommandLimits & limits)
{
  return limits.rearSteeringAngle.upper();
}

//-----------------------------------------------------------------------------
double get_maximal_rear_steering_angle(const OneAxleSteeringCommandLimits & /*limits*/)
{
  return 0;
}

//-----------------------------------------------------------------------------
OneAxleSteeringMeasure to_one_axle_steering_measure(
  const SkidSteeringMeasure & skidSteeringMeasure,
  const double & wheelbase)
{
  OneAxleSteeringMeasure oneAxleSteeringMeasure;
  oneAxleSteeringMeasure.longitudinalSpeed = skidSteeringMeasure.longitudinalSpeed;
  if (std::abs(skidSteeringMeasure.longitudinalSpeed) > 0.01) {
    oneAxleSteeringMeasure.steeringAngle = std::atan(
      wheelbase * skidSteeringMeasure.angularSpeed / skidSteeringMeasure.longitudinalSpeed);
  } else {
    oneAxleSteeringMeasure.steeringAngle = 0;
  }
  return oneAxleSteeringMeasure;
}

//-----------------------------------------------------------------------------
SkidSteeringCommand to_skid_steering_command(
  const OneAxleSteeringCommand & oneAxleSteeringCommand,
  const double & wheelbase)
{
  return {
    oneAxleSteeringCommand.longitudinalSpeed,
    oneAxleSteeringCommand.longitudinalSpeed *
    std::tan(oneAxleSteeringCommand.steeringAngle) / wheelbase
  };
}

//-----------------------------------------------------------------------------
OneAxleSteeringCommandLimits to_one_axle_steering_command_limits(
  const SkidSteeringCommandLimits skidSteeringCommandLimits)
{
  OneAxleSteeringCommandLimits oneAxleSteeringCommandLimits;
  oneAxleSteeringCommandLimits.longitudinalSpeed = skidSteeringCommandLimits.longitudinalSpeed;
  return oneAxleSteeringCommandLimits;
}

}  // namespace path_following
}  // namespace core
}  // namespace romea
