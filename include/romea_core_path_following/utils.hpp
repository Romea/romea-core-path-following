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


#ifndef ROMEA_CORE_PATH_FOLLOWING__UTILS_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__UTILS_HPP_

#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp>

namespace romea
{
namespace core
{
namespace path_following
{

double get_front_steering_angle(const TwoAxleSteeringCommand & command);

double get_front_steering_angle(const OneAxleSteeringCommand & command);

double get_rear_steering_angle(const TwoAxleSteeringCommand & command);

double get_rear_steering_angle(const OneAxleSteeringCommand & command);

double get_maximal_angular_speed(const SkidSteeringCommandLimits & limits);

double get_maximal_front_steering_angle(const TwoAxleSteeringCommandLimits & limits);

double get_maximal_front_steering_angle(const OneAxleSteeringCommandLimits & limits);

double get_maximal_rear_steering_angle(const TwoAxleSteeringCommandLimits & limits);

double get_maximal_rear_steering_angle(const OneAxleSteeringCommandLimits & limits);

double set_front_steering_angle(const double frontSteeringAngle, TwoAxleSteeringCommand & command);

double set_rear_steering_angle(const double rearSteeringAngle, TwoAxleSteeringCommand & command);

double set_front_steering_angle(const double frontSteeringAngle, OneAxleSteeringCommand & command);

double set_rear_steering_angle(const double rearSteeringAngle, OneAxleSteeringCommand & command);

OneAxleSteeringMeasure to_one_axle_steering_measure(
  const SkidSteeringMeasure & skidSteeringMeasure,
  const double & wheelbase);

SkidSteeringCommand to_skid_steering_command(
  const OneAxleSteeringCommand & oneAxleSteeringCommand,
  const double & wheelbase);

OneAxleSteeringCommandLimits to_one_axle_steering_command_limits(
  const SkidSteeringCommandLimits skidSteeringCommandLimits);

}  // namespace path_following
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGUTILS_HPP_
