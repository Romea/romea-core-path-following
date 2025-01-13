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


#ifndef ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGUTILS_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGUTILS_HPP_

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

double getFrontSteeringAngle(const TwoAxleSteeringCommand & command);

double getFrontSteeringAngle(const OneAxleSteeringCommand & command);

double getRearSteeringAngle(const TwoAxleSteeringCommand & command);

double getRearSteeringAngle(const OneAxleSteeringCommand & command);

double getMaximalAngularSpeed(const SkidSteeringCommandLimits & limits);

double getMaximalFrontSteeringAngle(const TwoAxleSteeringCommandLimits & limits);

double getMaximalFrontSteeringAngle(const OneAxleSteeringCommandLimits & limits);

double getMaximalRearSteeringAngle(const TwoAxleSteeringCommandLimits & limits);

double getMaximalRearSteeringAngle(const OneAxleSteeringCommandLimits & limits);

double setFrontSteeringAngle(const double frontSteeringAngle, TwoAxleSteeringCommand & command);

double setRearSteeringAngle(const double rearSteeringAngle, TwoAxleSteeringCommand & command);

double setFrontSteeringAngle(const double frontSteeringAngle, OneAxleSteeringCommand & command);

double setRearSteeringAngle(const double rearSteeringAngle, OneAxleSteeringCommand & command);

OneAxleSteeringMeasure toOneAxleSteeringMeasure(
  const SkidSteeringMeasure & skidSteeringMeasure,
  const double & wheelbase);

SkidSteeringCommand toSkidSteeringCommand(
  const OneAxleSteeringCommand & oneAxleSteeringCommand,
  const double & wheelbase);

OneAxleSteeringCommandLimits toOneAxleSteeringCommandLimits(
  const SkidSteeringCommandLimits skidSteeringCommandLimits);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGUTILS_HPP_
