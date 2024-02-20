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
#include "romea_core_path_following/PathFollowingUtils.hpp"
#include \
  "romea_core_path_following/lateral_control/PathFollowingLateralControlPredictiveOneAxleSteering.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathFollowingLateralControlPredictiveOneAxleSteering::
PathFollowingLateralControlPredictiveOneAxleSteering(
  const double & wheelbase,
  const MobileBaseInertia & /*inertia*/,
  const Parameters & parameters)
: gains_(parameters.gains),
  lateralControl_(wheelbase,
    {parameters.gains.frontKD, 0.0,
      parameters.horizon,
      parameters.a0,
      parameters.a1,
      parameters.b1,
      parameters.b2})
{
}

//-----------------------------------------------------------------------------
OneAxleSteeringCommand PathFollowingLateralControlPredictiveOneAxleSteering::computeCommand(
  const PathFollowingSetPoint & setPoint,
  const CommandLimits & commandLimits,
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const double & futurePathCurvature,
  const OdometryMeasure & odometryMeasure,
  const AxleSteeringSlidings & slidings)
{
  lateralControl_.setFrontKP(gains_.load().frontKD);

  auto steeringAngles = lateralControl_.computeSteeringAngles(
    frenetPose.lateralDeviation,
    frenetPose.courseDeviation,
    pathPosture.curvature,
    futurePathCurvature,
    getFrontSteeringAngle(odometryMeasure),
    getRearSteeringAngle(odometryMeasure),
    slidings.frontSteeringAngle,
    slidings.rearSteeringAngle,
    getMaximalFrontSteeringAngle(commandLimits),
    getMaximalRearSteeringAngle(commandLimits),
    setPoint.lateralDeviation,
    setPoint.courseDeviation,
    setPoint.lateralDeviation);

  return OneAxleSteeringCommand(setPoint.linearSpeed, steeringAngles.front);
}

//-----------------------------------------------------------------------------
void PathFollowingLateralControlPredictiveOneAxleSteering::updateGains(const Gains & gains)
{
  gains_.store(gains);
}


}  // namespace core
}  // namespace romea
