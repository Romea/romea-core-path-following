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
#include "romea_core_path_following/command/PathSectionFollowingPredictive.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
template<class CommandType>
PathSectionFollowingPredictive<CommandType>::PathSectionFollowingPredictive(
  const double & wheelbase,
  const MobileBaseInertia & /*inertia*/,
  const LongitudinalControlParameters & longitudinalControlParameters,
  const LateralControlParameters & lateralControlParameters,
  const CommandLimits & commandLimits)
: PathSectionFollowingBase<CommandType>(longitudinalControlParameters, commandLimits),
  lateralControl_(wheelbase, lateralControlParameters)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>

SteeringAngles PathSectionFollowingPredictive<CommandType>::computeSteeringAngles_(
  const PathFollowingSetPoint & setPoint,
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const double & futurePathCurvature,
  const OdometryMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  SlidingAngles slidingAngles = this->compute_sliding_angles_(
    frenetPose, pathPosture, odometryMeasure, filteredTwist);

  return lateralControl_.computeSteeringAngles(
    frenetPose.lateralDeviation,
    frenetPose.courseDeviation,
    pathPosture.curvature,
    futurePathCurvature,
    getFrontSteeringAngle(odometryMeasure),
    getRearSteeringAngle(odometryMeasure),
    slidingAngles.front,
    slidingAngles.rear,
    getMaximalFrontSteeringAngle(this->commandLimits_),
    getMaximalRearSteeringAngle(this->commandLimits_),
    setPoint.lateralDeviation,
    setPoint.courseDeviation,
    setPoint.lateralDeviation);
}

template class PathSectionFollowingPredictive<OneAxleSteeringCommand>;
template class PathSectionFollowingPredictive<TwoAxleSteeringCommand>;

}  // namespace core
}  // namespace romea
