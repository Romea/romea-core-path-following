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

#include "romea_core_path_following/PathFollowing.hpp"
#include "romea_core_path_following/PathFollowingLogs.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
template<typename CommandType>
PathFollowingBase<CommandType>::PathFollowingBase(
  std::unique_ptr<PathSectionFollowing> pathSectionFollowing,
  std::shared_ptr<SimpleFileLogger> logger)
: pathSectionFollowing_(std::move(pathSectionFollowing)),
  logger_(logger)
{

}

//-----------------------------------------------------------------------------
template<typename CommandType>
CommandType PathFollowingBase<CommandType>::computeCommand_(
  const PathFollowingSetPoint & userSetPoint,
  const PathMatchedPoint2D & matchedPoint,
  const OdometryMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  PathFollowingSetPoint setPoint = evaluateSetPoint(userSetPoint, matchedPoint);

  CommandType command;
  if (setPoint.lateralDeviation >= 0) {
    command = pathSectionFollowing_->computeCommand(
      setPoint,
      matchedPoint.frenetPose,
      matchedPoint.pathPosture,
      matchedPoint.futureCurvature,
      odometryMeasure,
      filteredTwist);
  } else {
    command = pathSectionFollowing_->computeCommand(
      setPoint,
      reverse(matchedPoint.frenetPose),
      reverse(matchedPoint.pathPosture),
      -matchedPoint.futureCurvature,
      odometryMeasure,
      filteredTwist);
  }

  if (logger_ != nullptr) {
    log(*logger_, userSetPoint);
    log(*logger_, matchedPoint);
    log(*logger_, odometryMeasure);
    log(*logger_, filteredTwist);
    pathSectionFollowing_->log(*logger_);
    log(*logger_, command);
  }

  return command;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingBase<CommandType>::reset()
{
  pathSectionFollowing_->reset();
}

//-----------------------------------------------------------------------------
OneAxleSteeringPathFollowing::OneAxleSteeringPathFollowing(
  std::unique_ptr<PathSectionFollowing> pathSectionFollowing,
  std::shared_ptr<SimpleFileLogger> logger)
: PathFollowingBase<OneAxleSteeringCommand>(std::move(pathSectionFollowing), logger)
{
}

//-----------------------------------------------------------------------------
OneAxleSteeringCommand OneAxleSteeringPathFollowing::computeCommand(
  const PathFollowingSetPoint & userSetPoint,
  const std::vector<PathMatchedPoint2D> & matchedPoints,
  const OneAxleSteeringMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  // ADD FSM
  return computeCommand_(userSetPoint, matchedPoints[0], odometryMeasure, filteredTwist);
}

//-----------------------------------------------------------------------------
TwoAxleSteeringPathFollowing::TwoAxleSteeringPathFollowing(
  std::unique_ptr<PathSectionFollowing> pathSectionFollowing,
  std::shared_ptr<SimpleFileLogger> logger)
: PathFollowingBase<TwoAxleSteeringCommand>(std::move(pathSectionFollowing), logger)
{
}

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand TwoAxleSteeringPathFollowing::computeCommand(
  const PathFollowingSetPoint & userSetPoint,
  const std::vector<PathMatchedPoint2D> & matchedPoints,
  const TwoAxleSteeringMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  // ADD FSM
  return computeCommand_(userSetPoint, matchedPoints[0], odometryMeasure, filteredTwist);
}

}  // namespace core
}  // namespace romea
