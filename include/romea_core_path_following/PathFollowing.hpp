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

#ifndef ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_


// std
#include <memory>
#include <utility>

// romea
#include "romea_core_path_following/PathFollowingTraits.hpp"

namespace romea
{
namespace core
{

template<typename CommandType>
class PathFollowingBase
{
public:
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using CommandLimits = typename PathFollowingTraits<CommandType>::Limits;

  PathFollowingBase();

  virtual ~PathFollowingBase() = default;

  virtual CommandType computeCommand(
    const PathFollowingSetPoint & userSetPoint,
    const std::vector<PathMatchedPoint2D> & matchedPoints,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)
  {
    PathFollowingSetPoint setPoint = evaluateSetPoint(userSetPoint, matchedPoint);

    if (setPoint.lateralDeviation >= 0) {
      return computeCommand_(
        setPoint,
        matchedPoint.frenetPose,
        matchedPoint.pathPosture,
        matchedPoint.futureCurvature,
        odometryMeasure,
        filteredTwist);
    } else {
      return computeCommand_(
        setPoint,
        reverse(matchedPoint.frenetPose),
        reverse(matchedPoint.pathPosture),
        -matchedPoint.futureCurvature,
        odometryMeasure,
        filteredTwist);
    }
  }

  virtual virtual void reset() = 0;

protected:
  virtual CommandType computeCommand_(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist) = 0;

};

template<typename LateralControl, typename LongitudinalControl>
class PathFollowingWithoutSlidingObserver : public PathFollowingBase<LateralControl::Command>
{
  PathFollowing(
    CommandLimits commandLimits_,
    std::shared_ptr<LateralControl> lateralControl,
    std::shared_ptr<LongitudinalControl> longitudinalControl,
    std::shared_ptr<SimpleFileLogger> logger)
    : commandLimits_(commandLimits_),
    lateralControl(std::move(lateralControl)),
    longitudinalControl(std::move(longitudinalControl)),
    logger(logger_),
  {
  }

  CommandType computeCommand_(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)
  {
    auto command = lateralControl_->computeCommand(
      setPoint, frenetPose, pathPosture, futureCurvature, odometryMeasure, filteredTwist);

    command.longitudinalSpeed = longitudinalControl_->computeLinearSpeed(
      setPoint, frenetPose, pathPosture, odometryMeasure, filteredTwist);

    if (logger_ != nullptr) {
      log(*logger_, userSetPoint);
      log(*logger_, matchedPoint);
      log(*logger_, odometryMeasure);
      log(*logger_, filteredTwist);
      lateralControl_->log(*logger_);
      longitudinalControl_->log(*logger_);
      log(*logger_, command);
    }

    return command;
  }

  void reset() override
  {
    lateralControl_->reset();
    longitudinalControl_->reset();
  }


  CommandLimits commandLimits_;
  std::shared_ptr<LateralControl> lateralControl_;
  std::shared_ptr<LongitudinalControl> longitudinalControl_;
  std::shared_ptr<SimpleFileLogger> logger_;
};

template<typename LateralControl, typename LongitudinalControl, typename SlidingObserver>
class PathFollowingWithSlidingObserver : public PathFollowingBase<LateralControl::Command>
{
  PathFollowing(
    CommandLimits commandLimits_,
    std::shared_ptr<LateralControl> lateralControl,
    std::shared_ptr<LongitudinalControl> longitudinalControl,
    std::shared_ptr<SlidingObserver> slidingObserver,
    std::shared_ptr<SimpleFileLogger> logger)
    : commandLimits_(commandLimits_),
    lateralControl(std::move(lateralControl)),
    longitudinalControl(std::move(longitudinalControl)),
    slidingObserver_(std::move(slidingObserver)),
    logger(logger_),
  {
  }

  CommandType computeCommand_(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)
  {

    if constexpr (std::is_same_v<LateralControl::Slidings, SlidingObserver::Slidings>) {
      auto slidings = slidingObserver_->computeSlidings(
        frenetPose, pathPosture, odometryMeasure, filteredTwist)
    } else {
    }

    auto command = lateralControl_->computeCommand(
      setPoint, frenetPose, pathPosture, futureCurvature, odometryMeasure, filteredTwist, slidings);

    command.longitudinalSpeed = longitudinalControl_->computeLinearSpeed(
      setPoint, frenetPose, pathPosture, odometryMeasure, filteredTwist);


    if (logger_ != nullptr) {
      log(*logger_, userSetPoint);
      log(*logger_, matchedPoint);
      log(*logger_, odometryMeasure);
      log(*logger_, filteredTwist);
      lateralControl_->log(*logger_);
      longitudinalControl_->log(*logger_);
      slidingObserver_->log(logger_);
      log(*logger_, command);
    }

    return command;
  }

  void reset() override
  {
    lateralControl_->reset();
    longitudinalControl_->reset();
    slidingObserver_->reset();
  }

private:
  CommandLimits commandLimits_;
  std::shared_ptr<LateralControl> lateralControl_;
  std::shared_ptr<LongitudinalControl> longitudinalControl_;
  std::shared_ptr<SlidingObserver> slidingObserver_;
  std::shared_ptr<SimpleFileLogger> logger_;
};


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_
