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
#include <vector>

// romea
#include "romea_core_common/geometry/Twist2D.hpp"
#include "romea_core_common/log/SimpleFileLogger.hpp"
#include "romea_core_path/PathFrenetPose2D.hpp"
#include "romea_core_path/PathPosture2D.hpp"
#include "romea_core_path_following/PathFollowingFSM.hpp"
#include "romea_core_path_following/PathFollowingLogs.hpp"
#include "romea_core_path_following/PathFollowingTraits.hpp"
#include "romea_core_path_following/PathFollowingSetPoint.hpp"
#include "romea_core_path_following/PathFollowingUtils.hpp"


namespace romea
{
namespace core
{

template<typename CommandType>
class PathFollowingBase
{
public:
  using Logger = core::SimpleFileLogger;
  using FSM = PathFollowingFSM<CommandType>;
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using CommandLimits = typename PathFollowingTraits<CommandType>::Limits;

  PathFollowingBase()
  : logger_(nullptr) {}

  virtual ~PathFollowingBase() = default;

  virtual void registerLogger(std::shared_ptr<Logger> logger)
  {
    logger_ = logger;
  }

  virtual void setStopAtTheEnd(bool value)
  {
    fsm_.setStopAtTheEnd(value);
  }

  virtual PathFollowingFSMStatus getStatus()
  {
    return fsm_.getStatus();
  }

  virtual std::optional<CommandType> computeCommand(
    const PathFollowingSetPoint & userSetPoint,
    const CommandLimits & commandLimits,
    const std::vector<PathMatchedPoint2D> & matchedPoints,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)
  {
    fsm_.updateMatchedPoints(matchedPoints);

    if (this->fsm_.getStatus() == PathFollowingFSMStatus::FAILED ||
      this->fsm_.getStatus() == PathFollowingFSMStatus::FINISH)
    {
      return {};
    }

    auto matchedPoint = *findMatchedPointBySectionIndex(
      matchedPoints, fsm_.getCurrentSectionIndex());

    auto setPoint = evaluateSetPoint(userSetPoint, matchedPoint);

    if (this->fsm_.getStatus() == PathFollowingFSMStatus::STOP ||
      this->fsm_.getStatus() == PathFollowingFSMStatus::CHANGE_DIRECTION)
    {
      setPoint.linearSpeed = 0;
    }

    CommandType command;
    if (direction(matchedPoint) >= 0) {
      command = computeCommand(
        setPoint,
        commandLimits,
        matchedPoint.frenetPose,
        matchedPoint.pathPosture,
        matchedPoint.futureCurvature,
        odometryMeasure,
        filteredTwist);
    } else {
      command = computeCommand(
        setPoint,
        commandLimits,
        reverse(matchedPoint.frenetPose),
        reverse(matchedPoint.pathPosture),
        -matchedPoint.futureCurvature,
        odometryMeasure,
        filteredTwist);
    }

    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>)
    {
      if (this->fsm_.getStatus() == PathFollowingFSMStatus::STOP ||
        this->fsm_.getStatus() == PathFollowingFSMStatus::CHANGE_DIRECTION)
      {
        command.longitudinalSpeed = 0;
        command.angularSpeed = 0;
      }
    }

    fsm_.updateOdometry(command, odometryMeasure);
    return command;
  }

  virtual void reset() = 0;

  virtual CommandType computeCommand(
    const PathFollowingSetPoint & setPoint,
    const CommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist) = 0;

protected:
  FSM fsm_;
  std::shared_ptr<Logger> logger_;
};

template<typename LateralControl, typename LongitudinalControl>
class PathFollowingWithoutSlidingObserver
  : public PathFollowingBase<typename LateralControl::Command>
{
public:
  using CommandType = typename LateralControl::Command;
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using CommandLimits = typename PathFollowingTraits<CommandType>::Limits;

public:
  PathFollowingWithoutSlidingObserver(
    std::shared_ptr<LateralControl> lateralControl,
    std::shared_ptr<LongitudinalControl> longitudinalControl)
  : lateralControl_(lateralControl),
    longitudinalControl_(longitudinalControl)
  {
  }

  CommandType computeCommand(
    const PathFollowingSetPoint & setPoint,
    const CommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futureCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)
  {
    auto command = this->lateralControl_->computeCommand(
      setPoint, commandLimits, frenetPose, pathPosture, futureCurvature, odometryMeasure);

    command.longitudinalSpeed = this->longitudinalControl_->computeLinearSpeed(
      setPoint, frenetPose, pathPosture, odometryMeasure, filteredTwist);

    if (this->logger_ != nullptr) {
      this->logger_->addEntry("fsm_status", static_cast<int>(this->fsm_.getStatus()));
      log(*this->logger_, setPoint);
      log(*this->logger_, frenetPose);
      log(*this->logger_, pathPosture);
      this->logger_->addEntry("path_future_curvature", futureCurvature);
      log(*this->logger_, odometryMeasure);
      log(*this->logger_, filteredTwist);
      lateralControl_->log(*this->logger_);
      longitudinalControl_->log(*this->logger_);
      log(*this->logger_, command);
    }

    return command;
  }

  void reset() override
  {
    this->fsm_.reset();
    this->lateralControl_->reset();
    this->longitudinalControl_->reset();
  }

  std::shared_ptr<LateralControl> lateralControl_;
  std::shared_ptr<LongitudinalControl> longitudinalControl_;
};

template<typename LateralControl, typename LongitudinalControl, typename SlidingObserver>
class PathFollowingWithSlidingObserver
  : public PathFollowingBase<typename LateralControl::Command>
{
public:
  using CommandType = typename LateralControl::Command;
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using CommandLimits = typename PathFollowingTraits<CommandType>::Limits;
  using LateralControlSlidings = typename LateralControl::Slidings;
  using ObserverSlidings = typename SlidingObserver::Slidings;

public:
  PathFollowingWithSlidingObserver(
    std::shared_ptr<LateralControl> lateralControl,
    std::shared_ptr<LongitudinalControl> longitudinalControl,
    std::shared_ptr<SlidingObserver> slidingObserver)
  : lateralControl_(lateralControl),
    longitudinalControl_(longitudinalControl),
    slidingObserver_(slidingObserver)
  {
  }

  CommandType computeCommand(
    const PathFollowingSetPoint & setPoint,
    const CommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futureCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)
  {
    LateralControlSlidings slidings;
    if constexpr (std::is_same_v<LateralControlSlidings, ObserverSlidings>) {
      slidings = slidingObserver_->computeSlidings(
        frenetPose, pathPosture, odometryMeasure, filteredTwist);
    } else {
      // convert slidings
    }

    auto command = this->lateralControl_->computeCommand(
      setPoint, commandLimits, frenetPose, pathPosture,
      futureCurvature, odometryMeasure, slidings);

    command.longitudinalSpeed = this->longitudinalControl_->computeLinearSpeed(
      setPoint, frenetPose, pathPosture, odometryMeasure, filteredTwist);

    if (this->logger_ != nullptr) {
      this->logger_->addEntry("fsm_status", static_cast<int>(this->fsm_.getStatus()));
      log(*this->logger_, setPoint);
      log(*this->logger_, frenetPose);
      log(*this->logger_, pathPosture);
      this->logger_->addEntry("path_future_curvature", futureCurvature);
      log(*this->logger_, odometryMeasure);
      log(*this->logger_, filteredTwist);
      lateralControl_->log(*this->logger_);
      longitudinalControl_->log(*this->logger_);
      slidingObserver_->log(*this->logger_);
      log(*this->logger_, command);
    }

    return command;
  }

  void reset() override
  {
    this->fsm_.reset();
    this->lateralControl_->reset();
    this->longitudinalControl_->reset();
    this->slidingObserver_->reset();
  }

private:
  std::shared_ptr<LateralControl> lateralControl_;
  std::shared_ptr<LongitudinalControl> longitudinalControl_;
  std::shared_ptr<SlidingObserver> slidingObserver_;
};


class OneAxleSteeringEquivalence
  : public PathFollowingBase<SkidSteeringCommand>
{

public:
  OneAxleSteeringEquivalence(
    std::unique_ptr<PathFollowingBase<OneAxleSteeringCommand>> pathFollowing)
  : pathFollowing_(std::move(pathFollowing))
  {
  }

  void registerLogger(std::shared_ptr<Logger> logger)
  {
    logger_ = logger;
    pathFollowing_->registerLogger(logger);
  }

  SkidSteeringCommand computeCommand(
    const PathFollowingSetPoint & setPoint,
    const SkidSteeringCommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futureCurvature,
    const SkidSteeringMeasure & odometryMeasure,
    const Twist2D & filteredTwist) override
  {
    OneAxleSteeringCommand command = pathFollowing_->computeCommand(
      setPoint, toOneAxleSteeringCommandLimits(commandLimits),
      frenetPose, pathPosture, futureCurvature,
      toOneAxleSteeringMeasure(odometryMeasure, wheelbase_), filteredTwist);

    if (logger_ != nullptr) {
      log(*this->logger_, odometryMeasure);
      log(*this->logger_, toSkidSteeringCommand(command, wheelbase_));
    }

    return toSkidSteeringCommand(command, wheelbase_);
  }

  void reset() override
  {
    pathFollowing_->reset();
  }

  const double wheelbase_ = 1.2;
  std::unique_ptr<PathFollowingBase<OneAxleSteeringCommand>> pathFollowing_;
};


}   // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_
