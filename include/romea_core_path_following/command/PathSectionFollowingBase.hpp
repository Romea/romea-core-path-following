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


#ifndef ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONFOLLOWINGBASE_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONFOLLOWINGBASE_HPP_

// std
#include <memory>

// romea
#include <romea_core_common/log/SimpleFileLogger.hpp>
#include <romea_core_common/geometry/Twist2D.hpp>
#include <romea_core_control/FrontRearData.hpp>
#include <romea_core_path/PathFrenetPose2D.hpp>
#include <romea_core_path/PathPosture2D.hpp>
#include "romea_core_path_following/PathFollowingLogs.hpp"
#include "romea_core_path_following/PathFollowingTraits.hpp"
#include "romea_core_path_following/observer/PathFollowingSlidingObserverBase.hpp"
#include "romea_core_path_following/command/PathSectionFollowingLongitudinalControl.hpp"


namespace romea
{
namespace core
{

using SteeringAngles = FrontRearData;

template<class CommandType>
class PathSectionFollowingBase
{
public:
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using CommandLimits = typename PathFollowingTraits<CommandType>::Limits;
  using LongitudinalControl = PathSectionFollowingLongitudinalControl<CommandType>;
  using LongitudinalControlParameters = typename LongitudinalControl::Parameters;
  using SlidingObserver = PathFollowingSlidingObserverBase<CommandType>;

public:
  PathSectionFollowingBase(
    const LongitudinalControlParameters & longitudinalControlParameters,
    const CommandLimits & commandLimits);

  virtual ~PathSectionFollowingBase() = default;

  virtual void registerSlidingObserver(std::unique_ptr<SlidingObserver> sliding_observer);

  virtual CommandType computeCommand(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist);

  virtual void log(SimpleFileLogger & logger);

  virtual void reset();

protected:
  CommandType makeCommand_(
    const double & linearSpeedCommand,
    const SteeringAngles & steeringAnglesCommand);

  virtual double computeLinearSpeed_(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist);

  virtual SlidingAngles compute_sliding_angles_(
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist);

  virtual SteeringAngles computeSteeringAngles_(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist) = 0;

  void log_sliding_observer_data_(SimpleFileLogger & logger);

  void reset_sliding_observer_();

protected:
  CommandLimits commandLimits_;
  LongitudinalControl longitudinalControl_;
  std::unique_ptr<SlidingObserver> sliding_observer_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONFOLLOWINGBASE_HPP_
