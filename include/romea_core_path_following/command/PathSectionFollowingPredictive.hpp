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


#ifndef ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONFOLLOWINGPREDICTIVE_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONFOLLOWINGPREDICTIVE_HPP_

// std
#include <memory>

// romea
#include "romea_core_control/command/FollowTrajectoryPredictiveSliding.hpp"
#include "romea_core_path_following/command/PathSectionFollowingBase.hpp"

namespace romea
{
namespace core
{

using PathSectionFollowingPredictiveParameters = FollowTrajectoryPredictiveSliding::Parameters;

template<class CommandType>
class PathSectionFollowingPredictive : public PathSectionFollowingBase<CommandType>
{
public:
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using CommandLimits = typename PathFollowingTraits<CommandType>::Limits;
  using SlidingObserver = PathFollowingSlidingObserverBase<CommandType>;
  using LongitudinalControl = PathSectionFollowingLongitudinalControl<CommandType>;
  using LongitudinalControlParameters = typename LongitudinalControl::Parameters;
  using LateralControl = FollowTrajectoryPredictiveSliding;
  using LateralControlParameters = LateralControl::Parameters;

public:
  PathSectionFollowingPredictive(
    const double & wheelbase,
    const MobileBaseInertia & inertia,
    const LongitudinalControlParameters & longitudinalControlParameters,
    const LateralControlParameters & lateralControlParameters,
    const CommandLimits & commandLimits);

private:
  SteeringAngles computeSteeringAngles_(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist) override;

private:
  LateralControl lateralControl_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONFOLLOWINGPREDICTIVE_HPP_
