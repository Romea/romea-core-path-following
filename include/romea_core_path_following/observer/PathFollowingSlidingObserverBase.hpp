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


#ifndef ROMEA_CORE_PATH_FOLLOWING__OBSERVER__PATHFOLLOWINGSLIDINGOBSERVERBASE_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__OBSERVER__PATHFOLLOWINGSLIDINGOBSERVERBASE_HPP_

// romea
#include <romea_core_common/log/SimpleFileLogger.hpp>
#include <romea_core_common/geometry/Twist2D.hpp>
#include <romea_core_control/FrontRearData.hpp>
#include <romea_core_mobile_base/info/MobileBaseInertia.hpp>
#include <romea_core_path/PathMatchedPoint2D.hpp>
#include <romea_core_path_following/PathFollowingTraits.hpp>

namespace romea
{
namespace core
{

using SlidingAngles = FrontRearData;

template<class CommandType>
class PathFollowingSlidingObserverBase
{
public:
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;

public:
  PathFollowingSlidingObserverBase() {}

  virtual ~PathFollowingSlidingObserverBase() = default;

  virtual SlidingAngles computeSlidingAngles(
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist) = 0;

  virtual void log(SimpleFileLogger & logger) = 0;

  virtual void reset() = 0;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__OBSERVER__PATHFOLLOWINGSLIDINGOBSERVERBASE_HPP_
