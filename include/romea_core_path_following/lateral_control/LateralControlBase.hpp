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


#ifndef ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__PATHFOLLOWINGLATERALCONTROLBASE_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__PATHFOLLOWINGLATERALCONTROLBASE_HPP_

// std
#include <memory>

// romea
#include "romea_core_common/concurrency/SharedVariable.hpp"
#include "romea_core_common/geometry/Twist2D.hpp"
#include "romea_core_common/log/SimpleFileLogger.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_path/PathFrenetPose2D.hpp"
#include "romea_core_path/PathPosture2D.hpp"
#include "romea_core_path_following/PathFollowingLogs.hpp"
#include "romea_core_path_following/PathFollowingTraits.hpp"


namespace romea
{
namespace core
{

template<typename CommandType, typename SlidingsType>
class PathFollowingLateralControlBase
{
public:
  using Command = CommandType;
  using Slidings = SlidingsType;
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using CommandLimits = typename PathFollowingTraits<CommandType>::Limits;

public:
  PathFollowingLateralControlBase() {}

  virtual ~PathFollowingLateralControlBase() = default;

  virtual CommandType computeCommand(
    const PathFollowingSetPoint & setPoint,
    const CommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const SlidingsType & slidings = {}) = 0;

  virtual void log(SimpleFileLogger & logger) = 0;

  virtual void reset() = 0;

};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__PATHFOLLOWINGLATERALCONTROLBASE_HPP_
