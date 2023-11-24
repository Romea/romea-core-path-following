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


#ifndef ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONLINEARSPEEDCONTROL_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONLINEARSPEEDCONTROL_HPP_

// std
#include <memory>

// romea
#include <romea_core_common/log/SimpleFileLogger.hpp>
#include <romea_core_common/geometry/Twist2D.hpp>
#include <romea_core_path/PathFrenetPose2D.hpp>
#include <romea_core_path/PathPosture2D.hpp>
#include "romea_core_path_following/PathFollowingLogs.hpp"
#include "romea_core_path_following/PathFollowingTraits.hpp"

namespace romea
{
namespace core
{

struct PathSectionFollowingLongitudinalControlParameters
{
  double minimalLinearSpeed;
};

template<class CommandType>
class PathSectionFollowingLongitudinalControl
{
public:
  using Parameters = PathSectionFollowingLongitudinalControlParameters;
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;

public:
  explicit PathSectionFollowingLongitudinalControl(const Parameters & parameters);

  double computeLinearSpeed(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist);

  void log(SimpleFileLogger & logger);

  void reset();

protected:
  double minimalLinearSpeed_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__COMMAND__PATHSECTIONLINEARSPEEDCONTROL_HPP_
