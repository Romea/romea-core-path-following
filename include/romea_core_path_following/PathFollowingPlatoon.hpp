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

#ifndef ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGPLATOON_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGPLATOON_HPP_


// std
#include <memory>
#include <utility>
#include <vector>

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/geometry/Twist2D.hpp"
#include "romea_core_common/log/SimpleFileLogger.hpp"
#include "romea_core_common/concurrency/SharedVariable.hpp"
#include "romea_core_path/PathMatchedPoint2D.hpp"

#include "romea_core_control/command/KeepInterdistance.hpp"


namespace romea
{
namespace core
{

class PathFollowingPlatoon
{
public:
  using Logger = core::SimpleFileLogger;
  using PathMatchedPoints2D = std::vector<PathMatchedPoint2D>;

  struct PathMatchingInfo
  {
    Duration stamp;
    PathMatchedPoints2D matchedPoints;
    Twist2D twist;
  };

  PathFollowingPlatoon(
    const double & samplingPeriod,
    const double & desiredInterDistance,
    const double & maximalLinearSpeed,
    const double & minimalLinearAcceleration,
    const double & maximalLinearAcceleration);

  virtual void registerLogger(std::shared_ptr<Logger> logger);

  void setPreviousVehicleInfo(const PathMatchingInfo & info);
  void setCurrentVehicleInfo(const PathMatchingInfo & info);
  void setNextVehicleInfo(const PathMatchingInfo & info);

  std::optional<double> computeLinearSpeedCommand(const Duration & stamp);

private:
  double computeLinearSpeedCommand_(
    const PathMatchingInfo & previousVehicleInfo_,
    const PathMatchingInfo & currentVehicleInfo,
    const PathMatchingInfo & nextVehicleInfo);

  void clampLinearSpeedCommand_();

  bool isVehicleInfoAvailable_(const PathMatchingInfo & info, const Duration & stamp);

private:
  SharedVariable<PathMatchingInfo> previousVehicleInfo_;
  SharedVariable<PathMatchingInfo> currentVehicleInfo_;
  SharedVariable<PathMatchingInfo> nextVehicleInfo_;

  double samplingPeriod_;
  double desiredInterDistance_;
  double maximalLinearSpeed_;
  double minimalLinearAcceleration_;
  double maximalLinearAcceleration_;
  std::shared_ptr<KeepInterdistance> keepInterdistance_;

  double previousLinearSpeedCommand_;
  double currentLinearSpeedCommand_;

  std::shared_ptr<Logger> logger_;
};


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGPLATOON_HPP_
