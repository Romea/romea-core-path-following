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


#ifndef ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGSETPOINT_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGSETPOINT_HPP_

// std
#include <limits>

// romea
#include <romea_core_path/PathMatchedPoint2D.hpp>

namespace romea
{
namespace core
{

struct PathFollowingSetPoint
{
  double linearSpeed = std::numeric_limits<double>::quiet_NaN();
  double lateralDeviation = 0;
  double courseDeviation = 0;
};

PathFollowingSetPoint evaluateSetPoint(
  const PathFollowingSetPoint & desiredSetPoint,
  const PathMatchedPoint2D & pathMatchedPoint);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGSETPOINT_HPP_
