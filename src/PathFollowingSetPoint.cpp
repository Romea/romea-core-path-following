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


// romea
#include "romea_core_common/math/Algorithm.hpp"
#include "romea_core_path_following/PathFollowingSetPoint.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathFollowingSetPoint evaluateSetPoint(
  const PathFollowingSetPoint & desiredSetPoint,
  const PathMatchedPoint2D & pathMatchedPoint)
{
  PathFollowingSetPoint setPoint;

  if (!std::isfinite(desiredSetPoint.linearSpeed)) {
    if (std::isfinite(pathMatchedPoint.desiredSpeed)) {
      setPoint.linearSpeed = pathMatchedPoint.desiredSpeed;
    } else {
      setPoint.linearSpeed = 0;
    }
  } else {
    if (std::isfinite(pathMatchedPoint.desiredSpeed)) {
      setPoint.linearSpeed = sign(pathMatchedPoint.desiredSpeed) * desiredSetPoint.linearSpeed;
    } else {
      setPoint.linearSpeed = desiredSetPoint.linearSpeed;
    }
  }

  setPoint.lateralDeviation = desiredSetPoint.lateralDeviation;
  setPoint.courseDeviation = desiredSetPoint.courseDeviation;
  return setPoint;
}

}  // namespace core
}  // namespace romea
