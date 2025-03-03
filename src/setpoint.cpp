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
#include "romea_core_path_following/setpoint.hpp"

namespace romea
{
namespace core
{
namespace path_following
{

//-----------------------------------------------------------------------------
SetPoint evaluate_setpoint(
  const SetPoint & desired_setpoint,
  const PathMatchedPoint2D & path_matched_point)
{
  SetPoint setPoint;

  if (!std::isfinite(desired_setpoint.linear_speed)) {
    if (std::isfinite(path_matched_point.desiredSpeed)) {
      setPoint.linear_speed = path_matched_point.desiredSpeed;
    } else {
      setPoint.linear_speed = 0;
    }
  } else {
    if (std::isfinite(path_matched_point.desiredSpeed)) {
      setPoint.linear_speed = sign(path_matched_point.desiredSpeed) * desired_setpoint.linear_speed;
    } else {
      setPoint.linear_speed = desired_setpoint.linear_speed;
    }
  }

  setPoint.lateral_deviation = desired_setpoint.lateral_deviation;
  setPoint.course_deviation = desired_setpoint.course_deviation;
  return setPoint;
}

}  // namespace path_following
}  // namespace core
}  // namespace romea
