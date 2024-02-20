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
#include \
  "romea_core_path_following/longitudinal_control/PathFollowingLongitudinalControlSkidSteering.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathFollowingLongitudinalControlSkidSteering::PathFollowingLongitudinalControlSkidSteering(
  const Parameters & parameters)
{
}

//-----------------------------------------------------------------------------
double PathFollowingLongitudinalControlSkidSteering::computeLinearSpeed(
  const PathFollowingSetPoint & setPoint,
  const PathFrenetPose2D & /*frenetPose*/,
  const PathPosture2D & /*pathPosture*/,
  const OdometryMeasure & /*odometryMeasure*/,
  const Twist2D & /*filteredTwist*/)
{
  return setPoint.linearSpeed;
}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlSkidSteering::log(SimpleFileLogger & /*logger*/)
{

}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlSkidSteering::reset()
{

}

}  // namespace core
}  // namespace romea
