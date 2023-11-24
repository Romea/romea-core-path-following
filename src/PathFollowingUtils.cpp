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

#include "romea_core_path_following/PathFollowingUtils.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
double getFrontSteeringAngle(const TwoAxleSteeringCommand & command)
{
  return command.frontSteeringAngle;
}

//-----------------------------------------------------------------------------
double getFrontSteeringAngle(const OneAxleSteeringCommand & command)
{
  return command.steeringAngle;
}

//-----------------------------------------------------------------------------
double getRearSteeringAngle(const TwoAxleSteeringCommand & command)
{
  return command.rearSteeringAngle;
}

//-----------------------------------------------------------------------------
double getRearSteeringAngle(const OneAxleSteeringCommand & /*command*/)
{
  return 0;
}

//-----------------------------------------------------------------------------
double getMaximalFrontSteeringAngle(const TwoAxleSteeringCommandLimits & limits)
{
  return limits.frontSteeringAngle.upper();
}

//-----------------------------------------------------------------------------
double getMaximalFrontSteeringAngle(const OneAxleSteeringCommandLimits & limits)
{
  return limits.steeringAngle.upper();
}

//-----------------------------------------------------------------------------
double getMaximalRearSteeringAngle(const TwoAxleSteeringCommandLimits & limits)
{
  return limits.rearSteeringAngle.upper();
}

//-----------------------------------------------------------------------------
double getMaximalRearSteeringAngle(const OneAxleSteeringCommandLimits & /*limits*/)
{
  return 0;
}

}  // namespace core
}  // namespace romea
