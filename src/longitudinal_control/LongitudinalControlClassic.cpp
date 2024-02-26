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
#include "romea_core_path_following/longitudinal_control/LongitudinalControlClassic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathFollowingLongitudinalControlClassic<OneAxleSteeringCommand>::
PathFollowingLongitudinalControlClassic(
  const Parameters & parameters)
: minimalLinearSpeed_()
{
}

//-----------------------------------------------------------------------------
double PathFollowingLongitudinalControlClassic<OneAxleSteeringCommand>::computeLinearSpeed(
  const PathFollowingSetPoint & setPoint,
  const PathFrenetPose2D & /*frenetPose*/,
  const PathPosture2D & /*pathPosture*/,
  const OdometryMeasure & /*odometryMeasure*/,
  const Twist2D & /*filteredTwist*/)
{
  return setPoint.linearSpeed;
  // double desiredLinearSpeed = setPoint.linearSpeed;
  // double desiredLateralDeviation = setPoint.lateralDeviation;

  // const double & EcartLat = frenetPose.lateralDeviation;
  // const double & EcartAng = frenetPose.courseDeviation;
  // const double & steering = getFrontSteeringAngle(odometryMeasure);

  // double flag = 1;
  // if (fabs(steering) < 10 / 180. * M_PI) {flag = 0;}

  // double linearSpeedCommand =
  //   fabs(desiredLinearSpeed) -
  //   2 * (EcartLat - desiredLateralDeviation) * (EcartLat - desiredLateralDeviation) -
  //   10 * EcartAng * EcartAng - 1 * flag * fabs(steering);

  // if (linearSpeedCommand < minimalLinearSpeed_) {
  //   linearSpeedCommand = minimalLinearSpeed_;
  // }

  // return std::copysign(linearSpeedCommand, desiredLinearSpeed);
}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlClassic<OneAxleSteeringCommand>::log(
  SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlClassic<OneAxleSteeringCommand>::reset()
{
}


//-----------------------------------------------------------------------------
PathFollowingLongitudinalControlClassic<SkidSteeringCommand>::
PathFollowingLongitudinalControlClassic(
  const Parameters & parameters)
{
}

//-----------------------------------------------------------------------------
double PathFollowingLongitudinalControlClassic<SkidSteeringCommand>::computeLinearSpeed(
  const PathFollowingSetPoint & setPoint,
  const PathFrenetPose2D & /*frenetPose*/,
  const PathPosture2D & /*pathPosture*/,
  const OdometryMeasure & /*odometryMeasure*/,
  const Twist2D & /*filteredTwist*/)
{
  return setPoint.linearSpeed;
}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlClassic<SkidSteeringCommand>::log(
  SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlClassic<SkidSteeringCommand>::reset()
{
}


//-----------------------------------------------------------------------------
PathFollowingLongitudinalControlClassic<TwoAxleSteeringCommand>::
PathFollowingLongitudinalControlClassic(
  const Parameters & parameters)
{
}

//-----------------------------------------------------------------------------
double PathFollowingLongitudinalControlClassic<TwoAxleSteeringCommand>::computeLinearSpeed(
  const PathFollowingSetPoint & setPoint,
  const PathFrenetPose2D & /*frenetPose*/,
  const PathPosture2D & /*pathPosture*/,
  const OdometryMeasure & /*odometryMeasure*/,
  const Twist2D & /*filteredTwist*/)
{
  return setPoint.linearSpeed;
}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlClassic<TwoAxleSteeringCommand>::log(
  SimpleFileLogger & /*logger*/)
{
}

//-----------------------------------------------------------------------------
void PathFollowingLongitudinalControlClassic<TwoAxleSteeringCommand>::reset()
{
}

}  // namespace core
}  // namespace romea
