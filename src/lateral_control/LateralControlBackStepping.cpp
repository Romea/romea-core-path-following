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
#include "romea_core_path_following/PathFollowingUtils.hpp"
#include "romea_core_path_following/lateral_control/LateralControlBackStepping.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
PathFollowingLateralControlBackStepping<SkidSteeringCommand>::
PathFollowingLateralControlBackStepping(
  const double & samplePeriod,
  const double & wheelbase,
  const MobileBaseInertia & /*inertia*/,
  const Parameters & parameters)
: omegaD_(0),
  gains_(parameters.gains),
  lateralControl_(samplePeriod, wheelbase,
    {parameters.gains.KP,
      parameters.gains.KD,
      parameters.gains.KI,
      parameters.gains.IClamp,
      0,
      parameters.maximalOmegaD})
{
}

//-----------------------------------------------------------------------------
SkidSteeringCommand
PathFollowingLateralControlBackStepping<SkidSteeringCommand>::computeCommand(
  const PathFollowingSetPoint & setPoint,
  const CommandLimits & commandLimits,
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const double & /*futurePathCurvature*/,
  const OdometryMeasure & odometryMeasure,
  const WildcardSlidings & /*slidings*/)
{
  double angular_speed = lateralControl_.computeAngularSpeed(
    frenetPose.lateralDeviation,
    frenetPose.courseDeviation,
    pathPosture.curvature,
    odometryMeasure.longitudinalSpeed,
    getMaximalAngularSpeed(commandLimits),
    setPoint.lateralDeviation,
    omegaD_);

  return SkidSteeringCommand(setPoint.linearSpeed, angular_speed);
}


//-----------------------------------------------------------------------------
void PathFollowingLateralControlBackStepping<SkidSteeringCommand>::updateGains(const Gains & gains)
{
  gains_.store(gains);
}

//-----------------------------------------------------------------------------
void PathFollowingLateralControlBackStepping<SkidSteeringCommand>::log(
  SimpleFileLogger & logger)
{
  logger.addEntry("omega_d", omegaD_);
}

void PathFollowingLateralControlBackStepping<SkidSteeringCommand>::reset()
{
  lateralControl_.reset();
}

// //-----------------------------------------------------------------------------
// OneAxleSteeringCommand
// PathFollowingLateralControlClassic<OneAxleSteeringCommand>::computeCommand(
//   const PathFollowingSetPoint & setPoint,
//   const CommandLimits & commandLimits,
//   const PathFrenetPose2D & frenetPose,
//   const PathPosture2D & pathPosture,
//   const double & /*futurePathCurvature*/,
//   const OdometryMeasure & odometryMeasure,
//   const AxleSteeringSlidings & slidings)
// {
//   lateralControl_.setFrontKP(gains_.load().frontKD);

//   auto steeringAngles = lateralControl_.computeSteeringAngles(
//     frenetPose.lateralDeviation,
//     frenetPose.courseDeviation,
//     pathPosture.curvature,
//     slidings.frontSteeringAngle,
//     slidings.rearSteeringAngle,
//     getRearSteeringAngle(odometryMeasure),
//     getMaximalFrontSteeringAngle(commandLimits),
//     getMaximalRearSteeringAngle(commandLimits),
//     setPoint.lateralDeviation,
//     setPoint.courseDeviation);

//   return OneAxleSteeringCommand(setPoint.linearSpeed, steeringAngles.front);
// }


// //-----------------------------------------------------------------------------
// void PathFollowingLateralControlClassic<OneAxleSteeringCommand>::updateGains(const Gains & gains)
// {
//   gains_.store(gains);
// }

// //-----------------------------------------------------------------------------
// void PathFollowingLateralControlClassic<OneAxleSteeringCommand>::
// log(SimpleFileLogger & /*logger*/)
// {
// }

// void PathFollowingLateralControlClassic<OneAxleSteeringCommand>::reset()
// {
// }


// //-----------------------------------------------------------------------------
// PathFollowingLateralControlClassic<TwoAxleSteeringCommand>::PathFollowingLateralControlClassic(
//   const double & wheelbase,
//   const MobileBaseInertia & /*inertia*/,
//   const Parameters & parameters)
// : gains_(parameters.gains),
//   lateralControl_(wheelbase, {parameters.gains.frontKD, parameters.gains.rearKD})
// {
// }

// //-----------------------------------------------------------------------------
// TwoAxleSteeringCommand
// PathFollowingLateralControlClassic<TwoAxleSteeringCommand>::computeCommand(
//   const PathFollowingSetPoint & setPoint,
//   const CommandLimits & commandLimits,
//   const PathFrenetPose2D & frenetPose,
//   const PathPosture2D & pathPosture,
//   const double & /*futurePathCurvature*/,
//   const OdometryMeasure & odometryMeasure,
//   const AxleSteeringSlidings & slidings)
// {
//   auto gains = gains_.load();
//   lateralControl_.setFrontKP(gains.frontKD);

//   auto steeringAngles = lateralControl_.computeSteeringAngles(
//     frenetPose.lateralDeviation,
//     frenetPose.courseDeviation,
//     pathPosture.curvature,
//     slidings.frontSteeringAngle,
//     slidings.rearSteeringAngle,
//     getRearSteeringAngle(odometryMeasure),
//     getMaximalFrontSteeringAngle(commandLimits),
//     getMaximalRearSteeringAngle(commandLimits),
//     setPoint.lateralDeviation,
//     setPoint.courseDeviation);

//  return TwoAxleSteeringCommand(setPoint.linearSpeed,steeringAngles.front, steeringAngles.rear);
// }

// //-----------------------------------------------------------------------------
// void PathFollowingLateralControlClassic<TwoAxleSteeringCommand>::
// updateGains(const Gains & gains)
// {
//   gains_.store(gains);
// }

// //-----------------------------------------------------------------------------
// void PathFollowingLateralControlClassic<TwoAxleSteeringCommand>::
// log(SimpleFileLogger & /*logger*/)
// {
// }

// //-----------------------------------------------------------------------------
// void PathFollowingLateralControlClassic<TwoAxleSteeringCommand>::reset()
// {
// }


}  // namespace core
}  // namespace romea
