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
#include "romea_core_path_following/lateral_control/front_rear_decoupled.hpp"
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>

#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

LateralControlFrontRearDecoupled<TwoAxleSteeringCommand>::LateralControlFrontRearDecoupled(
  const double & /*samplePeriod*/,
  const double & wheelbase,
  const MobileBaseInertia & /*inertia*/,
  const Parameters & parameters)
: gains(parameters.gains),
  lateralControl_(wheelbase, {parameters.gains.front_kp, parameters.gains.rear_kp})
{
}

TwoAxleSteeringCommand LateralControlFrontRearDecoupled<TwoAxleSteeringCommand>::compute_command(
  const SetPoint & setPoint,
  const CommandLimits & commandLimits,
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const double & futurePathCurvature,
  const OdometryMeasure & odometryMeasure,
  const ExtendedSlidings & slidings)
{
  auto steeringAngles = lateralControl_.computeSteeringAngles(
    frenetPose.lateralDeviation,
    frenetPose.courseDeviation,
    pathPosture.curvature,
    futurePathCurvature,
    odometryMeasure.longitudinalSpeed,
    get_front_steering_angle(odometryMeasure),
    get_rear_steering_angle(odometryMeasure),
    slidings.frontSteeringAngle,
    slidings.rearSteeringAngle,
    get_maximal_front_steering_angle(commandLimits),
    get_maximal_rear_steering_angle(commandLimits),
    setPoint.lateral_deviation,
    setPoint.course_deviation);

  return {setPoint.linear_speed, steeringAngles.front, steeringAngles.rear};
}

}  // namespace romea::core
