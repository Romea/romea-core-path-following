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

#ifndef ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__FRONT_REAR_DECOUPLED_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__FRONT_REAR_DECOUPLED_HPP_

// romea
#include <romea_core_control/command/FollowTrajectoryFrontRearDecoupled.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>

#include "romea_core_path_following/lateral_control/base.hpp"
#include "romea_core_path_following/sliding_observer/extended/base.hpp"

namespace romea::core::path_following
{

template<typename CommandType>
class LateralControlFrontRearDecoupled;

template<>
class LateralControlFrontRearDecoupled<TwoAxleSteeringCommand>
: public LateralControlBase<TwoAxleSteeringCommand, ExtendedSlidings>
{
public:
  using LateralControl = FollowTrajectoryFrontRearDecoupled;

  struct Gains
  {
    double front_kp;
    double rear_kp;
  };

  struct Parameters
  {
    Gains gains;
  };

public:
  LateralControlFrontRearDecoupled(
    const double & samplePeriod,
    const double & wheelbase,
    const MobileBaseInertia & inertia,
    const Parameters & parameters);

  TwoAxleSteeringCommand compute_command(
    const SetPoint & setPoint,
    const CommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const ExtendedSlidings & slidings = {}) override;

public:
  SharedVariable<Gains> gains;

private:
  LateralControl lateralControl_;
};

}  // namespace romea::core::path_following

#endif
