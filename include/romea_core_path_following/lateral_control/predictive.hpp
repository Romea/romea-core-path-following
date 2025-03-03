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


#ifndef ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__PREDICTIVE_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__PREDICTIVE_HPP_

// std
#include <memory>

// romea
#include "romea_core_control/command/FollowTrajectoryPredictiveSliding.hpp"
#include "romea_core_path_following/lateral_control/base.hpp"
#include "romea_core_path_following/sliding_observer/extended/base.hpp"

namespace romea
{
namespace core
{
namespace path_following
{

template<typename OneAxleSteering>
class LateralControlPredictive
{
};

template<>
class LateralControlPredictive<OneAxleSteeringCommand>
  : public LateralControlBase<OneAxleSteeringCommand, ExtendedSlidings>
{
public:
  using LateralControl = FollowTrajectoryPredictiveSliding;

  struct Gains
  {
    double front_kd;
  };
  struct Parameters
  {
    Gains gains;
    int horizon;
    double a0;
    double a1;
    double b1;
    double b2;
  };

public:
  LateralControlPredictive(
    const double & samplePeriod,
    const double & wheelbase,
    const MobileBaseInertia & inertia,
    const Parameters & parameter
  );

  OneAxleSteeringCommand compute_command(
    const SetPoint & set_point,
    const CommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_path_curvature,
    const OdometryMeasure & odometry_measure,
    const ExtendedSlidings & slidings = {}) override;

  SharedVariable<Gains> gains;

private:
  LateralControl lateralControl_;
};


template<>
class LateralControlPredictive<TwoAxleSteeringCommand>
  : public LateralControlBase<TwoAxleSteeringCommand, ExtendedSlidings>
{
public:
  using LateralControl = FollowTrajectoryPredictiveSliding;

  struct Gains
  {
    double front_kd;
    double rear_kd;
  };
  struct Parameters
  {
    Gains gains;
    int horizon;
    double a0;
    double a1;
    double b1;
    double b2;
  };

public:
  LateralControlPredictive(
    const double & sample_period,
    const double & wheelbase,
    const MobileBaseInertia & inertia,
    const Parameters & parameters
  );

  TwoAxleSteeringCommand compute_command(
    const SetPoint & set_point,
    const CommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_path_curvature,
    const OdometryMeasure & odometry_measure,
    const ExtendedSlidings & slidings = {}) override;

  SharedVariable<Gains> gains;

private:
  LateralControl lateralControl_;
};


}  // namespace path_following
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__PREDICTIVE_HPP_
