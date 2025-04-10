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


#ifndef ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__DESBOS_GENERIC_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__DESBOS_GENERIC_HPP_

// romea
#include <romea_core_control/command/FollowTrajectoryDesbosGeneric.hpp>

// local
#include "romea_core_path_following/lateral_control/base.hpp"
#include "romea_core_path_following/sliding_observer/skid/base.hpp"

namespace romea::core::path_following
{

template<typename CommandType>
class LateralControlDesbosGeneric
{
};

template<>
class LateralControlDesbosGeneric<SkidSteeringCommand>
  : public LateralControlBase<SkidSteeringCommand, SkidSlidingParameters>
{
public:
  struct Gains
  {
    double kp;
    double kd;
    double ks;
  };

  struct Parameters
  {
    Gains gains;
    bool adaptive_gains;
  };

public:
  LateralControlDesbosGeneric(
    double sample_period,
    double wheelbase,
    const MobileBaseInertia & inertia,
    const Parameters & parameters);

  SkidSteeringCommand compute_command(
    const SetPoint & set_point,
    const CommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_path_curvature,
    const OdometryMeasure & odometry_measure,
    const SkidSlidingParameters & slidings = {}) override;

  void log(SimpleFileLogger & logger) override;

  SharedVariable<Gains> gains;

private:
  core::FollowTrajectoryDesbosGeneric lateral_control_;
  double target_course_ = 0;
};


} // namespace romea::core::path_following



#endif  // ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__SKID_SLIDING_HPP_
