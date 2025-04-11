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

#ifndef ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__DESBOS_GENERIC_PREDICTIVE_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__DESBOS_GENERIC_PREDICTIVE_HPP_

// romea
#include <romea_core_control/command/FollowTrajectoryDesbosGenericPredictive.hpp>

// local
#include "romea_core_path_following/lateral_control/base.hpp"
#include "romea_core_path_following/sliding_observer/skid/base.hpp"

namespace romea::core::path_following
{

template<typename CommandType>
class LateralControlDesbosGenericPredictive
{
};

template<>
class LateralControlDesbosGenericPredictive<SkidSteeringCommand>
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
    double alpha;
    double a0;
    double a1;
    double b1;
    double b2;
    int horizon;
    bool adaptive_gains;
    bool lmpc;
    int model_order;
  };

public:
  LateralControlDesbosGenericPredictive(
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

  void set_lmpc(bool lmpc) { lateral_control_.set_lmpc(lmpc); }

  SharedVariable<Gains> gains;

private:
  core::FollowTrajectoryDesbosGenericPredictive lateral_control_;
  double sampling_period_;
  double target_course_ = 0;
};

}  // namespace romea::core::path_following

#endif
