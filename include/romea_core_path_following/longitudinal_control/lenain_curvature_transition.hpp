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

#ifndef ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__LENAIN_CURVATURE_TRANSITION_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__LENAIN_CURVATURE_TRANSITION_HPP_

// romea
#include "romea_core_path_following/longitudinal_control/base.hpp"

namespace romea::core::path_following
{

template<class CommandType>
class LongitudinalControlLenainCurvatureTransition : public LongitudinalControlBase<CommandType>
{
public:
  struct Parameters
  {
    double minimal_linear_speed;
  };
  using OdometryMeasure = typename LongitudinalControlBase<CommandType>::OdometryMeasure;

public:
  explicit LongitudinalControlLenainCurvatureTransition(const Parameters & parameters);

  double compute_linear_speed(
    const SetPoint & setpoint,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    double future_curvature,
    const OdometryMeasure & odometry_measure,
    const Twist2D & filtered_twist) override;

  void log(SimpleFileLogger & logger) override;

  void reset() override;

protected:
  double minimal_linear_speed_;
};

}  // namespace romea::core::path_following

#endif  // ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__LENAIN_CURVATURE_TRANSITION_HPP_
