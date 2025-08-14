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

#ifndef ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__BASE_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__BASE_HPP_

// romea
#include <romea_core_common/geometry/Twist2D.hpp>
#include <romea_core_common/log/SimpleFileLogger.hpp>
#include <romea_core_path/PathFrenetPose2D.hpp>
#include <romea_core_path/PathPosture2D.hpp>
#include <romea_core_path_following/logs.hpp>
#include <romea_core_path_following/traits.hpp>

namespace romea::core::path_following
{

template<class CommandType>
class LongitudinalControlBase
{
public:
  using Command = CommandType;
  using OdometryMeasure = typename Traits<CommandType>::Measure;

public:
  virtual ~LongitudinalControlBase() {}

  virtual double compute_linear_speed(
    const SetPoint & setpoint,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    double future_curvature,
    const OdometryMeasure & odometry_measure,
    const Twist2D & filtered_twist) = 0;

  virtual void log(SimpleFileLogger & logger) = 0;

  virtual void reset() = 0;
};

}  // namespace romea::core::path_following

#endif  // ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__BASE_HPP_
