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

#ifndef ROMEA_CORE_PATH_FOLLOWING__SLIDING_OBSERVER__SKID__PICARD_LYAPUNOV_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__SLIDING_OBSERVER__SKID__PICARD_LYAPUNOV_HPP_

// romea
#include "romea_core_control/observer/SlidingObserverPicardSkidLyapunov.hpp"
#include "romea_core_path_following/sliding_observer/skid/base.hpp"

namespace romea::core::path_following
{

template<typename CommandType>
class SlidingObserverPicardSkidLyapunov : public SlidingObserverSkid<CommandType>
{
public:
  using OdometryMeasure = typename Traits<CommandType>::Measure;
  using Observer = core::SlidingObserversPicardSkidLyapunov;
  using Parameters = Observer::Parameters;

public:
  SlidingObserverPicardSkidLyapunov(double samplingPeriod, const Parameters & parameters);

  SkidSlidingParameters compute_slidings(
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const OdometryMeasure & odometry_measure,
    const Twist2D & filtered_twist) override;

  void log(SimpleFileLogger & logger) override;

  void reset() override;

private:
  Observer observer_;
};

}  // namespace romea::core::path_following

#endif  // ROMEA_CORE_PATH_FOLLOWING__SLIDING_OBSERVER__EXTENDED__CINEMATICLINEARTANGENT_HPP_
