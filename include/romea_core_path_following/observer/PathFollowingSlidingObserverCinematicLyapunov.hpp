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


#ifndef ROMEA_CORE_PATH_FOLLOWING__OBSERVER__PATHFOLLOWINGSLIDINGOBSERVERCINEMATICLYAPUNOV_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__OBSERVER__PATHFOLLOWINGSLIDINGOBSERVERCINEMATICLYAPUNOV_HPP_


// romea
#include "romea_core_control/observer/SlidingObserverCinematicLyapunov.hpp"
#include "romea_core_path_following/observer/PathFollowingSlidingObserverBase.hpp"

namespace romea
{
namespace core
{

using PathFollowingSlidingObserverCinematicLyapunovParameters =
  SlidingObserverCinematicLyapunov::Parameters;

template<typename CommandType>
class PathFollowingSlidingObserverCinematicLyapunov
  : public PathFollowingSlidingObserverBase<CommandType>
{
public:
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;
  using Observer = SlidingObserverCinematicLyapunov;
  using ObserverParameters = Observer::Parameters;

public:
  PathFollowingSlidingObserverCinematicLyapunov(
    const double & samplingPeriod,
    const double & wheelBase,
    const MobileBaseInertia & inertia,
    const ObserverParameters & parameters);

  SlidingAngles computeSlidingAngles(
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTtwist) override;

  void log(SimpleFileLogger & logger) override;

  void reset() override;

private:
  double x_;
  double y_;
  double course_;
  Observer observer_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__OBSERVER__PATHFOLLOWINGSLIDINGOBSERVERCINEMATICLYAPUNOV_HPP_
