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
#include "romea_core_common/math/Algorithm.hpp"
#include "romea_core_path_following/PathFollowingUtils.hpp"
#include "romea_core_path_following/observer/PathFollowingSlidingObserverCinematicLinearTangent.hpp"


namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
template<typename CommandType>
PathFollowingSlidingObserverCinematicLinearTangent<CommandType>::
PathFollowingSlidingObserverCinematicLinearTangent(
  const double & samplingPeriod,
  const double & wheelBase,
  const MobileBaseInertia & /*inertia*/,
  const ObserverParameters & parameters)
: observer_(samplingPeriod, wheelBase, parameters)
{
}

//-----------------------------------------------------------------------------
template<typename CommandType> SlidingAngles
PathFollowingSlidingObserverCinematicLinearTangent<CommandType>::computeSlidingAngles(
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const OdometryMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  const double linearSpeed = filteredTwist.linearSpeeds.x();

  observer_.update(
    frenetPose.lateralDeviation,
    sign(linearSpeed) * frenetPose.courseDeviation,
    pathPosture.curvature,
    linearSpeed,
    getFrontSteeringAngle(odometryMeasure),
    getRearSteeringAngle(odometryMeasure));

  if (frenetPose.curvilinearAbscissa < 5) {
    observer_.initObserver_(frenetPose.lateralDeviation, frenetPose.courseDeviation);
  }

  return {observer_.getFrontSlidingAngle(), observer_.getRearSlidingAngle()};
}
//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingSlidingObserverCinematicLinearTangent<CommandType>::reset()
{
  observer_.reset();
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingSlidingObserverCinematicLinearTangent<CommandType>::log(SimpleFileLogger & logger)
{
  logger.addEntry("Elat4", observer_.getLateralDeviation());
  logger.addEntry("Ecap4", observer_.getCourseDeviation());
  logger.addEntry("BR", observer_.getRearSlidingAngle());
  logger.addEntry("BF", observer_.getFrontSlidingAngle());
}

template class PathFollowingSlidingObserverCinematicLinearTangent<OneAxleSteeringCommand>;
template class PathFollowingSlidingObserverCinematicLinearTangent<TwoAxleSteeringCommand>;

}  // namespace core
}  // namespace romea
