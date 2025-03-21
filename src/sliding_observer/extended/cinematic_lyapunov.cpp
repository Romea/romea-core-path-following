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
#include "romea_core_path_following/utils.hpp"
#include "romea_core_path_following/sliding_observer/extended/cinematic_lyapunov.hpp"


namespace romea
{
namespace core
{
namespace path_following
{

//-----------------------------------------------------------------------------
template<typename CommandType>
SlidingObserverExtendedCinematicLyapunov<CommandType>::SlidingObserverExtendedCinematicLyapunov(
  const double & samplingPeriod,
  const double & wheelBase,
  const MobileBaseInertia & /*inertia*/,
  const Parameters & parameters)
: x_(0), y_(0), course_(0), observer_(samplingPeriod, wheelBase, parameters)
{
}

//-----------------------------------------------------------------------------
template<typename CommandType>
ExtendedSlidings SlidingObserverExtendedCinematicLyapunov<CommandType>::compute_slidings(
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const OdometryMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  const double linearSpeed = filteredTwist.linearSpeeds.x();
  x_ = pathPosture.position.x() - std::sin(pathPosture.course) * frenetPose.lateralDeviation;
  y_ = pathPosture.position.y() + std::cos(pathPosture.course) * frenetPose.lateralDeviation;
  course_ = pathPosture.course + sign(linearSpeed) * frenetPose.courseDeviation;

  observer_.update(
    x_, y_, course_, linearSpeed,
    get_front_steering_angle(odometryMeasure),
    get_rear_steering_angle(odometryMeasure));

  if (frenetPose.curvilinearAbscissa < 5) {
    observer_.initObserverHandbooks_(x_, y_, course_);
  }

  return {0.0, observer_.getFrontSlidingAngle(), observer_.getRearSlidingAngle()};
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void SlidingObserverExtendedCinematicLyapunov<CommandType>::reset()
{
  observer_.reset();
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void SlidingObserverExtendedCinematicLyapunov<CommandType>::log(SimpleFileLogger & logger)
{
  logger.addEntry("x", x_);
  logger.addEntry("y", y_);
  logger.addEntry("course", course_);
  logger.addEntry("XObs", observer_.getX());
  logger.addEntry("YObs", observer_.getY());
  logger.addEntry("ThetaObs", observer_.getTheta());
  logger.addEntry("BetaRHand", observer_.getRearSlidingAngle());
  logger.addEntry("BetaFHand", observer_.getFrontSlidingAngle());
}

template class SlidingObserverExtendedCinematicLyapunov<OneAxleSteeringCommand>;
template class SlidingObserverExtendedCinematicLyapunov<TwoAxleSteeringCommand>;

}   // namespace path_following
}   // namespace core
}   // namespace romea
