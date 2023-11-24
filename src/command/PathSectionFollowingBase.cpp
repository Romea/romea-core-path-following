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
#include "romea_core_path_following/PathFollowingLogs.hpp"
#include "romea_core_path_following/command/PathSectionFollowingBase.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
template<class CommandType>
PathSectionFollowingBase<CommandType>::PathSectionFollowingBase(
  const LongitudinalControlParameters & longitudinalControlParameters,
  const CommandLimits & commandLimits)
: commandLimits_(commandLimits),
  longitudinalControl_(longitudinalControlParameters),
  sliding_observer_(nullptr)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathSectionFollowingBase<CommandType>::registerSlidingObserver(
  std::unique_ptr<SlidingObserver> sliding_observer)
{
  sliding_observer_ = std::move(sliding_observer);
}

//-----------------------------------------------------------------------------
template<class CommandType>
CommandType PathSectionFollowingBase<CommandType>::computeCommand(
  const PathFollowingSetPoint & setPoint,
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const double & futurePathCurvature,
  const OdometryMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  auto linearSpeedCommand = longitudinalControl_.computeLinearSpeed(
    setPoint, frenetPose, pathPosture, odometryMeasure, filteredTwist);

  auto steeringAnglesCommand = computeSteeringAngles_(
    setPoint, frenetPose, pathPosture, futurePathCurvature, odometryMeasure, filteredTwist);

  return makeCommand_(linearSpeedCommand, steeringAnglesCommand);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathSectionFollowingBase<CommandType>::log(SimpleFileLogger & logger)
{
  log_sliding_observer_data_(logger);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathSectionFollowingBase<CommandType>::reset()
{
  reset_sliding_observer_();
}

//-----------------------------------------------------------------------------
template<class CommandType>
double PathSectionFollowingBase<CommandType>::computeLinearSpeed_(
  const PathFollowingSetPoint & setPoint,
  const PathFrenetPose2D & /*frenetPose*/,
  const PathPosture2D & /*pathPosture*/,
  const OdometryMeasure & /*odometryMeasure*/,
  const Twist2D & /*filteredTwist*/)
{
  return setPoint.linearSpeed;
  // const double &EcartLat = frenet_pose_.lateralDeviation;
  // const double & EcartAng = frenet_pose_.courseDeviation;
  // const double & steering = front_steering_angle_;

  // double flag = 1;
  // if (fabs(steering) < 10 / 180. * M_PI) {flag = 0;}

  // linear_speed_command_ =
  //   fabs(desired_linear_speed_) -
  //   2 * (EcartLat - desired_lateral_deviation_) * (EcartLat - desired_lateral_deviation_) -
  //   10 * EcartAng * EcartAng - 1 * flag * fabs(steering);
  // if (linear_speed_command_ < minimal_linear_speed_command_) {
  //   linear_speed_command_ = minimal_linear_speed_command_;
  // }

  // linear_speed_command_ = std::copysign(linear_speed_command_, desired_linear_speed_);
}

//-----------------------------------------------------------------------------
template<>
TwoAxleSteeringCommand PathSectionFollowingBase<TwoAxleSteeringCommand>::makeCommand_(
  const double & linearSpeedCommand,
  const SteeringAngles & steeringAnglesCommand)
{
  return {linearSpeedCommand, steeringAnglesCommand.front, steeringAnglesCommand.rear};
}

//-----------------------------------------------------------------------------
template<>
OneAxleSteeringCommand PathSectionFollowingBase<OneAxleSteeringCommand>::makeCommand_(
  const double & linearSpeedCommand,
  const SteeringAngles & steeringAnglesCommand)
{
  return {linearSpeedCommand, steeringAnglesCommand.front};
}

//-----------------------------------------------------------------------------
template<class CommandType>
SlidingAngles PathSectionFollowingBase<CommandType>::compute_sliding_angles_(
  const PathFrenetPose2D & frenetPose,
  const PathPosture2D & pathPosture,
  const OdometryMeasure & odometryMeasure,
  const Twist2D & filteredTwist)
{
  if (sliding_observer_ != nullptr) {
    return sliding_observer_->computeSlidingAngles(
      frenetPose, pathPosture, odometryMeasure, filteredTwist);
  } else {
    return {};
  }
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathSectionFollowingBase<CommandType>::log_sliding_observer_data_(SimpleFileLogger & logger)
{
  if (sliding_observer_ != nullptr) {
    sliding_observer_->log(logger);
  }
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathSectionFollowingBase<CommandType>::reset_sliding_observer_()
{
  if (sliding_observer_ != nullptr) {
    sliding_observer_->reset();
  }
}

template class PathSectionFollowingBase<OneAxleSteeringCommand>;
template class PathSectionFollowingBase<TwoAxleSteeringCommand>;

}  // namespace core
}  // namespace romea
