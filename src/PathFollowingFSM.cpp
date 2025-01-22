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

// std
#include <limits>
#include <vector>
#include <iostream>

// romea
#include "romea_core_path_following/PathFollowingFSM.hpp"

namespace romea
{
namespace core
{
//-----------------------------------------------------------------------------
template<typename CommandType>
PathFollowingFSM<CommandType>::PathFollowingFSM()
: command_(),
  feedback_(),
  matchedPoints_(),
  currentSectionIndex_(std::numeric_limits<size_t>::max()),
  status_(PathFollowingFSMStatus::INIT)
{
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingFSM<CommandType>::updateMatchedPoints(
  const std::vector<PathMatchedPoint2D> & matchedPoints)
{
  matchedPoints_ = matchedPoints;

  switch (status_) {
    case PathFollowingFSMStatus::INIT:
      initCallback_();
      break;
    case PathFollowingFSMStatus::FOLLOW:
      followCallback_();
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingFSM<CommandType>::updateOdometry(
  const CommandType & command,
  const FeedbackType & feedback)
{
  command_ = command;
  feedback_ = feedback;

  switch (status_) {
    case PathFollowingFSMStatus::STOP:
      stopCallback_();
      break;
    case PathFollowingFSMStatus::CHANGE_DIRECTION:
      changeDirectionCallback_();
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingFSM<CommandType>::initCallback_()
{
  if (matchedPoints_.empty()) {
    // setStatus(PathFollowingFSMStatus::FAILED);
  } else {
    currentSectionIndex_ = matchedPoints_[0].sectionIndex;
    setStatus(PathFollowingFSMStatus::FOLLOW);
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingFSM<CommandType>::followCallback_()
{
  auto matchedPoint = findMatchedPointBySectionIndex(
    matchedPoints_, currentSectionIndex_);

  if (!matchedPoint.has_value()) {
    setStatus(PathFollowingFSMStatus::FAILED);
    currentSectionIndex_ = std::numeric_limits<size_t>::max();
  } else if (stop_at_the_end_ &&
    matchedPoint->frenetPose.curvilinearAbscissa >= matchedPoint->sectionMaximalCurvilinearAbscissa)
  {
    if (stop_at_the_end_) {
      setStatus(PathFollowingFSMStatus::STOP);
    }
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingFSM<CommandType>::stopCallback_()
{
  if (std::abs(feedback_.longitudinalSpeed) < 0.01) {
    if (findMatchedPointBySectionIndex(matchedPoints_, currentSectionIndex_ + 1).has_value()) {
      setStatus(PathFollowingFSMStatus::CHANGE_DIRECTION);
      currentSectionIndex_++;
    } else {
      currentSectionIndex_ = std::numeric_limits<size_t>::max();
      setStatus(PathFollowingFSMStatus::FINISH);
    }
  }
}

//-----------------------------------------------------------------------------
template<>
void PathFollowingFSM<OneAxleSteeringCommand>::changeDirectionCallback_()
{
  if (std::abs(command_.steeringAngle - feedback_.steeringAngle) < 0.05) {
    setStatus(PathFollowingFSMStatus::FOLLOW);
  }
}

//-----------------------------------------------------------------------------
template<>
void PathFollowingFSM<TwoAxleSteeringCommand>::changeDirectionCallback_()
{
  // is_rear_steering_command_enabled_
  if (std::abs(command_.frontSteeringAngle - feedback_.frontSteeringAngle) < 0.05 &&
    std::abs(command_.rearSteeringAngle - feedback_.rearSteeringAngle) < 0.05)
  {
    setStatus(PathFollowingFSMStatus::FOLLOW);
  }
}

//-----------------------------------------------------------------------------
template<>
void PathFollowingFSM<SkidSteeringCommand>::changeDirectionCallback_()
{
  setStatus(PathFollowingFSMStatus::FOLLOW);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingFSM<CommandType>::reset()
{
  command_ = CommandType();
  feedback_ = FeedbackType();
  setStatus(PathFollowingFSMStatus::INIT);
  currentSectionIndex_ = std::numeric_limits<size_t>::max();
}

//-----------------------------------------------------------------------------
template<typename CommandType>
const PathFollowingFSMStatus & PathFollowingFSM<CommandType>::getStatus() const
{
  return status_;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
const size_t & PathFollowingFSM<CommandType>::getCurrentSectionIndex() const
{
  return currentSectionIndex_;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
const char * PathFollowingFSM<CommandType>::getStatusString()
{
  return PATH_FOLLOWING_FSM_STATUS_STRINGS[static_cast<int>(status_)];
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowingFSM<CommandType>::setStatus(PathFollowingFSMStatus status)
{
  status_ = status;
  std::cout << "PathFollowingFSM transition to: " << getStatusString() << std::endl;
}

template class PathFollowingFSM<OneAxleSteeringCommand>;
template class PathFollowingFSM<TwoAxleSteeringCommand>;
template class PathFollowingFSM<SkidSteeringCommand>;

}  // namespace core
}  // namespace romea
