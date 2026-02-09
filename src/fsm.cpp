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
#include <iostream>
#include <limits>
#include <vector>

// romea
#include "romea_core_path_following/fsm.hpp"

namespace romea
{
namespace core
{
namespace path_following
{

//-----------------------------------------------------------------------------
template<typename CommandType>
FSM<CommandType>::FSM()
: command_(),
  feedback_(),
  matched_points_(),
  current_section_index_(std::numeric_limits<size_t>::max()),
  status_(FSMStatus::INIT)
{
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void FSM<CommandType>::update_matched_points(
  const std::vector<PathMatchedPoint2D> & matched_points)
{
  matched_points_ = matched_points;

  switch (status_) {
    case FSMStatus::INIT:
      init_callback_();
      break;
    case FSMStatus::FOLLOW:
      follow_callback_();
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void FSM<CommandType>::update_odometry(
  const CommandType & command,
  const FeedbackType & feedback)
{
  command_ = command;
  feedback_ = feedback;

  switch (status_) {
    case FSMStatus::STOP:
      stop_callback_();
      break;
    case FSMStatus::CHANGE_DIRECTION:
      change_direction_callback_();
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void FSM<CommandType>::init_callback_()
{
  if (matched_points_.empty()) {
    set_status(FSMStatus::FAILED);
  } else {
    current_section_index_ = matched_points_[0].sectionIndex;
    set_status(FSMStatus::FOLLOW);
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void FSM<CommandType>::follow_callback_()
{
  auto matched_point = findMatchedPointBySectionIndex(
    matched_points_, current_section_index_);

  if (!matched_point.has_value()) {
    set_status(FSMStatus::FAILED);
    current_section_index_ = std::numeric_limits<size_t>::max();
  } else if (
    matched_point->frenetPose.curvilinearAbscissa >=
    matched_point->sectionMaximalCurvilinearAbscissa)
  {
    set_status(FSMStatus::STOP);
  }

}

//-----------------------------------------------------------------------------
template<typename CommandType>
void FSM<CommandType>::stop_callback_()
{
  if (std::abs(feedback_.longitudinalSpeed) < 0.01) {
    if (findMatchedPointBySectionIndex(matched_points_, current_section_index_ + 1).has_value()) {
      set_status(FSMStatus::CHANGE_DIRECTION);
      current_section_index_++;
    } else if (stop_at_the_end_) {
      current_section_index_ = std::numeric_limits<size_t>::max();
      set_status(FSMStatus::FINISH);
    } else {
      set_status(FSMStatus::INIT);
    }

  }
}

//-----------------------------------------------------------------------------
template<>
void FSM<OneAxleSteeringCommand>::change_direction_callback_()
{
  if (std::abs(command_.steeringAngle - feedback_.steeringAngle) < 0.05) {
    set_status(FSMStatus::FOLLOW);
  }
}

//-----------------------------------------------------------------------------
template<>
void FSM<TwoAxleSteeringCommand>::change_direction_callback_()
{
  // is_rear_steering_command_enabled_
  if (
    std::abs(command_.frontSteeringAngle - feedback_.frontSteeringAngle) < 0.05 &&
    std::abs(command_.rearSteeringAngle - feedback_.rearSteeringAngle) < 0.05)
  {
    set_status(FSMStatus::FOLLOW);
  }
}

//-----------------------------------------------------------------------------
template<>
void FSM<SkidSteeringCommand>::change_direction_callback_()
{
  set_status(FSMStatus::FOLLOW);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void FSM<CommandType>::reset()
{
  command_ = CommandType();
  feedback_ = FeedbackType();
  set_status(FSMStatus::INIT);
  current_section_index_ = std::numeric_limits<size_t>::max();
}

//-----------------------------------------------------------------------------
template<typename CommandType>
const FSMStatus & FSM<CommandType>::get_status() const
{
  return status_;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
const size_t & FSM<CommandType>::get_current_section_index() const
{
  return current_section_index_;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
const char * FSM<CommandType>::get_status_string()
{
  return FSM_STATUS_STRINGS[static_cast<int>(status_)];
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void FSM<CommandType>::set_status(FSMStatus status)
{
  status_ = status;
  std::cout << "FSM transition to: " << get_status_string() << std::endl;
}

template class FSM<OneAxleSteeringCommand>;
template class FSM<TwoAxleSteeringCommand>;
template class FSM<SkidSteeringCommand>;

}  // namespace path_following
}  // namespace core
}  // namespace romea
