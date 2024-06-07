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

#ifndef ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGFSM_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGFSM_HPP_

// std
#include <optional>
#include <vector>

// romea
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include "romea_core_path_following/PathFollowingTraits.hpp"

namespace romea
{
namespace core
{

enum class PathFollowingFSMStatus
{
  INIT,
  FOLLOW,
  STOP,
  CHANGE_DIRECTION,
  FINISH,
  FAILED
};

constexpr const char * PATH_FOLLOWING_FSM_STATUS_STRINGS[] = {
  "INIT",
  "FOLLOW",
  "STOP",
  "CHANGE_DIRECTION",
  "FINISH",
  "FAILED"
};

template<typename CommandType>
class PathFollowingFSM
{
public:
  using FeedbackType = typename PathFollowingTraits<CommandType>::Measure;

public:
  PathFollowingFSM();

  void updateMatchedPoints(const std::vector<PathMatchedPoint2D> & matchedPoints);
  void updateOdometry(const CommandType & command, const FeedbackType & odometry);
  const PathFollowingFSMStatus & getStatus() const;
  const size_t & getCurrentSectionIndex() const;
  void reset();
  const char * getStatusString();
  void setStopAtTheEnd(bool value) { stop_at_the_end_ = value; }

private:
  void initCallback_();
  void followCallback_();
  void stopCallback_();
  void changeDirectionCallback_();
  void setStatus(PathFollowingFSMStatus status);

private:
  CommandType command_;
  FeedbackType feedback_;
  std::vector<PathMatchedPoint2D> matchedPoints_;

  size_t currentSectionIndex_;
  PathFollowingFSMStatus status_;
  bool stop_at_the_end_ = true;
};

}   // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGFSM_HPP_
