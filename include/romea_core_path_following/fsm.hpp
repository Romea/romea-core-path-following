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

#ifndef ROMEA_CORE_PATH_FOLLOWING__FSM_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__FSM_HPP_

// std
#include <optional>
#include <vector>

// romea
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include "romea_core_path_following/traits.hpp"

namespace romea
{
namespace core
{
namespace path_following
{

enum class FSMStatus
{
  INIT,
  FOLLOW,
  STOP,
  CHANGE_DIRECTION,
  FINISH,
  FAILED
};

constexpr const char * FSM_STATUS_STRINGS[] = {
  "INIT",
  "FOLLOW",
  "STOP",
  "CHANGE_DIRECTION",
  "FINISH",
  "FAILED"
};

template<typename CommandType>
class FSM
{
public:
  using FeedbackType = typename Traits<CommandType>::Measure;

public:
  FSM();

  void update_matched_points(const std::vector<PathMatchedPoint2D> & matched_points);
  void update_odometry(const CommandType & command, const FeedbackType & odometry);
  const FSMStatus & get_status() const;
  const size_t & get_current_section_index() const;
  void reset();
  const char * get_status_string();
  void set_stop_at_the_end(bool value) {stop_at_the_end_ = value;}

private:
  void init_callback_();
  void follow_callback_();
  void stop_callback_();
  void change_direction_callback_();
  void set_status(FSMStatus status);

private:
  CommandType command_;
  FeedbackType feedback_;
  std::vector<PathMatchedPoint2D> matched_points_;

  size_t current_section_index_;
  FSMStatus status_;
  bool stop_at_the_end_ = true;
};

}  // namespace path_following
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__FSM_HPP_
