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

#ifndef ROMEA_CORE_PATH_FOLLOWING__PLATOON_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PLATOON_HPP_


// std
#include <memory>
#include <utility>
#include <vector>

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/geometry/Twist2D.hpp"
#include "romea_core_common/log/SimpleFileLogger.hpp"
#include "romea_core_common/concurrency/SharedVariable.hpp"
#include "romea_core_path/PathMatchedPoint2D.hpp"

#include "romea_core_control/command/KeepInterdistance.hpp"


namespace romea
{
namespace core
{
namespace path_following
{

class Platoon
{
public:
  using Logger = core::SimpleFileLogger;
  using PathMatchedPoints2D = std::vector<PathMatchedPoint2D>;

  struct PathMatchingInfo
  {
    Duration stamp;
    PathMatchedPoints2D matched_points;
    Twist2D twist;
  };

  Platoon(
    const double & sampling_period,
    const double & desired_inter_distance,
    const double & maximal_linear_Speed,
    const double & minimal_linear_acceleration,
    const double & maximal_linear_acceleration);

  virtual void registerLogger(std::shared_ptr<Logger> logger);

  void set_previous_vehicle_info(const PathMatchingInfo & info);
  void set_current_vehicle_info(const PathMatchingInfo & info);
  void set_next_vehicle_info(const PathMatchingInfo & info);

  std::optional<double> compute_linear_speed_command(const Duration & stamp);

private:
  double compute_linear_speed_command_(
    const PathMatchingInfo & previous_vehicle_info_,
    const PathMatchingInfo & current_vehicle_info,
    const PathMatchingInfo & next_vehicle_info);

  void clamp_linear_speed_command_();

  bool is_vehicle_info_available_(const PathMatchingInfo & info, const Duration & stamp);

private:
  SharedVariable<PathMatchingInfo> previous_vehicle_info_;
  SharedVariable<PathMatchingInfo> current_vehicle_info_;
  SharedVariable<PathMatchingInfo> next_vehicle_info_;

  double sampling_period_;
  double desired_inter_distance_;
  double maximal_linear_speed_;
  double minimal_linear_acceleration_;
  double maximal_linear_acceleration_;
  std::shared_ptr<KeepInterdistance> keep_interdistance_;

  double previous_linear_speed_command_;
  double current_linear_speed_command_;

  std::shared_ptr<Logger> logger_;
};

}  // namespace path_following
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGPLATOON_HPP_
