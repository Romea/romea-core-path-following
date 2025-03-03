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
#include <memory>

// romea
#include "romea_core_path_following/external_control/platoon.hpp"

namespace
{

//-----------------------------------------------------------------------------
double get_curvature(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return info.matched_points[0].pathPosture.curvature;
}

//-----------------------------------------------------------------------------
double get_curvilinear_abscissa(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return info.matched_points[0].frenetPose.curvilinearAbscissa;
}

//-----------------------------------------------------------------------------
Eigen::Vector2d  get_position(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return globalPosition(info.matched_points[0]);
}

//-----------------------------------------------------------------------------
double get_lateral_deviation(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return info.matched_points[0].frenetPose.lateralDeviation;
}

//-----------------------------------------------------------------------------
double get_course_deviation(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return info.matched_points[0].frenetPose.courseDeviation;
}

//-----------------------------------------------------------------------------
double get_longitudinal_speed(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return info.twist.linearSpeeds.x();
}

//-----------------------------------------------------------------------------
double get_section_index(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return info.matched_points[0].sectionIndex;
}

//-----------------------------------------------------------------------------
bool has_single_matched_point(const romea::core::path_following::Platoon::PathMatchingInfo & info)
{
  return info.matched_points.size() == 1;
}

//-----------------------------------------------------------------------------
bool is_available(
  const romea::core::path_following::Platoon::PathMatchingInfo & info,
  const romea::core::Duration & stamp
)
{
  return has_single_matched_point(info) && romea::core::durationToSecond(stamp - info.stamp) < 0.2;
}

}  // namespace


namespace romea
{
namespace core
{
namespace path_following
{

//---------------------------------------------------------------------------
Platoon::Platoon(
  const double & samplingPeriod,
  const double & desiredInterDistance,
  const double & maximalLinearSpeed,
  const double & minimalLinearAcceleration,
  const double & maximalLinearAcceleration)
: sampling_period_(samplingPeriod),
  desired_inter_distance_(desiredInterDistance),
  maximal_linear_speed_(maximalLinearSpeed),
  minimal_linear_acceleration_(minimalLinearAcceleration),
  maximal_linear_acceleration_(maximalLinearAcceleration),
  previous_linear_speed_command_(std::numeric_limits<double>::quiet_NaN()),
  current_linear_speed_command_(std::numeric_limits<double>::quiet_NaN()),
  logger_(nullptr)
{
  keep_interdistance_ = std::make_shared<KeepInterdistance>(samplingPeriod);
}

//---------------------------------------------------------------------------
void Platoon::registerLogger(std::shared_ptr<Logger> logger)
{
  logger_ = logger;
}


//---------------------------------------------------------------------------
void Platoon::set_previous_vehicle_info(const PathMatchingInfo & info)
{
  previous_vehicle_info_ = info;
}

//---------------------------------------------------------------------------
void Platoon::set_current_vehicle_info(const PathMatchingInfo & info)
{
  current_vehicle_info_ = info;
}

//---------------------------------------------------------------------------
void Platoon::set_next_vehicle_info(const PathMatchingInfo & info)
{
  next_vehicle_info_ = info;
}

//---------------------------------------------------------------------------
bool Platoon::is_vehicle_info_available_(
  const PathMatchingInfo & info,
  const romea::core::Duration & stamp)
{
  return has_single_matched_point(info) &&
         romea::core::durationToSecond(stamp - info.stamp) < 2 * sampling_period_;
}

//---------------------------------------------------------------------------
std::optional<double> Platoon::compute_linear_speed_command(const Duration & stamp)
{
  auto previousVehicleInfo = previous_vehicle_info_.load();
  auto currentVehicleInfo = current_vehicle_info_.load();
  auto nextVehicleInfo = next_vehicle_info_.load();

  // check rate equal to sampling period = stamp;
  if (is_vehicle_info_available_(current_vehicle_info_, stamp) &&
    is_vehicle_info_available_(nextVehicleInfo, stamp) &&
    get_section_index(nextVehicleInfo) == get_section_index(current_vehicle_info_))
  {
    return compute_linear_speed_command_(previousVehicleInfo, currentVehicleInfo, nextVehicleInfo);
  } else {
    previous_linear_speed_command_ = std::numeric_limits<double>::quiet_NaN();
    return {};
  }
}

//---------------------------------------------------------------------------
double Platoon::compute_linear_speed_command_(
  const PathMatchingInfo & previous_vehicle_info_,
  const PathMatchingInfo & current_vehicle_info_,
  const PathMatchingInfo & next_vehicle_info_)
{
  auto leader_position = get_position(next_vehicle_info_);
  auto leader_linear_speed = get_longitudinal_speed(next_vehicle_info_);
  auto leader_course_deviation = get_course_deviation(next_vehicle_info_);
  auto leader_lateral_deviation = get_lateral_deviation(next_vehicle_info_);
  auto leader_curvilinear_abscissa = get_curvilinear_abscissa(next_vehicle_info_);

  auto follower_position = get_longitudinal_speed(current_vehicle_info_);
  auto follower_linear_speed = get_longitudinal_speed(current_vehicle_info_);
  auto follower_course_deviation = get_course_deviation(current_vehicle_info_);
  auto follower_lateral_deviation = get_lateral_deviation(current_vehicle_info_);
  auto follower_curvilinear_abscissa = get_curvilinear_abscissa(current_vehicle_info_);

  auto curvature = get_curvature(current_vehicle_info_);

  // Compute the correct speed to apply on the follower by taking account the curvature
  double projected_leader_linear_speed =
    leader_linear_speed * (1 + curvature * follower_lateral_deviation);

  current_linear_speed_command_ = keep_interdistance_->computeFollowerSpeed(
    desired_inter_distance_,
    leader_curvilinear_abscissa - follower_curvilinear_abscissa,
    projected_leader_linear_speed,
    maximal_linear_speed_,
    follower_lateral_deviation,
    follower_course_deviation,
    curvature,
    follower_linear_speed);

  clamp_linear_speed_command_();

  if (logger_) {
    logger_->addEntry("interdistance", desired_inter_distance_);
    logger_->addEntry("desired_speed", current_linear_speed_command_);
    logger_->addEntry("leader_x", leader_position.x());
    logger_->addEntry("leader_y", leader_position.y());
    logger_->addEntry("leader_curvilinear_abscissa", leader_curvilinear_abscissa);
    logger_->addEntry("leader_linear_speed", leader_linear_speed);
    logger_->addEntry("leader_lateral_deviation", leader_lateral_deviation);
    logger_->addEntry("leader_course_deviation", leader_course_deviation);
    logger_->addEntry("follower_x", leader_position.x());
    logger_->addEntry("follower_y", leader_position.y());
    logger_->addEntry("follower_curvilinear_abscissa", follower_curvilinear_abscissa);
    logger_->addEntry("follower_linear_speed", follower_linear_speed);
    logger_->addEntry("follower_lateral_deviation", follower_lateral_deviation);
    logger_->addEntry("follower_course_deviation", follower_course_deviation);
    logger_->addEntry("curvature", curvature);
    logger_->addEntry("linear_speed_command", current_linear_speed_command_);
    logger_->writeRow();
  }

  return current_linear_speed_command_;
}


//---------------------------------------------------------------------------
void Platoon::clamp_linear_speed_command_()
{
  const double & dt = sampling_period_;

  current_linear_speed_command_ = std::clamp(
    current_linear_speed_command_, -maximal_linear_speed_, maximal_linear_speed_);

  if (!std::isfinite(previous_linear_speed_command_)) {
    previous_linear_speed_command_ = get_longitudinal_speed(current_vehicle_info_);
  }

  double acceleration = (current_linear_speed_command_ - previous_linear_speed_command_) / dt;

  if (acceleration > maximal_linear_acceleration_) {
    current_linear_speed_command_ = previous_linear_speed_command_ + maximal_linear_acceleration_ *
      dt;
  } else if (acceleration < minimal_linear_acceleration_) {
    current_linear_speed_command_ = previous_linear_speed_command_ + minimal_linear_acceleration_ *
      dt;
  }

  previous_linear_speed_command_ = current_linear_speed_command_;
}


}  // namespace path_following
}  // namespace core
}  // namespace romea
