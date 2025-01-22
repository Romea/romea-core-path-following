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
#include "romea_core_path_following/PathFollowingPlatoon.hpp"

namespace
{

//-----------------------------------------------------------------------------
double getCurvature(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return info.matchedPoints[0].pathPosture.curvature;
}

//-----------------------------------------------------------------------------
double getCurvilinearAbscissa(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return info.matchedPoints[0].frenetPose.curvilinearAbscissa;
}

//-----------------------------------------------------------------------------
Eigen::Vector2d  getPosition(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return globalPosition(info.matchedPoints[0]);
}

//-----------------------------------------------------------------------------
double getLateralDeviation(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return info.matchedPoints[0].frenetPose.lateralDeviation;
}

//-----------------------------------------------------------------------------
double getCourseDeviation(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return info.matchedPoints[0].frenetPose.courseDeviation;
}

//-----------------------------------------------------------------------------
double getLongitudinalSpeed(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return info.twist.linearSpeeds.x();
}

//-----------------------------------------------------------------------------
double getSectionIndex(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return info.matchedPoints[0].sectionIndex;
}

//-----------------------------------------------------------------------------
bool hasSingleMatchedPoint(const romea::core::PathFollowingPlatoon::PathMatchingInfo & info)
{
  return info.matchedPoints.size() == 1;
}

//-----------------------------------------------------------------------------
bool isAvailable(
  const romea::core::PathFollowingPlatoon::PathMatchingInfo & info,
  const romea::core::Duration & stamp
)
{
  return hasSingleMatchedPoint(info) && romea::core::durationToSecond(stamp - info.stamp) < 0.2;
}

}  // namespace


namespace romea
{
namespace core
{

//---------------------------------------------------------------------------
PathFollowingPlatoon::PathFollowingPlatoon(
  const double & samplingPeriod,
  const double & desiredInterDistance,
  const double & maximalLinearSpeed,
  const double & minimalLinearAcceleration,
  const double & maximalLinearAcceleration)
: samplingPeriod_(samplingPeriod),
  desiredInterDistance_(desiredInterDistance),
  maximalLinearSpeed_(maximalLinearSpeed),
  minimalLinearAcceleration_(minimalLinearAcceleration),
  maximalLinearAcceleration_(maximalLinearAcceleration),
  previousLinearSpeedCommand_(std::numeric_limits<double>::quiet_NaN()),
  currentLinearSpeedCommand_(std::numeric_limits<double>::quiet_NaN()),
  logger_(nullptr)
{
  keepInterdistance_ = std::make_shared<KeepInterdistance>(samplingPeriod);
}

//---------------------------------------------------------------------------
void PathFollowingPlatoon::registerLogger(std::shared_ptr<Logger> logger)
{
  logger_ = logger;
}


//---------------------------------------------------------------------------
void PathFollowingPlatoon::setPreviousVehicleInfo(const PathMatchingInfo & info)
{
  previousVehicleInfo_ = info;
}

//---------------------------------------------------------------------------
void PathFollowingPlatoon::setCurrentVehicleInfo(const PathMatchingInfo & info)
{
  currentVehicleInfo_ = info;
}

//---------------------------------------------------------------------------
void PathFollowingPlatoon::setNextVehicleInfo(const PathMatchingInfo & info)
{
  nextVehicleInfo_ = info;
}

//---------------------------------------------------------------------------
bool PathFollowingPlatoon::isVehicleInfoAvailable_(
  const romea::core::PathFollowingPlatoon::PathMatchingInfo & info,
  const romea::core::Duration & stamp)
{
  return hasSingleMatchedPoint(info) &&
         romea::core::durationToSecond(stamp - info.stamp) < 2 * samplingPeriod_;
}

//---------------------------------------------------------------------------
std::optional<double> PathFollowingPlatoon::computeLinearSpeedCommand(const Duration & stamp)
{
  auto previousVehicleInfo = previousVehicleInfo_.load();
  auto currentVehicleInfo = currentVehicleInfo_.load();
  auto nextVehicleInfo = nextVehicleInfo_.load();

  // check rate equal to sampling period = stamp;
  if (isVehicleInfoAvailable_(currentVehicleInfo_, stamp) &&
    isVehicleInfoAvailable_(nextVehicleInfo, stamp) &&
    getSectionIndex(nextVehicleInfo) == getSectionIndex(currentVehicleInfo_))
  {
    return computeLinearSpeedCommand_(previousVehicleInfo, currentVehicleInfo, nextVehicleInfo);
  } else {
    previousLinearSpeedCommand_ = std::numeric_limits<double>::quiet_NaN();
    return {};
  }
}

//---------------------------------------------------------------------------
double PathFollowingPlatoon::computeLinearSpeedCommand_(
  const PathMatchingInfo & previousVehicleInfo_,
  const PathMatchingInfo & currentVehicleInfo,
  const PathMatchingInfo & nextVehicleInfo)
{
  auto leader_position = getPosition(nextVehicleInfo_);
  auto leader_linear_speed = getLongitudinalSpeed(nextVehicleInfo_);
  auto leader_course_deviation = getCourseDeviation(nextVehicleInfo_);
  auto leader_lateral_deviation = getLateralDeviation(nextVehicleInfo_);
  auto leader_curvilinear_abscissa = getCurvilinearAbscissa(nextVehicleInfo_);

  auto follower_position = getLongitudinalSpeed(currentVehicleInfo_);
  auto follower_linear_speed = getLongitudinalSpeed(currentVehicleInfo_);
  auto follower_course_deviation = getCourseDeviation(currentVehicleInfo_);
  auto follower_lateral_deviation = getLateralDeviation(currentVehicleInfo_);
  auto follower_curvilinear_abscissa = getCurvilinearAbscissa(currentVehicleInfo_);

  std::cout << "leader_lateral_deviation " << leader_lateral_deviation << std::endl;
  std::cout << "follower_lateral_deviation " << follower_lateral_deviation << std::endl;
  std::cout << "leader_curvilinear_abscissa " << leader_curvilinear_abscissa << std::endl;
  std::cout << "follower_curvilinear_abscissa " << follower_curvilinear_abscissa << std::endl;

  auto curvature = getCurvature(currentVehicleInfo_);

  // Compute the correct speed to apply on the follower by taking account the curvature
  double projected_leader_linear_speed =
    leader_linear_speed * (1 + curvature * follower_lateral_deviation);

  currentLinearSpeedCommand_ = keepInterdistance_->computeFollowerSpeed(
    desiredInterDistance_,
    leader_curvilinear_abscissa - follower_curvilinear_abscissa,
    projected_leader_linear_speed,
    maximalLinearSpeed_,
    follower_lateral_deviation,
    follower_course_deviation,
    curvature,
    follower_linear_speed);

  clampLinearSpeedCommand_();

// // avoid approaching the leader when the euclidian distance is below the desired interdistance
//   if(euclidian_distance > std::abs(desired_interdistance_))
//   {
//     speed = keep_interdistance_.computeFollowerSpeed(
//       desired_interdistance_,
//       interdistance,
//       projected_leader_linear_speed,
//       desired_maximal_speed_,
//       follower_lateral_deviation,
//       follower_course_deviation,
//       curvature,
//       follower_linear_speed);
//   }
//   else
//   {
//     if(leader_pos.x() > 0)
//     {
//       speed = 0;
//     }
//     else
//     {
//       speed = keep_interdistance_.computeFollowerSpeed(
//         std::copysign(leader_pos.x(), desired_interdistance_),
//         std::copysign(leader_pos.x(), euclidian_distance),
//         projected_leader_linear_speed,
//         desired_maximal_speed_,
//         follower_lateral_deviation,
//         follower_course_deviation,
//         curvature,
//         follower_linear_speed);
//     }
//   }


  if (logger_) {
    logger_->addEntry("interdistance", desiredInterDistance_);
    logger_->addEntry("desired_speed", currentLinearSpeedCommand_);
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
    logger_->addEntry("linear_speed_command", currentLinearSpeedCommand_);
    logger_->writeRow();
  }

  return currentLinearSpeedCommand_;
}


//---------------------------------------------------------------------------
void PathFollowingPlatoon::clampLinearSpeedCommand_()
{
  const double & dt = samplingPeriod_;

  std::cout << "previousLinearSpeedCommand_ " << previousLinearSpeedCommand_ << std::endl;
  std::cout << "currentLinearSpeedCommand_ " << currentLinearSpeedCommand_ << std::endl;
  currentLinearSpeedCommand_ = std::clamp(
    currentLinearSpeedCommand_, -maximalLinearSpeed_, maximalLinearSpeed_);
  std::cout << "clamped currentLinearSpeedCommand_ " << currentLinearSpeedCommand_ << std::endl;

  if (!std::isfinite(previousLinearSpeedCommand_)) {
    previousLinearSpeedCommand_ = getLongitudinalSpeed(currentVehicleInfo_);
  }

  double acceleration = (currentLinearSpeedCommand_ - previousLinearSpeedCommand_) / dt;

  if (acceleration > maximalLinearAcceleration_) {
    currentLinearSpeedCommand_ = previousLinearSpeedCommand_ + maximalLinearAcceleration_ * dt;
  } else if (acceleration < minimalLinearAcceleration_) {
    currentLinearSpeedCommand_ = previousLinearSpeedCommand_ + minimalLinearAcceleration_ * dt;
  }

  std::cout << "clamped acc currentLinearSpeedCommand_ " << currentLinearSpeedCommand_ << std::endl;

  previousLinearSpeedCommand_ = currentLinearSpeedCommand_;
}


}   // namespace core
}  // namespace romea
