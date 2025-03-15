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

#ifndef ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_


// std
#include <memory>
#include <utility>
#include <vector>
#include <iostream>

// romea
#include "romea_core_common/geometry/Twist2D.hpp"
#include "romea_core_common/log/SimpleFileLogger.hpp"
#include "romea_core_path/PathFrenetPose2D.hpp"
#include "romea_core_path/PathPosture2D.hpp"
#include "romea_core_path_following/fsm.hpp"
#include "romea_core_path_following/logs.hpp"
#include "romea_core_path_following/traits.hpp"
#include "romea_core_path_following/setpoint.hpp"
#include "romea_core_path_following/utils.hpp"


namespace romea
{
namespace core
{
namespace path_following
{

template<typename CommandType>
class PathFollowingBase
{
public:
  using Logger = core::SimpleFileLogger;
  // using FSM = FSM<CommandType>;
  using OdometryMeasure = typename Traits<CommandType>::Measure;
  using CommandLimits = typename Traits<CommandType>::Limits;

  PathFollowingBase()
  : logger_(nullptr) {}

  virtual ~PathFollowingBase() = default;

  virtual void register_logger(std::shared_ptr<Logger> logger)
  {
    logger_ = logger;
  }

  virtual void set_stop_at_the_end(bool value)
  {
    fsm_.set_stop_at_the_end(value);
  }

  virtual FSMStatus get_status()
  {
    return fsm_.get_status();
  }

  virtual std::optional<CommandType> compute_command(
    const SetPoint & user_setpoint,
    const CommandLimits & command_limits,
    const std::vector<PathMatchedPoint2D> & matched_points,
    const OdometryMeasure & odometry_measure,
    const Twist2D & filtered_twist)
  {
    fsm_.update_matched_points(matched_points);

    if (this->fsm_.get_status() == FSMStatus::FAILED ||
      this->fsm_.get_status() == FSMStatus::FINISH)
    {
      return {};
    }

    auto matched_point = *findMatchedPointBySectionIndex(
      matched_points, fsm_.get_current_section_index());

    auto setpoint = evaluate_setpoint(user_setpoint, matched_point);

    if (this->fsm_.get_status() == FSMStatus::STOP ||
      this->fsm_.get_status() == FSMStatus::CHANGE_DIRECTION)
    {
      setpoint.linear_speed = 0;
    }

    CommandType command;
    if (direction(matched_point) >= 0) {
      command = compute_command(
        setpoint,
        command_limits,
        matched_point.frenetPose,
        matched_point.pathPosture,
        matched_point.futureCurvature,
        odometry_measure,
        filtered_twist);
    } else {
      command = compute_command(
        setpoint,
        command_limits,
        reverse(matched_point.frenetPose),
        reverse(matched_point.pathPosture),
        -matched_point.futureCurvature,
        odometry_measure,
        filtered_twist);
    }

    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>)
    {
      if (this->fsm_.get_status() == FSMStatus::STOP ||
        this->fsm_.get_status() == FSMStatus::CHANGE_DIRECTION)
      {
        command.longitudinalSpeed = 0;
        command.angularSpeed = 0;
      }
    }

    fsm_.update_odometry(command, odometry_measure);
    return command;
  }

  virtual void reset() = 0;

  virtual CommandType compute_command(
    const SetPoint & setpoint,
    const CommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_path_curvature,
    const OdometryMeasure & odometry_measure,
    const Twist2D & filtered_twist) = 0;

protected:
  FSM<CommandType> fsm_;
  std::shared_ptr<Logger> logger_;
};

template<typename LateralControl, typename LongitudinalControl>
class PathFollowingWithoutSlidingObserver
  : public PathFollowingBase<typename LateralControl::Command>
{
public:
  using CommandType = typename LateralControl::Command;
  using OdometryMeasure = typename Traits<CommandType>::Measure;
  using CommandLimits = typename Traits<CommandType>::Limits;

public:
  PathFollowingWithoutSlidingObserver(
    std::shared_ptr<LateralControl> lateral_control,
    std::shared_ptr<LongitudinalControl> longitudinal_control)
  : lateral_control_(lateral_control),
    longitudinal_control_(longitudinal_control)
  {
  }

  CommandType compute_command(
    const SetPoint & setpoint,
    const CommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_curvature,
    const OdometryMeasure & odometry_measure,
    const Twist2D & filtered_twist)
  {
    auto command = this->lateral_control_->compute_command(
      setpoint, command_limits, frenet_pose, path_posture, future_curvature, odometry_measure);

    command.longitudinalSpeed = this->longitudinal_control_->compute_linear_speed(
      setpoint, frenet_pose, path_posture, odometry_measure, filtered_twist);

    if (this->logger_ != nullptr) {
      this->logger_->addEntry("fsm_status", static_cast<int>(this->fsm_.get_status()));
      log(*this->logger_, setpoint);
      log(*this->logger_, frenet_pose);
      log(*this->logger_, path_posture);
      this->logger_->addEntry("path_future_curvature", future_curvature);
      log(*this->logger_, odometry_measure);
      log(*this->logger_, filtered_twist);
      lateral_control_->log(*this->logger_);
      longitudinal_control_->log(*this->logger_);
      log(*this->logger_, command);
    }

    return command;
  }

  void reset() override
  {
    this->fsm_.reset();
    this->lateral_control_->reset();
    this->longitudinal_control_->reset();
  }

  std::shared_ptr<LateralControl> lateral_control_;
  std::shared_ptr<LongitudinalControl> longitudinal_control_;
};

template<typename LateralControl, typename LongitudinalControl, typename SlidingObserver>
class PathFollowingWithSlidingObserver
  : public PathFollowingBase<typename LateralControl::Command>
{
public:
  using CommandType = typename LateralControl::Command;
  using OdometryMeasure = typename Traits<CommandType>::Measure;
  using CommandLimits = typename Traits<CommandType>::Limits;
  using LateralControlSlidings = typename LateralControl::Slidings;
  using ObserverSlidings = typename SlidingObserver::Slidings;

public:
  PathFollowingWithSlidingObserver(
    std::shared_ptr<LateralControl> lateral_control,
    std::shared_ptr<LongitudinalControl> longitudinal_control,
    std::shared_ptr<SlidingObserver> sliding_observer)
  : lateral_control_(lateral_control),
    longitudinal_control_(longitudinal_control),
    sliding_observer_(sliding_observer)
  {
  }

  CommandType compute_command(
    const SetPoint & setpoint,
    const CommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_curvature,
    const OdometryMeasure & odometry_measure,
    const Twist2D & filtered_twist)
  {
    LateralControlSlidings slidings;
    if constexpr (std::is_same_v<LateralControlSlidings, ObserverSlidings>) {
      slidings = sliding_observer_->compute_slidings(
        frenet_pose, path_posture, odometry_measure, filtered_twist);
    } else {
      // convert slidings
      std::cerr << "no sliding converter available for the current config\n";
    }

    auto command = this->lateral_control_->compute_command(
      setpoint, command_limits, frenet_pose, path_posture,
      future_curvature, odometry_measure, slidings);

    command.longitudinalSpeed = this->longitudinal_control_->compute_linear_speed(
      setpoint, frenet_pose, path_posture, odometry_measure, filtered_twist);

    if (this->logger_ != nullptr) {
      this->logger_->addEntry("fsm_status", static_cast<int>(this->fsm_.get_status()));
      log(*this->logger_, setpoint);
      log(*this->logger_, frenet_pose);
      log(*this->logger_, path_posture);
      this->logger_->addEntry("path_future_curvature", future_curvature);
      log(*this->logger_, odometry_measure);
      log(*this->logger_, filtered_twist);
      lateral_control_->log(*this->logger_);
      longitudinal_control_->log(*this->logger_);
      sliding_observer_->log(*this->logger_);
      log(*this->logger_, command);
    }

    return command;
  }

  void reset() override
  {
    this->fsm_.reset();
    this->lateral_control_->reset();
    this->longitudinal_control_->reset();
    this->sliding_observer_->reset();
  }

private:
  std::shared_ptr<LateralControl> lateral_control_;
  std::shared_ptr<LongitudinalControl> longitudinal_control_;
  std::shared_ptr<SlidingObserver> sliding_observer_;
};


class OneAxleSteeringEquivalence
  : public PathFollowingBase<SkidSteeringCommand>
{
public:
  OneAxleSteeringEquivalence(
    std::unique_ptr<PathFollowingBase<OneAxleSteeringCommand>> path_following)
  : path_following_(std::move(path_following))
  {
  }

  void register_logger(std::shared_ptr<Logger> logger)
  {
    logger_ = logger;
    path_following_->register_logger(logger);
  }

  SkidSteeringCommand compute_command(
    const SetPoint & setpoint,
    const SkidSteeringCommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_curvature,
    const SkidSteeringMeasure & odometry_measure,
    const Twist2D & filtered_twist) override
  {
    OneAxleSteeringCommand command = path_following_->compute_command(
      setpoint, to_one_axle_steering_command_limits(command_limits),
      frenet_pose, path_posture, future_curvature,
      to_one_axle_steering_measure(odometry_measure, wheelbase_), filtered_twist);

    if (logger_ != nullptr) {
      log(*this->logger_, odometry_measure);
      log(*this->logger_, to_skid_steering_command(command, wheelbase_));
    }

    return to_skid_steering_command(command, wheelbase_);
  }

  void reset() override
  {
    path_following_->reset();
  }

  const double wheelbase_ = 1.2;
  std::unique_ptr<PathFollowingBase<OneAxleSteeringCommand>> path_following_;
};


}  // namespace path_following
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_
