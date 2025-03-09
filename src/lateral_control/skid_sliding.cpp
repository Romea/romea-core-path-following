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
#include <boost/atomic/detail/futex.hpp>
#include <romea_core_control/command/FollowTrajectorySkidSliding.hpp>

// local
#include "romea_core_path_following/lateral_control/skid_sliding.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea::core::path_following
{

LateralControlSkidSliding<SkidSteeringCommand>::LateralControlSkidSliding(
  double /*samplePeriod*/,
  double /*wheelbase*/,
  const MobileBaseInertia & /*inertia*/,
  const Parameters & parameters)
: gains(parameters.gains), target_course_(0)
{
}

SkidSteeringCommand LateralControlSkidSliding<SkidSteeringCommand>::compute_command(
  const SetPoint & set_point,
  const CommandLimits & command_limits,
  const PathFrenetPose2D & frenet_pose,
  const PathPosture2D & path_posture,
  const double & /*future_path_curvature*/,
  const OdometryMeasure & odometry_measure,
  const WildcardSlidings & /*slidings*/)
{
  auto cur_gains = gains.load();

  double angular_speed = computeBacksteppingSkidSteering(
    frenet_pose.lateralDeviation,
    frenet_pose.courseDeviation,
    path_posture.curvature,
    odometry_measure.longitudinalSpeed,
    0.,
    0.,
    0.,
    get_maximal_angular_speed(command_limits),
    set_point.lateral_deviation,
    cur_gains.lateral_kp,
    cur_gains.course_kp,
    target_course_);

  return {set_point.linear_speed, angular_speed};
}

void LateralControlSkidSliding<SkidSteeringCommand>::log(SimpleFileLogger & logger)
{
  logger.addEntry("target_course", target_course_);
}

}  // namespace romea::core::path_following
