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

#include <romea_core_common/math/Algorithm.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_path_following/sliding_observer/skid/picard_lyapunov.hpp>

namespace romea::core::path_following
{

template<typename CommandType>
SlidingObserverPicardSkidLyapunov<CommandType>::SlidingObserverPicardSkidLyapunov(
  double sampling_period, const Parameters & parameters)
: observer_(sampling_period, parameters)
{
}

template<typename CommandType>
SkidSlidingParameters SlidingObserverPicardSkidLyapunov<CommandType>::compute_slidings(
  const PathFrenetPose2D & frenet_pose,
  const PathPosture2D & path_posture,
  const OdometryMeasure & odometry_measure,
  const Twist2D & /* filtered_twist */)
{
  const double linear_speed = odometry_measure.longitudinalSpeed;
  const auto & path_pos = path_posture.position;
  double x = path_pos.x() - std::sin(path_posture.course) * frenet_pose.lateralDeviation;
  double y = path_pos.y() + std::cos(path_posture.course) * frenet_pose.lateralDeviation;
  double course = path_posture.course + sign(linear_speed) * frenet_pose.courseDeviation;

  observer_.update(x, y, course, linear_speed, odometry_measure.angularSpeed);

  return {
    observer_.getBetaR(),
    observer_.getDotEpsilonSP(),
    observer_.getDotThetaP(),
  };
}

template<typename CommandType>
void SlidingObserverPicardSkidLyapunov<CommandType>::log(SimpleFileLogger & logger)
{
  logger.addEntry("slip_angle", observer_.getBetaR());
  logger.addEntry("lin_vel_disturb", observer_.getDotEpsilonSP());
  logger.addEntry("ang_vel_disturb", observer_.getDotThetaP());
}

template<typename CommandType>
void SlidingObserverPicardSkidLyapunov<CommandType>::reset()
{
  observer_.reset();
}

template class SlidingObserverPicardSkidLyapunov<SkidSteeringCommand>;

}  // namespace romea::core::path_following
