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


#ifndef ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGLOGS_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGLOGS_HPP_

#include "romea_core_common/geometry/Twist2D.hpp"
#include "romea_core_common/log/SimpleFileLogger.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringMeasure.hpp"
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include "romea_core_path_following/PathFollowingSetPoint.hpp"

namespace romea
{
namespace core
{

void log(SimpleFileLogger & logger, const PathFrenetPose2D & frenetPose);

void log(SimpleFileLogger & logger, const PathPosture2D & pathPosture);

void log(SimpleFileLogger & logger, const PathMatchedPoint2D & parthMatchedPoint);

void log(SimpleFileLogger & logger, const PathFollowingSetPoint & setpoint);

void log(SimpleFileLogger & logger, const SkidSteeringMeasure & odometry_measure);

void log(SimpleFileLogger & logger, const OneAxleSteeringMeasure & odometry_measure);

void log(SimpleFileLogger & logger, const TwoAxleSteeringMeasure & odometry_measure);

void log(SimpleFileLogger & logger, const SkidSteeringCommand & command);

void log(SimpleFileLogger & logger, const OneAxleSteeringCommand & command);

void log(SimpleFileLogger & logger, const TwoAxleSteeringCommand & command);

void log(SimpleFileLogger & logger, const Twist2D & twist);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWINGLOGS_HPP_
