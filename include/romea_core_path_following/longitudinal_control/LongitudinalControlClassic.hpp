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


#ifndef ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__LONGITUDINALCONTROLCLASSIC_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__LONGITUDINALCONTROLCLASSIC_HPP_


// romea
#include "romea_core_path_following/longitudinal_control/LongitudinalControlBase.hpp"

namespace romea
{
namespace core
{
template<class CommandType>
class PathFollowingLongitudinalControlClassic
{
};

template<>
class PathFollowingLongitudinalControlClassic<OneAxleSteeringCommand>
  : public PathFollowingLongitudinalControlBase<OneAxleSteeringCommand>
{
public:
  struct Parameters
  {
    // double minimalLinearSpeed;
  };

public:
  explicit PathFollowingLongitudinalControlClassic(const Parameters & parameters);

  double computeLinearSpeed(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist) override;

  void log(SimpleFileLogger & logger) override;

  void reset() override;

protected:
  double minimalLinearSpeed_;
};


template<>
class PathFollowingLongitudinalControlClassic<SkidSteeringCommand>
  : public PathFollowingLongitudinalControlBase<SkidSteeringCommand>
{
public:
  struct Parameters
  {
  };

public:
  explicit PathFollowingLongitudinalControlClassic(const Parameters & parameters);

  double computeLinearSpeed(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)override;

  void log(SimpleFileLogger & logger)override;

  void reset()override;
};

template<>
class PathFollowingLongitudinalControlClassic<TwoAxleSteeringCommand>
  : public PathFollowingLongitudinalControlBase<TwoAxleSteeringCommand>
{
public:
  struct Parameters
  {
  };

public:
  explicit PathFollowingLongitudinalControlClassic(const Parameters & parameters);

  double computeLinearSpeed(
    const PathFollowingSetPoint & setPoint,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist)override;

  void log(SimpleFileLogger & logger)override;

  void reset()override;
};


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__LONGITUDINAL_CONTROL__LONGITUDINALCONTROLCLASSIC_HPP_
