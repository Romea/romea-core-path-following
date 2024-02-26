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


#ifndef ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__LATERALCONTROLCLASSIC_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__LATERALCONTROLCLASSIC_HPP_

// romea
#include "romea_core_control/command/FollowTrajectoryClassicSliding.hpp"
#include "romea_core_path_following/lateral_control/LateralControlBase.hpp"
#include "romea_core_path_following/sliding_observer/SlidingObserverExtended.hpp"

namespace romea
{
namespace core
{

template<typename CommandType>
class PathFollowingLateralControlClassic
{
};

template<>
class PathFollowingLateralControlClassic<OneAxleSteeringCommand>
  : public PathFollowingLateralControlBase<OneAxleSteeringCommand, ExtendedSlidings>
{
public:
  using LateralControl = FollowTrajectoryClassicSliding;

  struct Gains
  {
    double frontKD;
  };

  struct Parameters
  {
    Gains gains;
  };

public:
  PathFollowingLateralControlClassic(
    const double & samplePeriod,
    const double & wheelbase,
    const MobileBaseInertia & inertia,
    const Parameters & parameters);

  OneAxleSteeringCommand computeCommand(
    const PathFollowingSetPoint & setPoint,
    const CommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const ExtendedSlidings & slidings = {}) override;

  void updateGains(const Gains & gains);

  void log(SimpleFileLogger & logger) override;

  void reset() override;

private:
  SharedVariable<Gains> gains_;
  LateralControl lateralControl_;
};


template<>
class PathFollowingLateralControlClassic<TwoAxleSteeringCommand>
  : public PathFollowingLateralControlBase<TwoAxleSteeringCommand, ExtendedSlidings>
{
public:
  using LateralControl = FollowTrajectoryClassicSliding;

  struct Gains
  {
    double frontKD;
    double rearKD;
  };

  struct Parameters
  {
    Gains gains;
  };

public:
  PathFollowingLateralControlClassic(
    const double & samplePeriod,
    const double & wheelbase,
    const MobileBaseInertia & inertia,
    const Parameters & parameters);

  TwoAxleSteeringCommand computeCommand(
    const PathFollowingSetPoint & setPoint,
    const CommandLimits & commandLimits,
    const PathFrenetPose2D & frenetPose,
    const PathPosture2D & pathPosture,
    const double & futurePathCurvature,
    const OdometryMeasure & odometryMeasure,
    const ExtendedSlidings & slidings = {}) override;

  void updateGains(const Gains & gains);

  void log(SimpleFileLogger & logger) override;

  void reset() override;

private:
  SharedVariable<Gains> gains_;
  LateralControl lateralControl_;
};


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__LATERAL_CONTROL__PATHFOLLOWINGLATERALCONTROLCLASSICONEAXLESTEERING_HPP_
