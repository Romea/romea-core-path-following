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

// romea
#include "romea_core_path_following/command/PathSectionFollowingBase.hpp"

namespace romea
{
namespace core
{

template<typename CommandType>
class PathFollowingBase
{
public:
  using PathSectionFollowing = PathSectionFollowingBase<CommandType>;
  using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;

  PathFollowingBase(
    std::unique_ptr<PathSectionFollowing> pathSectionFollowing,
    std::shared_ptr<SimpleFileLogger> logger);

  virtual ~PathFollowingBase() = default;

  virtual CommandType computeCommand(
    const PathFollowingSetPoint & setPoint,
    const std::vector<PathMatchedPoint2D> & matchedPoints,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist) = 0;

  virtual void reset();

protected:
  virtual CommandType computeCommand_(
    const PathFollowingSetPoint & setPoint,
    const PathMatchedPoint2D & matchedPoint,
    const OdometryMeasure & odometryMeasure,
    const Twist2D & filteredTwist);

protected:
  std::unique_ptr<PathSectionFollowing> pathSectionFollowing_;
  std::shared_ptr<SimpleFileLogger> logger_;
};


class OneAxleSteeringPathFollowing : public PathFollowingBase<OneAxleSteeringCommand>
{
public:
  OneAxleSteeringPathFollowing(
    std::unique_ptr<PathSectionFollowing> sectionFollowing,
    std::shared_ptr<SimpleFileLogger> logger);

  OneAxleSteeringCommand computeCommand(
    const PathFollowingSetPoint & setPoint,
    const std::vector<PathMatchedPoint2D> & matchedPoints,
    const OneAxleSteeringMeasure & odometryMeasure,
    const Twist2D & filteredTwist) override;
};

class TwoAxleSteeringPathFollowing : public PathFollowingBase<TwoAxleSteeringCommand>
{
public:
  TwoAxleSteeringPathFollowing(
    std::unique_ptr<PathSectionFollowing> sectionFollowing,
    std::shared_ptr<SimpleFileLogger> logger);

  TwoAxleSteeringCommand computeCommand(
    const PathFollowingSetPoint & setPoint,
    const std::vector<PathMatchedPoint2D> & matchedPoints,
    const TwoAxleSteeringMeasure & odometryMeasure,
    const Twist2D & filteredTwist) override;
};


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__PATHFOLLOWING_HPP_
