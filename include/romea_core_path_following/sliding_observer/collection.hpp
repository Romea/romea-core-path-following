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


#ifndef ROMEA_CORE_PATH_FOLLOWING__SLIDING_OBSERVER__COLLECTION_HPP_
#define ROMEA_CORE_PATH_FOLLOWING__SLIDING_OBSERVER__COLLECTION_HPP_


// //romea
// namespace romea
// {

// template<typename CommandType>
// class PathFollowingSlidingObserverCollection
//   : public PathFollowingSlidingObserverBase<CommandType>
// {

// public:
//   using SlidingObserver = PathFollowingSlidingObserverBase<CommandType>;
//   using OdometryMeasure = typename PathFollowingTraits<CommandType>::Measure;

// public:
//   PathFollowingSlidingObserverCollection();

//   void register (
//     const std::string & observerName,
//     std::unique_ptr<SlidingObserver> oberserver);

//   void select(const std::string & observerName);

//   SlidingAngles computeSlidingAngles(
//     const PathFrenetPose2D & frenetPose,
//     const PathPosture2D & pathPosture,
//     const OdometryMeasure & odometryMeasure,
//     const Twist2D & filteredTwist) override;

//   void log(SimpleFileLogger & logger) override;

//   void reset() override;

// private:
//   std::string selectedObserver_;
//   std::map<std::string, std::unique_ptr<SlidingObserver>> observers
// };

// }  // namespace romea

#endif  // ROMEA_CORE_PATH_FOLLOWING__SLIDING_OBSERVER__COLLECTION_HPP_
