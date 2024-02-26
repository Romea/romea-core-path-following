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


#include "romea_core_path_following/PathFollowingLogs.hpp"
#include "romea_core_path_following/PathFollowingUtils.hpp"

namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const PathFrenetPose2D & frenetPose)
{
  logger.addEntry("curvilinear_abscissa", frenetPose.curvilinearAbscissa);
  logger.addEntry("lateral_deviation", frenetPose.lateralDeviation);
  logger.addEntry("course_deviation", frenetPose.courseDeviation);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const PathPosture2D & pathPosture)
{
  logger.addEntry("path_course", pathPosture.course);
  logger.addEntry("path_curvature", pathPosture.curvature);
}


//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const PathMatchedPoint2D & pathMatchedPoint)
{
  log(logger, pathMatchedPoint.frenetPose);
  log(logger, pathMatchedPoint.pathPosture);
  logger.addEntry("path_future_curvature", pathMatchedPoint.futureCurvature);
  logger.addEntry("path_desired_speed", pathMatchedPoint.desiredSpeed);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const PathFollowingSetPoint & setpoint)
{
  logger.addEntry("desired_linear_speed", setpoint.linearSpeed);
  logger.addEntry("desired_lateral_deviation", setpoint.lateralDeviation);
  logger.addEntry("desired_course_deviation", setpoint.courseDeviation);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const Twist2D & twist)
{
  logger.addEntry("filtered_longitudinal_speed", twist.linearSpeeds.x());
  logger.addEntry("filtered_lateral_speed", twist.linearSpeeds.y());
  logger.addEntry("filtered_angular_speed", twist.angularSpeed);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const SkidSteeringMeasure & odometry_measure)
{
  logger.addEntry("odometry_longitudinal_speed", odometry_measure.longitudinalSpeed);
  logger.addEntry("odometry_angular_speed", odometry_measure.angularSpeed);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const OneAxleSteeringMeasure & odometry_measure)
{
  logger.addEntry("odometry_longitudinal_speed", odometry_measure.longitudinalSpeed);
  logger.addEntry("odometry_front_steering_angle", getFrontSteeringAngle(odometry_measure));
  logger.addEntry("odometry_rear_steering_angle", getRearSteeringAngle(odometry_measure));
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const TwoAxleSteeringMeasure & odometry_measure)
{
  logger.addEntry("odometry_longitudinal_speed", odometry_measure.longitudinalSpeed);
  logger.addEntry("odometry_front_steering_angle", getFrontSteeringAngle(odometry_measure));
  logger.addEntry("odometry_rear_steering_angle", getFrontSteeringAngle(odometry_measure));
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const SkidSteeringCommand & command)
{
  logger.addEntry("command_linear_speed", command.longitudinalSpeed);
  logger.addEntry("command_angular_speed", command.angularSpeed);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const OneAxleSteeringCommand & command)
{
  logger.addEntry("command_linear_speed", command.longitudinalSpeed);
  logger.addEntry("command_front_steering_angle", getFrontSteeringAngle(command));
  logger.addEntry("command_rear_steering_angle", getRearSteeringAngle(command));
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const TwoAxleSteeringCommand & command)
{
  logger.addEntry("command_linear_speed_command", command.longitudinalSpeed);
  logger.addEntry("command_front_steering_angle", getFrontSteeringAngle(command));
  logger.addEntry("command_rear_steering_angle", getFrontSteeringAngle(command));
}

}  // namespace core
}  // namespace romea
