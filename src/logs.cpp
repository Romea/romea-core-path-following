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


#include "romea_core_path_following/logs.hpp"
#include "romea_core_path_following/utils.hpp"

namespace romea
{
namespace core
{
namespace path_following
{


//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const PathFrenetPose2D & frenet_pose)
{
  logger.addEntry("curvilinear_abscissa", frenet_pose.curvilinearAbscissa);
  logger.addEntry("lateral_deviation", frenet_pose.lateralDeviation);
  logger.addEntry("course_deviation", frenet_pose.courseDeviation);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const PathPosture2D & path_posture)
{
  logger.addEntry("path_x", path_posture.position.x());
  logger.addEntry("path_y", path_posture.position.y());
  logger.addEntry("path_course", path_posture.course);
  logger.addEntry("path_curvature", path_posture.curvature);
}


//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const PathMatchedPoint2D & path_matched_point)
{
  log(logger, path_matched_point.frenetPose);
  log(logger, path_matched_point.pathPosture);
  logger.addEntry("path_future_curvature", path_matched_point.futureCurvature);
  logger.addEntry("path_desired_speed", path_matched_point.desiredSpeed);
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const SetPoint & setpoint)
{
  logger.addEntry("desired_linear_speed", setpoint.linear_speed);
  logger.addEntry("desired_lateral_deviation", setpoint.lateral_deviation);
  logger.addEntry("desired_course_deviation", setpoint.course_deviation);
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
  logger.addEntry("odometry_front_steering_angle", get_front_steering_angle(odometry_measure));
  logger.addEntry("odometry_rear_steering_angle", get_rear_steering_angle(odometry_measure));
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const TwoAxleSteeringMeasure & odometry_measure)
{
  logger.addEntry("odometry_longitudinal_speed", odometry_measure.longitudinalSpeed);
  logger.addEntry("odometry_front_steering_angle", get_front_steering_angle(odometry_measure));
  logger.addEntry("odometry_rear_steering_angle", get_rear_steering_angle(odometry_measure));
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
  logger.addEntry("command_front_steering_angle", get_front_steering_angle(command));
  logger.addEntry("command_rear_steering_angle", get_rear_steering_angle(command));
}

//-----------------------------------------------------------------------------
void log(SimpleFileLogger & logger, const TwoAxleSteeringCommand & command)
{
  logger.addEntry("command_linear_speed_command", command.longitudinalSpeed);
  logger.addEntry("command_front_steering_angle", get_front_steering_angle(command));
  logger.addEntry("command_rear_steering_angle", get_rear_steering_angle(command));
}

}  // namespace path_following
}  // namespace core
}  // namespace romea
