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

// std
#include <limits>
#include <vector>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_path_following/fsm.hpp"

template<typename CommandType>
void check_status(
  const romea::core::path_following::FSM<CommandType> & fsm,
  const romea::core::path_following::FSMStatus & status)
{
  EXPECT_EQ(static_cast<int>(fsm.get_status()), static_cast<int>(status));
}

template<typename CommandType>
void check_section_index(
  const romea::core::path_following::FSM<CommandType> & fsm,
  const size_t & section_index)
{
  EXPECT_EQ(fsm.get_current_section_index(), section_index);
}

class TestFSM : public ::testing::Test
{
public:
  TestFSM()
  {
    first_matched_point.sectionIndex = 1;
    first_matched_point.frenetPose.curvilinearAbscissa = 19.;
    first_matched_point.sectionMinimalCurvilinearAbscissa = 10;
    first_matched_point.sectionMaximalCurvilinearAbscissa = 20;
    second_matched_point.sectionIndex = 1;
    second_matched_point.frenetPose.curvilinearAbscissa = 21.;
    second_matched_point.sectionMinimalCurvilinearAbscissa = 10;
    second_matched_point.sectionMaximalCurvilinearAbscissa = 20;
    third_matched_point.sectionIndex = 2;
    third_matched_point.frenetPose.curvilinearAbscissa = 21.;
    third_matched_point.sectionMinimalCurvilinearAbscissa = 20;
    third_matched_point.sectionMaximalCurvilinearAbscissa = 30;
  }

  romea::core::PathMatchedPoint2D first_matched_point;
  romea::core::PathMatchedPoint2D second_matched_point;
  romea::core::PathMatchedPoint2D third_matched_point;
};

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testInit)
{
  romea::core::path_following::FSM<romea::core::OneAxleSteeringCommand> fsm;

  check_status(fsm, romea::core::path_following::FSMStatus::INIT);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testInitFailed)
{
  romea::core::path_following::FSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.update_matched_points({});

  check_status(fsm, romea::core::path_following::FSMStatus::FAILED);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFollow)
{
  romea::core::path_following::FSM<romea::core::SkidSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});

  check_status(fsm, romea::core::path_following::FSMStatus::FOLLOW);
  check_section_index(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInFollow)
{
  romea::core::path_following::FSM<romea::core::SkidSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});
  fsm.update_matched_points({first_matched_point});

  check_status(fsm, romea::core::path_following::FSMStatus::FOLLOW);
  check_section_index(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testFailedFromFollow)
{
  romea::core::path_following::FSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});
  fsm.update_matched_points({third_matched_point});

  check_status(fsm, romea::core::path_following::FSMStatus::FAILED);
  check_section_index(fsm, std::numeric_limits<size_t>::max());
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToStop)
{
  romea::core::path_following::FSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});
  fsm.update_matched_points({second_matched_point});

  check_status(fsm, romea::core::path_following::FSMStatus::STOP);
  check_section_index(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInStop)
{
  romea::core::SkidSteeringCommand command;
  romea::core::SkidSteeringMeasure measure;
  romea::core::path_following::FSM<romea::core::SkidSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});
  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.update_odometry(command, measure);

  fsm.update_matched_points({second_matched_point});
  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.5;
  fsm.update_odometry(command, measure);

  check_status(fsm, romea::core::path_following::FSMStatus::STOP);
  check_section_index(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToChangeDirection)
{
  romea::core::SkidSteeringCommand command;
  romea::core::SkidSteeringMeasure measure;
  romea::core::path_following::FSM<romea::core::SkidSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.update_odometry(command, measure);

  fsm.update_matched_points({second_matched_point, third_matched_point});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.update_odometry(command, measure);

  check_status(fsm, romea::core::path_following::FSMStatus::CHANGE_DIRECTION);
  check_section_index(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInChangeDirectionOneAxleSteering)
{
  romea::core::OneAxleSteeringCommand command;
  romea::core::OneAxleSteeringMeasure measure;
  romea::core::path_following::FSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.update_odometry(command, measure);

  fsm.update_matched_points({second_matched_point, third_matched_point});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.update_odometry(command, measure);

  command.steeringAngle = 1.0;
  measure.steeringAngle = 0.0;
  fsm.update_odometry(command, measure);

  check_status(fsm, romea::core::path_following::FSMStatus::CHANGE_DIRECTION);
  check_section_index(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFollowOneAxleSteering)
{
  romea::core::OneAxleSteeringCommand command;
  romea::core::OneAxleSteeringMeasure measure;
  romea::core::path_following::FSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.update_odometry(command, measure);

  fsm.update_matched_points({second_matched_point, third_matched_point});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.update_odometry(command, measure);

  command.steeringAngle = 1.0;
  measure.steeringAngle = 1.0;
  fsm.update_odometry(command, measure);

  check_status(fsm, romea::core::path_following::FSMStatus::FOLLOW);
  check_section_index(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInChangeDirectionTwoAxleSteering)
{
  romea::core::TwoAxleSteeringCommand command;
  romea::core::TwoAxleSteeringMeasure measure;
  romea::core::path_following::FSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.update_odometry(command, measure);

  fsm.update_matched_points({second_matched_point, third_matched_point});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.update_odometry(command, measure);

  command.frontSteeringAngle = 1.0;
  command.rearSteeringAngle = -1.0;
  measure.frontSteeringAngle = 0.0;
  measure.rearSteeringAngle = 0.0;
  fsm.update_odometry(command, measure);

  check_status(fsm, romea::core::path_following::FSMStatus::CHANGE_DIRECTION);
  check_section_index(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFollowTwoAxleSteering)
{
  romea::core::TwoAxleSteeringCommand command;
  romea::core::TwoAxleSteeringMeasure measure;
  romea::core::path_following::FSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.update_odometry(command, measure);

  fsm.update_matched_points({second_matched_point, third_matched_point});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.update_odometry(command, measure);

  command.frontSteeringAngle = 1.0;
  command.rearSteeringAngle = -1.0;
  measure.frontSteeringAngle = 1.0;
  measure.rearSteeringAngle = -1.0;
  fsm.update_odometry(command, measure);

  check_status(fsm, romea::core::path_following::FSMStatus::FOLLOW);
  check_section_index(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFinish)
{
  romea::core::TwoAxleSteeringCommand command;
  romea::core::TwoAxleSteeringMeasure measure;
  romea::core::path_following::FSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.update_matched_points({first_matched_point});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.update_odometry(command, measure);

  fsm.update_matched_points({second_matched_point});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.update_odometry(command, measure);

  check_status(fsm, romea::core::path_following::FSMStatus::FINISH);
  check_section_index(fsm, std::numeric_limits<size_t>::max());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
