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
#include "romea_core_path_following/PathFollowingFSM.hpp"

template<typename CommandType>
void checkStatus(
  const romea::core::PathFollowingFSM<CommandType> & fsm,
  const romea::core::PathFollowingFSMStatus & status)
{
  EXPECT_EQ(static_cast<int>(fsm.getStatus()), static_cast<int>(status));
}

template<typename CommandType>
void checkSectionIndex(
  const romea::core::PathFollowingFSM<CommandType> & fsm,
  const size_t & sectionIndex)
{
  EXPECT_EQ(fsm.getCurrentSectionIndex(), sectionIndex);
}

class TestFSM : public ::testing::Test
{
public:
  TestFSM()
  {
    firstMatchedPoint.sectionIndex = 1;
    firstMatchedPoint.frenetPose.curvilinearAbscissa = 19.;
    firstMatchedPoint.sectionMinimalCurvilinearAbscissa = 10;
    firstMatchedPoint.sectionMaximalCurvilinearAbscissa = 20;
    secondMatchedPoint.sectionIndex = 1;
    secondMatchedPoint.frenetPose.curvilinearAbscissa = 21.;
    secondMatchedPoint.sectionMinimalCurvilinearAbscissa = 10;
    secondMatchedPoint.sectionMaximalCurvilinearAbscissa = 20;
    thirdMatchedPoint.sectionIndex = 2;
    thirdMatchedPoint.frenetPose.curvilinearAbscissa = 21.;
    thirdMatchedPoint.sectionMinimalCurvilinearAbscissa = 20;
    thirdMatchedPoint.sectionMaximalCurvilinearAbscissa = 30;
  }

  romea::core::PathMatchedPoint2D firstMatchedPoint;
  romea::core::PathMatchedPoint2D secondMatchedPoint;
  romea::core::PathMatchedPoint2D thirdMatchedPoint;
};

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testInit)
{
  romea::core::PathFollowingFSM<romea::core::OneAxleSteeringCommand> fsm;

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::INIT);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testInitFailed)
{
  romea::core::PathFollowingFSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({});

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::FAILED);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFollow)
{
  romea::core::PathFollowingFSM<romea::core::SkidSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::FOLLOW);
  checkSectionIndex(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInFollow)
{
  romea::core::PathFollowingFSM<romea::core::SkidSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});
  fsm.updateMatchedPoints({firstMatchedPoint});

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::FOLLOW);
  checkSectionIndex(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testFailedFromFollow)
{
  romea::core::PathFollowingFSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});
  fsm.updateMatchedPoints({thirdMatchedPoint});

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::FAILED);
  checkSectionIndex(fsm, std::numeric_limits<size_t>::max());
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToStop)
{
  romea::core::PathFollowingFSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});
  fsm.updateMatchedPoints({secondMatchedPoint});

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::STOP);
  checkSectionIndex(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInStop)
{
  romea::core::SkidSteeringCommand command;
  romea::core::SkidSteeringMeasure measure;
  romea::core::PathFollowingFSM<romea::core::SkidSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});
  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.updateOdometry(command, measure);

  fsm.updateMatchedPoints({secondMatchedPoint});
  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.5;
  fsm.updateOdometry(command, measure);

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::STOP);
  checkSectionIndex(fsm, 1);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToChangeDirection)
{
  romea::core::SkidSteeringCommand command;
  romea::core::SkidSteeringMeasure measure;
  romea::core::PathFollowingFSM<romea::core::SkidSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.updateOdometry(command, measure);

  fsm.updateMatchedPoints({secondMatchedPoint, thirdMatchedPoint});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.updateOdometry(command, measure);

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::CHANGE_DIRECTION);
  checkSectionIndex(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInChangeDirectionOneAxleSteering)
{
  romea::core::OneAxleSteeringCommand command;
  romea::core::OneAxleSteeringMeasure measure;
  romea::core::PathFollowingFSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.updateOdometry(command, measure);

  fsm.updateMatchedPoints({secondMatchedPoint, thirdMatchedPoint});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.updateOdometry(command, measure);

  command.steeringAngle = 1.0;
  measure.steeringAngle = 0.0;
  fsm.updateOdometry(command, measure);

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::CHANGE_DIRECTION);
  checkSectionIndex(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFollowOneAxleSteering)
{
  romea::core::OneAxleSteeringCommand command;
  romea::core::OneAxleSteeringMeasure measure;
  romea::core::PathFollowingFSM<romea::core::OneAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.updateOdometry(command, measure);

  fsm.updateMatchedPoints({secondMatchedPoint, thirdMatchedPoint});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.updateOdometry(command, measure);

  command.steeringAngle = 1.0;
  measure.steeringAngle = 1.0;
  fsm.updateOdometry(command, measure);

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::FOLLOW);
  checkSectionIndex(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testStayInChangeDirectionTwoAxleSteering)
{
  romea::core::TwoAxleSteeringCommand command;
  romea::core::TwoAxleSteeringMeasure measure;
  romea::core::PathFollowingFSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.updateOdometry(command, measure);

  fsm.updateMatchedPoints({secondMatchedPoint, thirdMatchedPoint});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.updateOdometry(command, measure);

  command.frontSteeringAngle = 1.0;
  command.rearSteeringAngle = -1.0;
  measure.frontSteeringAngle = 0.0;
  measure.rearSteeringAngle = 0.0;
  fsm.updateOdometry(command, measure);

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::CHANGE_DIRECTION);
  checkSectionIndex(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFollowTwoAxleSteering)
{
  romea::core::TwoAxleSteeringCommand command;
  romea::core::TwoAxleSteeringMeasure measure;
  romea::core::PathFollowingFSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.updateOdometry(command, measure);

  fsm.updateMatchedPoints({secondMatchedPoint, thirdMatchedPoint});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.updateOdometry(command, measure);

  command.frontSteeringAngle = 1.0;
  command.rearSteeringAngle = -1.0;
  measure.frontSteeringAngle = 1.0;
  measure.rearSteeringAngle = -1.0;
  fsm.updateOdometry(command, measure);

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::FOLLOW);
  checkSectionIndex(fsm, 2);
}

//-----------------------------------------------------------------------------
TEST_F(TestFSM, testGoToFinish)
{
  romea::core::TwoAxleSteeringCommand command;
  romea::core::TwoAxleSteeringMeasure measure;
  romea::core::PathFollowingFSM<romea::core::TwoAxleSteeringCommand> fsm;

  fsm.updateMatchedPoints({firstMatchedPoint});

  command.longitudinalSpeed = 1.0;
  measure.longitudinalSpeed = 1.0;
  fsm.updateOdometry(command, measure);

  fsm.updateMatchedPoints({secondMatchedPoint});

  command.longitudinalSpeed = 0.0;
  measure.longitudinalSpeed = 0.0;
  fsm.updateOdometry(command, measure);

  checkStatus(fsm, romea::core::PathFollowingFSMStatus::FINISH);
  checkSectionIndex(fsm, std::numeric_limits<size_t>::max());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
