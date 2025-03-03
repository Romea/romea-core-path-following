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
#include <random>
#include <string>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_path_following/setpoint.hpp"

class TestEvaluateSetPoint : public ::testing::Test
{
public:
  TestEvaluateSetPoint()
  {}

  double evaluateLinearSpeed()
  {
    return romea::core::path_following::evaluate_setpoint(
      desiredSetPoint, pathMatchedPoint).linear_speed;
  }

  romea::core::path_following::SetPoint desiredSetPoint;
  romea::core::PathMatchedPoint2D pathMatchedPoint;
};

//-----------------------------------------------------------------------------
TEST_F(TestEvaluateSetPoint, testZeroSpeed) {
  desiredSetPoint.linear_speed = std::numeric_limits<double>::quiet_NaN();
  pathMatchedPoint.desiredSpeed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(evaluateLinearSpeed(), 0);
}

//-----------------------------------------------------------------------------
TEST_F(TestEvaluateSetPoint, testLinearSpeedIsUserLinearSpeed) {
  desiredSetPoint.linear_speed = std::numeric_limits<double>::quiet_NaN();
  pathMatchedPoint.desiredSpeed = 1.0;
  EXPECT_DOUBLE_EQ(evaluateLinearSpeed(), 1.0);
}

//-----------------------------------------------------------------------------
TEST_F(TestEvaluateSetPoint, testLinearSpeedIsPathSpeed) {
  desiredSetPoint.linear_speed = std::numeric_limits<double>::quiet_NaN();
  pathMatchedPoint.desiredSpeed = -2.0;
  EXPECT_DOUBLE_EQ(evaluateLinearSpeed(), -2.0);
}

//-----------------------------------------------------------------------------
TEST_F(TestEvaluateSetPoint, testLinearSpeedIsSignedUserSpeed) {
  desiredSetPoint.linear_speed = 1.0;
  pathMatchedPoint.desiredSpeed = -2.0;
  EXPECT_DOUBLE_EQ(evaluateLinearSpeed(), -1.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
