/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2019, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Adversarial testing of TrackJoint
*/

// C++
#include <iostream>
#include <math.h>
#include <string>

// Testing
#include <gtest/gtest.h>
#include "test_utilities.h"

// Target testing library
#include <trackjoint/trajectory_generator.h>

namespace trackjoint
{
class HelperFunctionTest : public ::testing::Test
{
public:
  HelperFunctionTest()
  {
  }
};  // class HelperFunctionTest

TEST_F(HelperFunctionTest, MultipleCalls)
{
  EXPECT_EQ(true, true);
}
}  // namespace trackjoint

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
