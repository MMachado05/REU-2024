/*********************************************************************
 * C++ unit test for dbw_polaris_can/pedal_lut.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dbw_polaris_can/pedal_lut.h>
using namespace dbw_polaris_can;

// Test converting from throttle percent to throttle pedal position
TEST(pedal_lut, throttlePedalFromPercent)
{
  // Out of range
  EXPECT_EQ((float)0.200, throttlePedalFromPercent(-INFINITY));
  EXPECT_EQ((float)0.200, throttlePedalFromPercent(-1));
  EXPECT_EQ((float)0.800, throttlePedalFromPercent(2));
  EXPECT_EQ((float)0.800, throttlePedalFromPercent(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.200, throttlePedalFromPercent(0));
  EXPECT_EQ((float)0.800, throttlePedalFromPercent(1));

  // Jump from zero to non-zero
  EXPECT_EQ((float)0.200, throttlePedalFromPercent(0.000));
  EXPECT_EQ((float)0.300, throttlePedalFromPercent(0.001));

  // Normal values
  EXPECT_NEAR((float)0.4496, throttlePedalFromPercent(0.3), (float)0.001);
  EXPECT_NEAR((float)0.5497, throttlePedalFromPercent(0.5), (float)0.001);
  EXPECT_NEAR((float)0.6498, throttlePedalFromPercent(0.7), (float)0.001);
}

// Test converting from throttle pedal position to throttle percent
TEST(pedal_lut, throttlePercentFromPedal)
{
  // Out of range
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(-INFINITY));
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(-1.0));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(1.0));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(0.200));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(0.800));

  // Normal values
  EXPECT_NEAR((float)0.3, throttlePercentFromPedal(0.4496), (float)0.001);
  EXPECT_NEAR((float)0.5, throttlePercentFromPedal(0.5497), (float)0.001);
  EXPECT_NEAR((float)0.7, throttlePercentFromPedal(0.6498), (float)0.001);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

