/*********************************************************************
 * C++ unit test for dbw_polaris_can/PlatformVersion.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dbw_polaris_can/PlatformVersion.h>
using namespace dbw_polaris_can;

// Test empty constructor
TEST(PlatformVersion, empty)
{
  EXPECT_EQ((Platform)0,     PlatformVersion().p);
  EXPECT_EQ(  (Module)0,     PlatformVersion().m);
  EXPECT_EQ(ModuleVersion(), PlatformVersion().v);
}

// Test that fields are populated by each constructor
TEST(PlatformVersion, constructor)
{
  EXPECT_EQ((Platform)1,          PlatformVersion((Platform)1, (Module)2, ModuleVersion(3,4,5)).p);
  EXPECT_EQ((Module)2,            PlatformVersion((Platform)1, (Module)2, ModuleVersion(3,4,5)).m);
  EXPECT_EQ(ModuleVersion(3,4,5), PlatformVersion((Platform)1, (Module)2, ModuleVersion(3,4,5)).v);
  EXPECT_EQ((Platform)1,          PlatformVersion((Platform)1, (Module)2, 3, 4, 5).p);
  EXPECT_EQ((Module)2,            PlatformVersion((Platform)1, (Module)2, 3, 4, 5).m);
  EXPECT_EQ(ModuleVersion(3,4,5), PlatformVersion((Platform)1, (Module)2, 3, 4, 5).v);
}

// Test operators
TEST(ModuleVersion, operators)
{
  const Platform x = (Platform)1; const Module y = (Module)2;
  const Platform X = (Platform)3; const Module Y = (Module)3;

  // Compare PlatformVersion with PlatformVersion
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) <  PlatformVersion(x,y,1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) <  PlatformVersion(x,y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) <  PlatformVersion(X,y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) <  PlatformVersion(x,Y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,9,9,9) <= PlatformVersion(x,y,1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) <= PlatformVersion(x,y,1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) <= PlatformVersion(x,y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) <= PlatformVersion(X,y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) <= PlatformVersion(x,Y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) >  PlatformVersion(x,y,9,9,9));
  EXPECT_TRUE (PlatformVersion(x,y,9,9,9) >  PlatformVersion(x,y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,9,9,9) >  PlatformVersion(X,y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,9,9,9) >  PlatformVersion(x,Y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) >= PlatformVersion(x,y,9,9,9));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) >= PlatformVersion(x,y,1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,9,9,9) >= PlatformVersion(x,y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,9,9,9) >= PlatformVersion(X,y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,9,9,9) >= PlatformVersion(x,Y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) == PlatformVersion(x,y,9,9,9));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) == PlatformVersion(x,y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) == PlatformVersion(X,y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) == PlatformVersion(x,Y,1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) != PlatformVersion(x,y,1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) != PlatformVersion(x,y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) != PlatformVersion(X,y,9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) != PlatformVersion(x,Y,9,9,9));

  // Compare PlatformVersion with ModuleVersion
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) <  ModuleVersion(1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) <  ModuleVersion(9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,9,9,9) <= ModuleVersion(1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) <= ModuleVersion(1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) <= ModuleVersion(9,9,9));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) >  ModuleVersion(9,9,9));
  EXPECT_TRUE (PlatformVersion(x,y,9,9,9) >  ModuleVersion(1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) >= ModuleVersion(9,9,9));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) >= ModuleVersion(1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,9,9,9) >= ModuleVersion(1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) == ModuleVersion(9,9,9));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) == ModuleVersion(1,1,1));
  EXPECT_FALSE(PlatformVersion(x,y,1,1,1) != ModuleVersion(1,1,1));
  EXPECT_TRUE (PlatformVersion(x,y,1,1,1) != ModuleVersion(9,9,9));
}

// Test platformToString()
TEST(ModuleVersion, platformToString)
{
  EXPECT_STREQ("FORD_CD4",    platformToString(P_FORD_CD4));
  EXPECT_STREQ("FORD_P5",     platformToString(P_FORD_P5));
  EXPECT_STREQ("FORD_C1",     platformToString(P_FORD_C1));
  EXPECT_STREQ("FORD_T6",     platformToString(P_FORD_T6));
  EXPECT_STREQ("FORD_U6",     platformToString(P_FORD_U6));
  EXPECT_STREQ("FORD_CD5",    platformToString(P_FORD_CD5));
  EXPECT_STREQ("FORD_GE1",    platformToString(P_FORD_GE1));
  EXPECT_STREQ("FORD_P702",   platformToString(P_FORD_P702));
  EXPECT_STREQ("FCA_RU",      platformToString(P_FCA_RU));
  EXPECT_STREQ("FCA_WK2",     platformToString(P_FCA_WK2));
  EXPECT_STREQ("POLARIS_GEM", platformToString(P_POLARIS_GEM));
  EXPECT_STREQ("POLARIS_RZR", platformToString(P_POLARIS_RZR));
  for (size_t i = 0x90; i <= UINT8_MAX; i++) {
    EXPECT_STREQ("UNKNOWN", platformToString((Platform)i)) << "i = " << i;
  }
}

// Test moduleToString()
TEST(ModuleVersion, moduleToString)
{
  EXPECT_STREQ("BPEC ", moduleToString(M_BPEC));
  EXPECT_STREQ("TPEC ", moduleToString(M_TPEC));
  EXPECT_STREQ("STEER", moduleToString(M_STEER));
  EXPECT_STREQ("SHIFT", moduleToString(M_SHIFT));
  EXPECT_STREQ("ABS  ", moduleToString(M_ABS));
  EXPECT_STREQ("BOO  ", moduleToString(M_BOO));
  EXPECT_STREQ("EPS  ", moduleToString(M_EPS));
  EXPECT_STREQ("UNKNOWN", moduleToString((Module)0));
  for (size_t i = 8; i <= UINT8_MAX; i++) {
    EXPECT_STREQ("UNKNOWN", moduleToString((Module)i)) << "i = " << i;
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

