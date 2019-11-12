// #include <cmath>

#include <gtest/gtest.h>

#include "convert_3d.hpp"

int return0() { return 0; }

TEST(two, twoTwo) {
  RobotPose a{0.7061677339487551,  0.262849431412194, 0.09274767218605995,
              -0.9026369937560488, 3.297144346959495, 0.2371163594259623},
      b{0.7061677339487551, 0.262849431412194,  0.09274767218605995,
        0.7524445536723043, -2.748522743586558, -0.1976618668089841};
  GeometricTransformation a_t = toGeometricTransformation(a);
  GeometricTransformation b_t = toGeometricTransformation(b);

  double eps = 0.00005;

  EXPECT_NEAR(a_t(0,0), b_t(0,0), eps);
  EXPECT_NEAR(a_t(0,1), b_t(0,1), eps);
  EXPECT_NEAR(a_t(0,2), b_t(0,2), eps);
  EXPECT_NEAR(a_t(0,3), b_t(0,3), eps);
  EXPECT_NEAR(a_t(1,0), b_t(1,0), eps);
  EXPECT_NEAR(a_t(1,1), b_t(1,1), eps);
  EXPECT_NEAR(a_t(1,2), b_t(1,2), eps);
  EXPECT_NEAR(a_t(1,3), b_t(1,3), eps);
  EXPECT_NEAR(a_t(2,0), b_t(2,0), eps);
  EXPECT_NEAR(a_t(2,1), b_t(2,1), eps);
  EXPECT_NEAR(a_t(2,2), b_t(2,2), eps);
  EXPECT_NEAR(a_t(2,3), b_t(2,3), eps);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}