#include <gtest/gtest.h>
#include <seds_control_nodes/kdl_epfl_conversions.hpp>

using namespace std;

class ConversionTests : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }

  public:
 
};

TEST_F(ConversionTests, Vector3)
{
  KDL::Vector vector(1.1, 2.2, 3.3);
  KDL::Vector vector2 = toKDL(toMathLib(vector));
  EXPECT_TRUE(KDL::Equal(vector, vector2));
}

TEST_F(ConversionTests, Rotation)
{
  KDL::Rotation rot = KDL::Rotation::Quaternion(0.481, 0.725, 0.289, -0.400);
  KDL::Rotation rot2 = toKDL(toMathLib(rot));  
  EXPECT_TRUE(KDL::Equal(rot, rot2));
}

TEST_F(ConversionTests, Frame)
{
  KDL::Frame frame = KDL::Frame(KDL::Rotation::Quaternion(0.410, 0.714, 0.322, -0.468), KDL::Vector(-0.135, 0.615, 0.748));
  KDL::Frame frame2 = toKDL(toMathLib(frame));
  EXPECT_TRUE(KDL::Equal(frame, frame2));

  KDL::Frame pose0 = KDL::Frame(KDL::Rotation::Quaternion(-0.413, -0.692, -0.170, 0.567), KDL::Vector(-0.251, 0.428, 0.846));
  KDL::Frame pose1 = KDL::Frame(KDL::Rotation::Quaternion(0.286, 0.826, 0.158, -0.460), KDL::Vector(-0.400, 0.562, 0.630));
  KDL::Frame pose2 = KDL::Frame(KDL::Rotation::Quaternion(0.481, 0.725, 0.289, -0.400), KDL::Vector(-0.071, 0.667, 0.659));
  KDL::Frame pose3 = KDL::Frame(KDL::Rotation::Quaternion(0.410, 0.714, 0.322, -0.468), KDL::Vector(-0.135, 0.615, 0.748));

  frame2 = toKDL(toMathLib(pose0));
  EXPECT_TRUE(KDL::Equal(pose0, frame2));

  frame2 = toKDL(toMathLib(pose1));
  EXPECT_TRUE(KDL::Equal(pose1, frame2));

  frame2 = toKDL(toMathLib(pose2));
  EXPECT_TRUE(KDL::Equal(pose2, frame2));

  frame2 = toKDL(toMathLib(pose3));
  EXPECT_TRUE(KDL::Equal(pose3, frame2));
}
