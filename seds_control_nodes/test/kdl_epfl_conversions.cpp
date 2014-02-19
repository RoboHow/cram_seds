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
}
