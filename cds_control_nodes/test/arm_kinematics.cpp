#include <gtest/gtest.h>

#include <seds_control_nodes/arm_kinematics.hpp>

using namespace std;

class ArmKinematicsTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      loading_success =
          robot_model.initFile("test_data/boxy_fixed_torso_description.urdf");
      root_name = "calib_right_arm_base_link";
      tip_name = "right_arm_7_link";
      lambda = 0.1;
      joint_weights = Eigen::MatrixXd::Identity(7,7);
      task_weights = Eigen::MatrixXd::Identity(6,6);

      params.robot_model_ = robot_model;
      params.root_name_ = root_name;
      params.tip_name_ = tip_name;
      params.lambda_ = lambda;
      params.joint_weights_ = joint_weights;
      params.task_weights_ = task_weights;

      // SOME TEST DATA RECORDED ON OUR LWR
      using Eigen::operator<<;
      q0.resize(7);
      q1.resize(7);
      q2.resize(7);
      q3.resize(7);

      q0.data << -1.1857824325561523, 0.3677162826061249, 0.26641538739204407, -0.7819305658340454, 0.10815606266260147, 0.7503344416618347, 0.04469866678118706;
      q1.data << -1.0241795778274536, 0.8732185363769531, 0.2493722140789032, -0.5471154451370239, 0.0947522297501564, 0.7503224015235901, -0.33370718359947205;
      q2.data << -1.5342185497283936, 0.8062047362327576, 0.21584944427013397, -0.5880086421966553, 0.0981459990143776, 0.7503093481063843, -0.333711177110672;
      q3.data << -1.4162391424179077, 0.7288282513618469, 0.21713267266750336, -0.48435530066490173, 0.09829842299222946, 0.7503142356872559, -0.333711177110672;

      pose0 = KDL::Frame(KDL::Rotation::Quaternion(-0.413, -0.692, -0.170, 0.567), KDL::Vector(-0.251, 0.428, 0.846));
      pose1 = KDL::Frame(KDL::Rotation::Quaternion(0.286, 0.826, 0.158, -0.460), KDL::Vector(-0.400, 0.562, 0.630));
      pose2 = KDL::Frame(KDL::Rotation::Quaternion(0.481, 0.725, 0.289, -0.400), KDL::Vector(-0.071, 0.667, 0.659));
      pose3 = KDL::Frame(KDL::Rotation::Quaternion(0.410, 0.714, 0.322, -0.468), KDL::Vector(-0.135, 0.615, 0.748));
    }

    virtual void TearDown()
    {
    }

    // our tiny little LWR
    urdf::Model robot_model;
    bool loading_success;
    string root_name, tip_name;    
    double lambda;
    Eigen::MatrixXd task_weights, joint_weights;
    ArmKinematicsParams params;

    // some kinematic configurations as recorded on robot
    KDL::JntArray q0, q1, q2, q3;
    KDL::Frame pose0, pose1, pose2, pose3;

    static const double eps_position = 0.001;
    static const double eps_orientation = 0.1;
  public:
    void printFrame(const KDL::Frame& frame) const
    {
      std::cout << "p: " << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "\n";
      double x, y, z, w;
      frame.M.GetQuaternion(x, y, z, w);
      std::cout << "M: " << x << ", " << y << ", " << z << ", " << w << "\n";
    }

    void testCartCtrl(ArmKinematics& arm, const KDL::JntArray& start_q, const KDL::Frame& goal)
    {
      KDL::JntArray q_state = start_q;
      double dt = 0.001;
      for(unsigned int i=0; i<10; i++)
      {
        ASSERT_NO_THROW(arm.get_vel_ik(start_q, goal, dt));
        KDL::JntArray des_q_vel = arm.get_vel_ik(q_state, goal, dt);
        KDL::Multiply(des_q_vel, dt, des_q_vel);
        KDL::Add(q_state, des_q_vel, q_state);
      }
      KDL::Frame new_pose = arm.get_pos_fk(q_state);
      EXPECT_TRUE(KDL::Equal(new_pose.p, goal.p, eps_position)); 
      EXPECT_TRUE(KDL::Equal(new_pose.M, goal.M, eps_orientation)); 

      // print out values in case we have an error
      if(!(KDL::Equal(new_pose.p, goal.p, eps_position) &&
           KDL::Equal(new_pose.M, goal.M, eps_orientation)))
      {
        std::cout << "new_pose:\n";
        printFrame(new_pose);
        std::cout << "goal:\n";
        printFrame(goal);
        std::cout << "start:\n";
        printFrame(arm.get_pos_fk(start_q));
      }
    }


    void testCartFK(ArmKinematics& arm, const KDL::JntArray& q, const KDL::Frame& pose)
    {
      ASSERT_NO_THROW(arm.get_pos_fk(q));
      EXPECT_EQ(arm.get_dof(), 7);
      KDL::Frame pose_calculated = arm.get_pos_fk(q);
      EXPECT_TRUE(KDL::Equal(pose_calculated.p, pose.p, eps_position)); 
      EXPECT_TRUE(KDL::Equal(pose_calculated.M, pose.M, eps_orientation)); 

      // print out values in case we have an error
      if(!(KDL::Equal(pose_calculated.p, pose.p, eps_position) &&
           KDL::Equal(pose_calculated.M, pose.M, eps_orientation)))
      {
        std::cout << "pose_calculated:\n";
        printFrame(pose_calculated);
        std::cout << "pose:\n";
        printFrame(pose);
      }
 
    }
};

TEST_F(ArmKinematicsTest, Init)
{
  ASSERT_TRUE(loading_success);
  ArmKinematics arm;
  ASSERT_NO_THROW(arm.init(robot_model, root_name, tip_name, lambda, task_weights, joint_weights));
  ASSERT_NO_THROW(arm.init(params));
}

TEST_F(ArmKinematicsTest, Position_FK)
{
  ASSERT_TRUE(loading_success);
  ArmKinematics arm(robot_model, root_name, tip_name, lambda, task_weights, joint_weights);

  testCartFK(arm, q0, pose0);
  testCartFK(arm, q1, pose1);
  testCartFK(arm, q2, pose2);
  testCartFK(arm, q3, pose3);
}

TEST_F(ArmKinematicsTest, Velocity_IK)
{
  ASSERT_TRUE(loading_success);
  ArmKinematics arm(params);
  testCartCtrl(arm, q0, pose0);
  testCartCtrl(arm, q0, pose1);
  testCartCtrl(arm, q0, pose2);
  testCartCtrl(arm, q0, pose3);

  testCartCtrl(arm, q1, pose0);
  testCartCtrl(arm, q1, pose1);
  testCartCtrl(arm, q1, pose2);
  testCartCtrl(arm, q1, pose3);

  testCartCtrl(arm, q2, pose0);
  testCartCtrl(arm, q2, pose1);
  testCartCtrl(arm, q2, pose2);
  testCartCtrl(arm, q2, pose3);

  testCartCtrl(arm, q3, pose0);
  testCartCtrl(arm, q3, pose1);
  testCartCtrl(arm, q3, pose2);
  testCartCtrl(arm, q3, pose3);
}
