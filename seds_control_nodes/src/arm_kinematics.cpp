#include "include/seds_control_nodes/arm_kinematics.hpp"
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <ros/ros.h>

using namespace std;
using namespace KDL;

ArmKinematics::ArmKinematics() :
    fk_solver_(NULL), ik_solver_(NULL), dof_(0), q_tmp_(JntArray(dof_))
{
}

ArmKinematics::ArmKinematics(const urdf::ModelInterface& robot_model, 
    const std::string &root_name, const std::string &tip_name)
{
  this->init(robot_model, root_name, tip_name);
}

ArmKinematics::~ArmKinematics()
{
  if(fk_solver_)
    delete(fk_solver_);

  if(ik_solver_)
    delete(ik_solver_);
}
 
void ArmKinematics::digestIkErrorCode(int error_code) const
{
  if(KDL::SolverI::E_NOERROR != error_code)
  {
    cout << "IK-Solver returned with an error.\n";
    throw;
  }
}

KDL::Frame ArmKinematics::get_fk(const KDL::JntArray& q)
{
  assert(q.rows() == dof_);
  assert(fk_solver_);

  KDL::Frame pose;
  fk_solver_->JntToCart(q, pose);

  return pose;
}

KDL::JntArray& ArmKinematics::get_vel_ik(const KDL::JntArray& q,
    const KDL::Frame& des_pose, double dt)
{
  assert(ik_solver_);
  assert(q.rows() == dof_);
  assert(q.rows() == q_tmp_.rows());
  // TODO(Georg): set JS & TS weights
  // TODO(Georg): set lambda

  digestIkErrorCode(
      ik_solver_->CartToJnt(q, KDL::diff(get_fk(q), des_pose, dt), q_tmp_));

  return q_tmp_;
}

void ArmKinematics::initInternals(const KDL::Chain& chain)
{
  dof_ = chain.getNrOfJoints();
  q_tmp_.resize(dof_);

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain);
  ik_solver_ = new KDL::ChainIkSolverVel_wdls(chain);
}

void ArmKinematics::init(const urdf::ModelInterface& robot_model,
    const string &root_name, const string &tip_name) 
{
  KDL::Chain chain = readChainFromUrdf(robot_model, root_name, tip_name);
  initInternals(chain);
}

KDL::Chain ArmKinematics::readChainFromUrdf(const urdf::ModelInterface& robot_model,
    const std::string &root_name, const std::string &tip_name)
{
  KDL::Tree tree;
  KDL::Chain chain;

  if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
    cout << "Could not parse robot model into a kdl_tree.\n";
    throw;
  }

  if (!tree.getChain(root_name, tip_name, chain)) {
    cout << "Could not initialize chain object\n";
    throw;
  }

  return chain; 
}

int main(int argc, char **argv)
{

  std::cout << "lalala\n";

  ros::init(argc, argv, "seds_wrapper");
  //ros::NodeHandle n;

  ArmKinematics sedswrapper;

//  //Get better frames into the URDF
//  sedswrapper.init("robot_description", "base_link", "left_arm_7_link");
//
//  KDL::JntArray jntarray(7);
//  for(unsigned int i=0; i < 7; i++)
//    jntarray(i) = 0.0;
//  KDL::Frame pose = sedswrapper.get_fk(jntarray);
//  cout << "Position: " << pose.p(0) << "," << pose.p(1) << "," << pose.p(2) << endl;

  return 0;
};
