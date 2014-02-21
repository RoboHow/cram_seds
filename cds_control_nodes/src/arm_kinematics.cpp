#include "seds_control_nodes/arm_kinematics.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>

using namespace std;
using namespace KDL;

ArmKinematics::ArmKinematics() :
    fk_solver_(NULL), ik_solver_(NULL), dof_(0), q_tmp_(JntArray(dof_))
{
}

ArmKinematics::ArmKinematics(const urdf::ModelInterface& robot_model, 
        const std::string &root_name, const std::string &tip_name, double lambda,
        const Eigen::MatrixXd& task_weights, const Eigen::MatrixXd& joint_weights) :
    fk_solver_(NULL), ik_solver_(NULL), dof_(0), q_tmp_(JntArray(dof_))
{
  this->init(robot_model, root_name, tip_name, lambda, task_weights, joint_weights);
}

ArmKinematics::ArmKinematics(const ArmKinematicsParams& params) :
    fk_solver_(NULL), ik_solver_(NULL), dof_(0), q_tmp_(JntArray(dof_))
{
  this->init(params);
}
 
ArmKinematics::~ArmKinematics()
{
  deleteInternals();
}

KDL::Frame ArmKinematics::get_pos_fk(const KDL::JntArray& q)
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

  digestIkErrorCode(
      ik_solver_->CartToJnt(q, KDL::diff(get_pos_fk(q), des_pose, dt), q_tmp_));

  return q_tmp_;
}

void ArmKinematics::init(const ArmKinematicsParams& params)
{
  init(params.robot_model_, params.root_name_, params.tip_name_, params.lambda_,
      params.task_weights_, params.joint_weights_);
}
 
void ArmKinematics::init(const urdf::ModelInterface& robot_model,
    const string &root_name, const string &tip_name, double lambda,
    const Eigen::MatrixXd& task_weights, const Eigen::MatrixXd& joint_weights) 
{
  KDL::Chain chain = readChainFromUrdf(robot_model, root_name, tip_name);
  initInternals(chain);
  setWeights(lambda, task_weights, joint_weights);
}

void ArmKinematics::setLambda(double lambda)
{
  assert(ik_solver_);
  ik_solver_->setLambda(lambda);
}

void ArmKinematics::setJointWeights(const Eigen::MatrixXd& joint_weights)
{
  assert(ik_solver_);
  ik_solver_->setWeightJS(joint_weights);
}

void ArmKinematics::setTaskWeights(const Eigen::MatrixXd& task_weights)
{
  assert(ik_solver_);
  ik_solver_->setWeightTS(task_weights);
}

void ArmKinematics::setWeights(double lambda, const Eigen::MatrixXd& task_weights,
    const Eigen::MatrixXd& joint_weights)
{
  setLambda(lambda);
  setJointWeights(joint_weights);
  setTaskWeights(task_weights);
}

// **************** //
// INTERNAL HELPERS //
// **************** //

void ArmKinematics::deleteInternals()
{
  if(fk_solver_)
    delete fk_solver_;
  if(ik_solver_)
    delete ik_solver_;
}

void ArmKinematics::digestIkErrorCode(int error_code) const
{
  if(KDL::SolverI::E_NOERROR != error_code)
  {
    cout << "IK-Solver returned with an error.\n";
    throw;
  }
}

void ArmKinematics::initInternals(const KDL::Chain& chain)
{
  dof_ = chain.getNrOfJoints();
  q_tmp_.resize(dof_);
  
  deleteInternals();

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain);
  ik_solver_ = new KDL::ChainIkSolverVel_wdls(chain);
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
