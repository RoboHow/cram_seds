#include <seds_control_nodes/cds_wrapper.hpp>
#include <seds_control_nodes/kdl_epfl_conversions.hpp>
#include <iostream>

CdsWrapper::CdsWrapper() : 
    arm_( ArmKinematics() ), cds_controller_( CDSExecution() )
{
}

CdsWrapper::CdsWrapper(CdsWrapperParams params) :
    arm_( ArmKinematics() ), cds_controller_( CDSExecution() )
{
  this->init(params);
}

CdsWrapper::~CdsWrapper()
{
}

void CdsWrapper::init(CdsWrapperParams params)
{
  arm_.init(params.arm_params_);

  cds_controller_.init(static_cast<GMRDynamics*>(params.cds_params_.master_dyn_),
      static_cast<GMRDynamics*>(params.cds_params_.slave_dyn_), 
      static_cast<GMR*>(params.cds_params_.coupling_));

  cds_controller_.setMotionParameters(params.cds_params_.alpha_, 
      params.cds_params_.beta_, params.cds_params_.lambda_,
      params.cds_params_.reachingThreshold_); 

  cds_controller_.setDT(params.cds_params_.dt_);
}

void CdsWrapper::setGoal(const KDL::Frame& object_frame, const KDL::Frame& attractor_frame)
{
   cds_controller_.setObjectFrame(toMathLib(object_frame));
   cds_controller_.setAttractorFrame(toMathLib(attractor_frame));
}

void printFrame(const KDL::Frame& frame)
{
  std::cout << "p: " << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "\n";
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  std::cout << "M: " << x << ", " << y << ", " << z << ", " << w << "\n";
}

const KDL::JntArray& CdsWrapper::update(const KDL::JntArray& q, double dt)
{
  assert(q.rows() == arm_.get_dof());

  KDL::Frame current_pose = arm_.get_pos_fk(q);
//  cds_controller_.setCurrentEEPose(toMathLib(arm_.get_pos_fk(q)));
  cds_controller_.setCurrentEEPose(toMathLib(current_pose));

  KDL::Frame des_pose = toKDL(cds_controller_.getNextEEPose());

  std::cout << "Current Pose:\n";
  printFrame(current_pose);
  std::cout << "Desired Pose:\n";
  printFrame(des_pose);
  std::cout << "\n\n";
//  return arm_.get_vel_ik(q, toKDL(cds_controller_.getNextEEPose()), dt);
  return arm_.get_vel_ik(q, des_pose, dt);
}
