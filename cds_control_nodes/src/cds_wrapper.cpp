#include <seds_control_nodes/cds_wrapper.hpp>
#include <seds_control_nodes/kdl_epfl_conversions.hpp>
#include <iostream>

CdsWrapper::CdsWrapper() : 
    arm_( ArmKinematics() ), cds_controller_( CDSExecution() )
{
}

CdsWrapper::CdsWrapper(CdsWrapperParams params, const KDL::JntArray& q_init) :
    arm_( ArmKinematics() ), cds_controller_( CDSExecution() )
{
  this->init(params, q_init);
}

CdsWrapper::~CdsWrapper()
{
}

void CdsWrapper::init(CdsWrapperParams params, const KDL::JntArray& q_init)
{
  arm_.init(params.arm_params_);

  cds_controller_.setObjectFrame(toMathLib(params.cds_params_.object_frame_));

  cds_controller_.setAttractorFrame(toMathLib(params.cds_params_.attractor_frame_));

  cds_controller_.setMotionParameters(params.cds_params_.alpha_, 
      params.cds_params_.beta_, params.cds_params_.lambda_,
      params.cds_params_.reachingThreshold_); 

  cds_controller_.setDT(params.cds_params_.dt_);

  cds_controller_.setCurrentEEPose(toMathLib(arm_.get_pos_fk(q_init)));

  cds_controller_.init(static_cast<GMRDynamics*>(params.cds_params_.master_dyn_),
      static_cast<GMRDynamics*>(params.cds_params_.slave_dyn_), 
      static_cast<GMR*>(params.cds_params_.coupling_));
}

void printFrame(const KDL::Frame& frame)
{
  std::cout << "p: " << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "\n";
//  double x, y, z, w;
//  frame.M.GetQuaternion(x, y, z, w);
//  std::cout << "M: " << x << ", " << y << ", " << z << ", " << w << "\n";
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
//  std::cout << "\n\n";
//  return arm_.get_vel_ik(q, toKDL(cds_controller_.getNextEEPose()), dt);
  return arm_.get_vel_ik(q, des_pose, dt);
}
