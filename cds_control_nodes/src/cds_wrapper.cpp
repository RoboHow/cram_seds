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

  cds_controller_.setDT(params.cds_params_.dt_);

  cds_controller_.setCurrentEEPose(toMathLib(arm_.get_pos_fk(q_init)));

  cds_controller_.init(static_cast<GMRDynamics*>(params.cds_params_.master_dyn_),
      static_cast<GMRDynamics*>(params.cds_params_.slave_dyn_), 
      static_cast<GMR*>(params.cds_params_.coupling_));

  cds_controller_.setMotionParameters(params.cds_params_.alpha_, 
      params.cds_params_.beta_, params.cds_params_.lambda_,
      params.cds_params_.reachingThreshold_); 
}

const KDL::JntArray& CdsWrapper::update(const KDL::JntArray& q, double dt)
{
  assert(q.rows() == arm_.get_dof());

  cds_controller_.setCurrentEEPose(toMathLib(arm_.get_pos_fk(q)));

  KDL::Frame des_pose = toKDL(cds_controller_.getNextEEPose());
// TODO(GEORG): REMOVE THIS!!
des_pose.M = arm_.get_pos_fk(q).M;

//  return arm_.get_vel_ik(q, toKDL(cds_controller_.getNextEEPose()), dt);
  return arm_.get_vel_ik(q, des_pose, dt);
}
