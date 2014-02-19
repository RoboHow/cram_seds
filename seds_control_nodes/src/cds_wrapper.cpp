#include <seds_control_nodes/cds_wrapper.hpp>
#include <seds_control_nodes/kdl_epfl_conversions.hpp>

CdsWrapper::CdsWrapper() : 
    arm_( ArmKinematics() ), cds_controller_( CDSExecution() )
{
}

CdsWrapper::CdsWrapper(const CdsWrapperParams& params) :
    arm_( ArmKinematics() ), cds_controller_( CDSExecution() )
{
  this->init(params);
}

CdsWrapper::~CdsWrapper()
{
}

void CdsWrapper::init(const CdsWrapperParams& params)
{
  arm_.init(params.arm_params_);

  CdsWrapperParams params_copy = params;
  cds_controller_.init(&params_copy.cds_params_.master_dyn_,
      &params_copy.cds_params_.slave_dyn_, &params_copy.cds_params_.coupling_);

  cds_controller_.setMotionParameters(params.cds_params_.alpha_, 
      params.cds_params_.beta_, params.cds_params_.lambda_,
      params.cds_params_.reachingThreshold_); 

  cds_controller_.setDT(params.cds_params_.dt_);
}

const KDL::JntArray& CdsWrapper::update(const KDL::JntArray& q, double dt)
{
  assert(q.rows() == arm_.get_dof());
 
  cds_controller_.setCurrentEEPose(toMathLib(arm_.get_pos_fk(q)));
  return arm_.get_vel_ik(q, toKDL(cds_controller_.getNextEEPose()), dt);
}
