#include <seds_control_nodes/cds_arm_controller.hpp>

CDSArmController::CDSArmController() : 
    arm_( ArmKinematics() ), cds_controller_( CdsCartesianWrapper() ), dt_( 0.0 )
{
}

CDSArmController::CDSArmController(CDSArmControllerParams params, 
        const KDL::JntArray& q_init) :
    arm_( ArmKinematics() ), cds_controller_( CdsCartesianWrapper() ), dt_( 0.0 )
{
  this->init(params, q_init);
}

CDSArmController::~CDSArmController()
{
}

void CDSArmController::init(CDSArmControllerParams params, 
    const KDL::JntArray& q_init)
{
  dt_ = params.cds_params_.dt_;
  arm_.init(params.arm_params_);
  cds_controller_.init(params.cds_params_, arm_.get_pos_fk(q_init));
}

const KDL::JntArray& CDSArmController::update(const KDL::JntArray& q)
{
  assert(q.rows() == arm_.get_dof());
  assert(dt_ > 0.0);

  return arm_.get_vel_ik(q, cds_controller_.update(arm_.get_pos_fk(q)), dt_);
//  KDL::Frame pose = arm_.get_pos_fk(q);
//
//  cds_controller_.setCurrentEEPose(toMathLib(pose));
//
//  des_pose = toKDL(cds_controller_.getNextEEPose());
//
//  des_pose.M = arm_.get_pos_fk(q).M;
////  return arm_.get_vel_ik(q, toKDL(cds_controller_.getNextEEPose()), dt);
//  return arm_.get_vel_ik(q, des_pose, dt_);
}
