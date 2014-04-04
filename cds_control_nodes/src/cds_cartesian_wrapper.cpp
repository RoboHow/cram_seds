#include <seds_control_nodes/cds_cartesian_wrapper.hpp>
#include <seds_control_nodes/kdl_epfl_conversions.hpp>

CdsCartesianWrapper::CdsCartesianWrapper() :
    cds_controller_( CDSExecution() ) {}

CdsCartesianWrapper::CdsCartesianWrapper(CDSExecutionParams params,
    const KDL::Frame& pose_init)
{
  this->init(params, pose_init);
}

CdsCartesianWrapper::~CdsCartesianWrapper() {}

void CdsCartesianWrapper::init(CDSExecutionParams params,
    const KDL::Frame& pose_init)
{
  cds_controller_.setObjectFrame(toMathLib(params.object_frame_));

  cds_controller_.setAttractorFrame(toMathLib(params.attractor_frame_));

  cds_controller_.setDT(params.dt_);

  cds_controller_.setCurrentEEPose(toMathLib(pose_init));

  cds_controller_.init(static_cast<GMRDynamics*>(params.master_dyn_),
      static_cast<GMRDynamics*>(params.slave_dyn_), 
      static_cast<GMR*>(params.coupling_));

  cds_controller_.setMotionParameters(params.alpha_, 
      params.beta_, params.lambda_, params.reachingThreshold_,
      params.slave_dynamics_id_); 

  cds_controller_.postInit();
}

KDL::Frame CdsCartesianWrapper::update(const KDL::Frame& current_pose)
{
  cds_controller_.setCurrentEEPose(toMathLib(current_pose));
  return toKDL(cds_controller_.getNextEEPose());
}
