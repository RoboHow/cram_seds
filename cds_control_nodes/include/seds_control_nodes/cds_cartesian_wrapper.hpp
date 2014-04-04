#ifndef __CDS_CARTESIAN_WRAPPER_HPP
#define __CDS_CARTESIAN_WRAPPER_HPP

#include <CDSExecution.h>

class CDSExecutionParams
{
  public:
    GMRDynamics* master_dyn_;
    GMRDynamics* slave_dyn_;
    GMR* coupling_;
    double alpha_, beta_, lambda_, reachingThreshold_, dt_;
    KDL::Frame object_frame_, attractor_frame_;
    int slave_dynamics_id_;
};

class CdsCartesianWrapper
{
  public:
    CdsCartesianWrapper();
    CdsCartesianWrapper(CDSExecutionParams params, const KDL::Frame& pose_init);

    ~CdsCartesianWrapper();
 
    void init(CDSExecutionParams params, const KDL::Frame& pose_init);
    KDL::Frame update(const KDL::Frame& current_pose);

  private:
    CDSExecution cds_controller_;
};
#endif  // __CDS_CARTESIAN_WRAPPER_HPP
