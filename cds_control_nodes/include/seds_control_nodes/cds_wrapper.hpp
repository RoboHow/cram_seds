#ifndef __CDS_WRAPPER_HPP
#define __CDS_WRAPPER_HPP

#include <seds_control_nodes/arm_kinematics.hpp>
#include <CDSExecution.h>

class CDSExecutionParams
{
  public:
    GMRDynamics* master_dyn_;
    GMRDynamics* slave_dyn_;
    GMR* coupling_;
    double alpha_, beta_, lambda_, reachingThreshold_, dt_;
    KDL::Frame object_frame_, attractor_frame_;
};

class CdsWrapperParams
{
  public:
    ArmKinematicsParams arm_params_;
    CDSExecutionParams cds_params_;
};

class CdsWrapper
{
  public:
    CdsWrapper();
    CdsWrapper(CdsWrapperParams params, const KDL::JntArray& q_init);

    ~CdsWrapper();

    void init(CdsWrapperParams params, const KDL::JntArray& q_init);
    const KDL::JntArray& update(const KDL::JntArray& q, double dt, KDL::Frame& des_pose);

//  private:
    ArmKinematics arm_;
    CDSExecution cds_controller_;
};
#endif  // __CDS_WRAPPER_HPP
