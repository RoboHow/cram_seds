#ifndef __CDS_WRAPPER_HPP
#define __CDS_WRAPPER_HPP

#include <seds_control_nodes/arm_kinematics.hpp>
#include <CDSExecution.h>

class CDSExecutionParams
{
  public:
    GMRDynamics master_dyn_, slave_dyn_;
    GMR coupling_;
    double alpha_, beta_, lambda_, reachingThreshold_, dt_;
};

class CdsWrapperParams
{
  public:
    ArmKinematicsParams arm_params_;
    CDSExecutionParams cds_params_;
};

class CdsWrapper
{
  private:
    ArmKinematics arm_;
    CDSExecution cds_controller_;

  public:
    CdsWrapper();
    CdsWrapper(const CdsWrapperParams& params);

    ~CdsWrapper();

    void init(const CdsWrapperParams& params);
    const KDL::JntArray& update(const KDL::JntArray& q, double dt);
};
#endif  // __CDS_WRAPPER_HPP
