#ifndef __CDS_ARM_CONTROLLER_HPP
#define __CDS_ARM_CONTROLLER_HPP

#include <seds_control_nodes/cds_cartesian_wrapper.hpp>
#include <seds_control_nodes/arm_kinematics.hpp>

class CDSArmControllerParams
{
  public:
    ArmKinematicsParams arm_params_;
    CDSExecutionParams cds_params_;
};

class CDSArmController
{
  public:
    CDSArmController();
    CDSArmController(CDSArmControllerParams params, const KDL::JntArray& q_init);

    ~CDSArmController();

    void init(CDSArmControllerParams params, const KDL::JntArray& q_init);
    const KDL::JntArray& update(const KDL::JntArray& q);

  private:
    ArmKinematics arm_;
    CDSExecution cds_controller_;
    double dt_;
};
#endif  // __CDS_ARM_CONTROLLER_HPP
