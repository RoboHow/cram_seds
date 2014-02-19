#ifndef __ARM_KINEMATICS_HPP
#define __ARM_KINEMATICS_HPP

#include <string>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chain.hpp>
#include <urdf/model.h>
#include <Eigen/Core>

class ArmKinematics {
  private:
    // forward kinematics solver
    KDL::ChainFkSolverPos_recursive *fk_solver_;
    // velocity-resolved inverse kinematics solver
    KDL::ChainIkSolverVel_wdls *ik_solver_;
    // keeping track of the number of joints we have
    unsigned int dof_;

    // memory for temporary values
    KDL::JntArray q_tmp_;

    // some internal helper methods
    void deleteInternals();
    void digestIkErrorCode(int error_code) const;
    KDL::Chain readChainFromUrdf(const urdf::ModelInterface& robot_model,
        const std::string &root_name, const std::string &tip_name);
    void initInternals(const KDL::Chain& chain);

  public:
    ArmKinematics();
    ArmKinematics(const urdf::ModelInterface& robot_model,
        const std::string &root_name, const std::string &tip_name, double lambda,
        const Eigen::MatrixXd& task_weights, const Eigen::MatrixXd& joint_weights);
    
    ~ArmKinematics();

    void init(const urdf::ModelInterface& robot_model,
        const std::string &root_name, const std::string &tip_name, double lambda,
        const Eigen::MatrixXd& task_weights, const Eigen::MatrixXd& joint_weights);

    void setLambda(double lambda);
    void setJointWeights(const Eigen::MatrixXd& joint_weights);
    void setTaskWeights(const Eigen::MatrixXd& task_weights);
    void setWeights(double lambda, const Eigen::MatrixXd& task_weights,
        const Eigen::MatrixXd& joint_weights);
 
    KDL::Frame get_pos_fk(const KDL::JntArray& q);
    KDL::JntArray& get_vel_ik(const KDL::JntArray& q, const KDL::Frame& des_pose,
        double dt);
};
#endif  // __ARM_KINEMATICS_HPP
