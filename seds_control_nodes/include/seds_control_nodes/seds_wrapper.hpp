
#ifndef __SEDS_WRAPPER_HPP
#define __SEDS_WRAPPER_HPP

#include <string>
#include <kdl/chainfksolverpos_recursive.hpp>

class SedsWrapper {
private:

    KDL::ChainFkSolverPos_recursive *fk_solver_;
    unsigned int dof_;

    // memory for output values
    //KDL::JntArray& q_tmp_;
public:
    void parse_urdf(const std::string &param_name, const std::string &root_name,
        const std::string &tip_name);
    KDL::Frame get_fk(const KDL::JntArray& q);
    //KDL::JntArray& get_vel_ik(const KDL::JntArray& q, const KDL::Frame& des_pose);
};

#endif  // __SEDS_WRAPPER_HPP
