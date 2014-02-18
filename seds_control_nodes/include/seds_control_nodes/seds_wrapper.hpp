
#ifndef __SEDS_WRAPPER_HPP
#define __SEDS_WRAPPER_HPP

#include <string>
#include <kdl/chainfksolverpos_recursive.hpp>

class SedsWrapper {
private:

    KDL::ChainFkSolverPos_recursive *fk_solver_;

public:
    void parse_urdf(const std::string &param_name, const std::string &root_name, const std::string &tip_name);
    void get_fk(void);

};


#endif  // __SEDS_WRAPPER_HPP
